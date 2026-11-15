#!/usr/bin/env python3
"""
Prepara l'albero di Marlin per il plotter SCARA.

  python3 prepare.py                   # usa machine.json, crea build/Marlin
  python3 prepare.py --check           # solo verifica la geometria, non tocca Marlin
  python3 prepare.py --config altro.json --out build/Marlin

Passi:
  1. legge machine.json (stessi campi dell'esportazione dello studio interattivo
     + la sezione "hardware") e controlla che il foglio stia in zona sicura;
  2. clona Marlin alla versione fissata (MARLIN_TAG) se manca;
  3. applica patches/marlin-*-scara-plotter.patch (gomito, homing dei giunti);
  4. riscrive Configuration.h / Configuration_adv.h con i valori calcolati.

Poi si compila con PlatformIO:  pio run -d build/Marlin -e mega2560 [-t upload]

Convenzioni (le stesse del modello interattivo):
  - theta1 = angolo della spalla rispetto alla direzione di montaggio, antiorario +
  - theta2 = angolo del gomito rispetto al braccio 1, antiorario +
  - il G-code ha l'origine nell'angolo in basso a sinistra del foglio, assi
    paralleli ai lati del foglio (il foglio ruotato si ruota insieme al G-code).
"""
import argparse
import json
import math
import os
import re
import subprocess
import sys

MARLIN_URL = "https://github.com/MarlinFirmware/Marlin.git"
MARLIN_TAG = "2.1.2.6"
HERE = os.path.dirname(os.path.abspath(__file__))
PATCH = os.path.join(HERE, "patches", "marlin-%s-scara-plotter.patch" % MARLIN_TAG)
MARKER = ".scara-plotter-build"

D2R = math.pi / 180.0
R2D = 180.0 / math.pi


class ConfigError(Exception):
    pass


# --------------------------------------------------------------------------- #
#  Modello della macchina
# --------------------------------------------------------------------------- #
class Machine:
    def __init__(self, cfg):
        try:
            self.L1 = float(cfg["bracci_mm"]["L1"])
            self.L2 = float(cfg["bracci_mm"]["L2"])
            self.base_r = float(cfg.get("ingombro_base_mm", 0))
            self.mount = float(cfg.get("rotazione_montaggio_deg", 0))
            gomito = cfg.get("gomito", "pos")
            fc = cfg["finecorsa_deg"]
            self.t1lo, self.t1hi = (float(v) for v in fc["theta1"])
            self.t2lo, self.t2hi = (float(v) for v in fc["theta2"])
            self.margin = float(cfg.get("margine_finecorsa_deg", 0))
            self.sing = float(cfg.get("angolo_minimo_gomito_deg", 0))
            f = cfg["foglio"]
            self.pw = float(f["larghezza_mm"])
            self.ph = float(f["altezza_mm"])
            self.pc = tuple(float(v) for v in f["centro_mm"])
            self.prot = float(f.get("rotazione_deg", 0))
            hw = cfg["hardware"]
        except (KeyError, TypeError, ValueError) as e:
            raise ConfigError("machine.json incompleto o con valori non numerici: %r" % (e,))

        if gomito not in ("pos", "neg"):
            raise ConfigError('"gomito" deve essere "pos" o "neg" (con "auto" il braccio dovrebbe ribaltarsi '
                              'a metà tratto: scegli un verso solo)')
        self.sign = 1.0 if gomito == "pos" else -1.0
        if self.L1 <= 0 or self.L2 <= 0:
            raise ConfigError("L1 e L2 devono essere positivi")
        if not (self.t1lo < self.t1hi and self.t2lo < self.t2hi):
            raise ConfigError("finecorsa: il minimo deve essere minore del massimo")
        if self.t1hi - self.t1lo >= 360:
            raise ConfigError("la corsa di theta1 deve essere minore di 360 gradi")
        if self.pw <= 0 or self.ph <= 0:
            raise ConfigError("dimensioni del foglio non valide")

        self.hw = hw
        self.driver = hw.get("driver", "A4988")
        self.steps = float(hw.get("passi_motore", 200))
        self.micro = float(hw.get("microstep", 16))
        self.ratio1 = float(hw.get("rapporto_spalla", 1))
        self.ratio2 = float(hw.get("rapporto_gomito", 1))
        self.elbow_motor = hw.get("motore_gomito", "base")
        if self.elbow_motor not in ("base", "giunto"):
            raise ConfigError('"motore_gomito" deve essere "base" o "giunto"')
        inv = hw.get("inverti_direzione", {})
        self.inv1 = bool(inv.get("spalla", False))
        self.inv2 = bool(inv.get("gomito", False))
        hm = hw.get("homing", {})
        self.home1 = hm.get("spalla", "min")
        self.home2 = hm.get("gomito", "max")
        for v in (self.home1, self.home2):
            if v not in ("min", "max"):
                raise ConfigError('homing: usa "min" o "max"')
        self.park = hm.get("parcheggio", "centro")
        if self.park not in ("centro", "nessuno"):
            raise ConfigError('homing.parcheggio: "centro" oppure "nessuno"')
        self.home_override = hm.get("posa")
        if self.home_override is not None:
            try:
                self.home_override = (float(self.home_override["theta1"]), float(self.home_override["theta2"]))
            except (KeyError, TypeError, ValueError):
                raise ConfigError('homing.posa deve essere {"theta1": gradi, "theta2": gradi}')
        self.nc = bool(hw.get("finecorsa_normalmente_chiusi", True))
        self.use_max = bool(hw.get("finecorsa_massimo_attivo", True))
        self.vmax = float(hw.get("velocita_max_giunto_deg_s", 90))
        self.amax = float(hw.get("accelerazione_giunto_deg_s2", 300))
        self.adraw = float(hw.get("accelerazione_disegno_mm_s2", 500))
        self.vhome = float(hw.get("velocita_homing_deg_s", 20))
        self.segs = int(hw.get("segmenti_al_secondo", 100))
        ser = hw.get("seriale", {})
        self.usb_baud = int(ser.get("usb_baud", 115200))
        self.esp_port = ser.get("esp32_porta", 1)
        self.esp_baud = int(ser.get("esp32_baud", 115200))
        if self.esp_port not in (None, 1, 2, 3):
            raise ConfigError("esp32_porta: 1, 2, 3 oppure null")

        self.rel = self.elbow_motor == "giunto"
        self._frame()

    # -- cornice G-code -------------------------------------------------- #
    def _frame(self):
        c, s = math.cos(-self.prot * D2R), math.sin(-self.prot * D2R)
        self._rot_inv = (c, s)
        self.tower = self.world_to_g(0.0, 0.0)
        self.theta_off = self.mount - self.prot          # theta_marlin = theta1 + theta_off
        self.theta_center = self.theta_off + (self.t1lo + self.t1hi) / 2.0

    def world_to_g(self, x, y):
        c, s = self._rot_inv
        dx, dy = x - self.pc[0], y - self.pc[1]
        return (dx * c - dy * s + self.pw / 2.0, dx * s + dy * c + self.ph / 2.0)

    # -- cinematica come in Marlin (scara.cpp) --------------------------- #
    def ik(self, x, y):
        """Cartesiano G-code -> (theta1, theta2) del modello, oppure None se irraggiungibile.
        Replica inverse_kinematics() di Marlin, poi toglie gli offset."""
        sx, sy = x - self.tower[0], y - self.tower[1]
        h2 = sx * sx + sy * sy
        c2 = (h2 - (self.L1 ** 2 + self.L2 ** 2)) / (2 * self.L1 * self.L2)
        if c2 > 1 or c2 < -1:
            return None
        s2 = self.sign * math.sqrt(1 - c2 * c2)
        sk1 = self.L1 + self.L2 * c2
        sk2 = self.L2 * s2
        theta = math.atan2(sk1, sk2) - math.atan2(sx, sy)
        psi = math.atan2(s2, c2)
        a = theta * R2D
        a -= 360.0 * math.floor((a - self.theta_center + 180.0) / 360.0)
        return (a - self.theta_off, psi * R2D)

    def fk(self, t1, t2):
        a1 = (t1 + self.theta_off) * D2R
        a2 = a1 + t2 * D2R
        return (self.tower[0] + self.L1 * math.cos(a1) + self.L2 * math.cos(a2),
                self.tower[1] + self.L1 * math.sin(a1) + self.L2 * math.sin(a2))

    def axes(self, t1, t2):
        """Valori che Marlin chiama A (X) e B (Y), in gradi."""
        a = t1 + self.theta_off
        return (a, t2 if self.rel else a + t2)

    # -- pose di homing -------------------------------------------------- #
    def home_pose(self):
        """Posa del braccio quando i due finecorsa di homing scattano (theta1, theta2 del modello)."""
        if self.home_override is not None:
            return self.home_override
        t1 = self.t1lo if self.home1 == "min" else self.t1hi
        t2 = self.t2lo if self.home2 == "min" else self.t2hi
        return t1, t2

    def steps_per_deg(self):
        base = self.steps * self.micro / 360.0
        return base * self.ratio1, base * self.ratio2

    def work_area(self):
        return int(math.ceil(self.pw)), int(math.ceil(self.ph))


# --------------------------------------------------------------------------- #
#  Verifiche geometriche
# --------------------------------------------------------------------------- #
def check_geometry(m):
    errors, warnings, info = [], [], []

    # posa di homing
    h1, h2 = m.home_pose()
    if not (m.t1lo <= h1 <= m.t1hi and m.t2lo <= h2 <= m.t2hi):
        errors.append("posa di homing (theta1 %.1f°, theta2 %.1f°) fuori dai finecorsa impostati" % (h1, h2))
    if m.sign * h2 <= 0 or abs(h2) >= 180:
        errors.append("posa di homing: theta2 = %.1f° non è nel verso del gomito scelto (%s). Marlin la "
                      "ricalcolerebbe dal lato opposto e la posizione iniziale sarebbe sbagliata. "
                      "Usa l'altro finecorsa del gomito in hardware.homing.gomito."
                      % (h2, "theta2>0" if m.sign > 0 else "theta2<0"))
    info.append("posa di homing: theta1 %.1f°, theta2 %.1f°" % (h1, h2))

    # il foglio deve stare in zona sicura (stesse regole dello studio)
    nx, ny = max(2, int(m.pw / 5)), max(2, int(m.ph / 5))
    bad, total = 0, 0
    t1_seen, t2_seen = [], []
    ax_a, ax_b = [], []
    worst = {}
    for j in range(ny + 1):
        for i in range(nx + 1):
            x, y = m.pw * i / nx, m.ph * j / ny
            total += 1
            q = m.ik(x, y)
            reasons = []
            if q is None:
                reasons.append("fuori portata")
            else:
                t1, t2 = q
                t1_seen.append(t1)
                t2_seen.append(t2)
                aa, bb = m.axes(t1, t2)
                ax_a.append(aa)
                ax_b.append(bb)
                if t1 < m.t1lo + m.margin or t1 > m.t1hi - m.margin:
                    reasons.append("theta1 oltre il margine dei finecorsa")
                if m.sign > 0 and not (max(m.sing, m.t2lo + m.margin) <= t2 <= m.t2hi - m.margin):
                    reasons.append("theta2 fuori dall'intervallo sicuro")
                if m.sign < 0 and not (m.t2lo + m.margin <= t2 <= min(-m.sing, m.t2hi - m.margin)):
                    reasons.append("theta2 fuori dall'intervallo sicuro")
                sx, sy = x - m.tower[0], y - m.tower[1]
                if math.hypot(sx, sy) < m.base_r:
                    reasons.append("dentro l'ingombro della base")
            if reasons:
                bad += 1
                for r in reasons:
                    worst.setdefault(r, (x, y))
    if bad:
        errors.append("%d punti su %d del foglio sono fuori zona sicura: %s"
                      % (bad, total, "; ".join("%s (es. a X=%.0f Y=%.0f)" % (r, p[0], p[1]) for r, p in worst.items())))
    if t1_seen:
        info.append("angoli richiesti dal foglio: theta1 %.1f … %.1f°, theta2 %.1f … %.1f°"
                    % (min(t1_seen), max(t1_seen), min(t2_seen), max(t2_seen)))

    # il finecorsa di homing deve stare fuori dal campo di lavoro, dal lato giusto, con margine
    if ax_a:
        a_home, b_home = m.axes(h1, h2)
        for nome, home, side, lo, hi in (("X (spalla)", a_home, m.home1, min(ax_a), max(ax_a)),
                                         ("Y (gomito, %s)" % ("angolo relativo" if m.rel else "angolo assoluto psi+theta"),
                                          b_home, m.home2, min(ax_b), max(ax_b))):
            ok = home <= lo - m.margin if side == "min" else home >= hi + m.margin
            info.append("asse %s: serve %.1f … %.1f°, finecorsa di homing (%s) a %.1f°" % (nome, lo, hi, side, home))
            if not ok:
                errors.append("asse %s: il finecorsa di homing a %.1f° (%s) cade dentro il campo richiesto dal foglio "
                              "(%.1f … %.1f°, margine %.1f°): il braccio non potrebbe raggiungere tutto il foglio. "
                              "Cambia lato di homing o la posa (hardware.homing.posa)."
                              % (nome, home, side, lo, hi, m.margin))

    # raggio morto
    if m.sign > 0:
        psi_in = min(m.t2hi - m.margin, 179.9)
    else:
        psi_in = min(-(m.t2lo + m.margin), 179.9)
    r_in = math.sqrt(max(0.0, m.L1 ** 2 + m.L2 ** 2 + 2 * m.L1 * m.L2 * math.cos(psi_in * D2R)))
    m.dead_zone = int(math.floor(max(m.base_r, r_in)))
    info.append("raggio minimo raggiungibile %.0f mm (zona morta impostata a %d mm)" % (r_in, m.dead_zone))

    return errors, warnings, info


# --------------------------------------------------------------------------- #
#  Modifica dei file di configurazione di Marlin
# --------------------------------------------------------------------------- #
class Conf:
    def __init__(self, path):
        self.path = path
        with open(path, encoding="utf-8") as f:
            self.text = f.read()
        self.log = []

    @staticmethod
    def _rx(name):
        return re.compile(r"^([ \t]*)(//)?[ \t]*#define[ \t]+" + re.escape(name) + r"\b[^\n]*$", re.M)

    def set(self, name, value):
        """Imposta '#define NAME value' (decommenta se serve). Tutte le occorrenze di riga."""
        rx = self._rx(name)
        n = len(rx.findall(self.text))
        if not n:
            raise ConfigError("%s: #define %s non trovato" % (os.path.basename(self.path), name))
        val = "" if value is None else " " + str(value)
        self.text = rx.sub(lambda mo: "%s#define %s%s" % (mo.group(1), name, val), self.text)
        self.log.append("set %s%s" % (name, val))

    def enable(self, name):
        rx = self._rx(name)
        if not rx.search(self.text):
            raise ConfigError("%s: #define %s non trovato" % (os.path.basename(self.path), name))
        self.text = rx.sub(lambda mo: mo.group(0).replace("//", "", 1) if mo.group(2) else mo.group(0), self.text)
        self.log.append("enable %s" % name)

    def disable(self, name):
        rx = self._rx(name)
        if not rx.search(self.text):
            raise ConfigError("%s: #define %s non trovato" % (os.path.basename(self.path), name))
        self.text = rx.sub(lambda mo: mo.group(0) if mo.group(2) else "%s//%s" % (mo.group(1), mo.group(0)[len(mo.group(1)):]),
                           self.text)
        self.log.append("disable %s" % name)

    def after(self, anchor, block):
        """Inserisce 'block' dopo la riga che contiene l'ancora (deve essere unica)."""
        rx = re.compile(r"^[^\n]*" + re.escape(anchor) + r"[^\n]*\n", re.M)
        found = rx.findall(self.text)
        if len(found) != 1:
            raise ConfigError("%s: ancora '%s' trovata %d volte" % (os.path.basename(self.path), anchor, len(found)))
        self.text = rx.sub(lambda mo: mo.group(0) + block, self.text, count=1)
        self.log.append("insert after %s" % anchor)

    def save(self):
        with open(self.path, "w", encoding="utf-8", newline="\n") as f:
            f.write(self.text)


def fmt(v, nd=4):
    s = ("%." + str(nd) + "f") % v
    return s.rstrip("0").rstrip(".") if "." in s else s


def apply_config(m, marlin_dir):
    cfg = Conf(os.path.join(marlin_dir, "Marlin", "Configuration.h"))
    adv = Conf(os.path.join(marlin_dir, "Marlin", "Configuration_adv.h"))

    # --- macchina ---------------------------------------------------------
    cfg.set("CUSTOM_MACHINE_NAME", '"SCARA Plotter"')
    cfg.set("MOTHERBOARD", "BOARD_RAMPS_14_EFB")
    cfg.set("SERIAL_PORT", 0)
    cfg.set("BAUDRATE", m.usb_baud)
    if m.esp_port is not None:
        cfg.set("SERIAL_PORT_2", m.esp_port)
        cfg.set("BAUDRATE_2", m.esp_baud)
    cfg.set("EXTRUDERS", 0)
    cfg.set("TEMP_SENSOR_0", 0)
    cfg.set("TEMP_SENSOR_BED", 0)
    cfg.set("X_DRIVER_TYPE", m.driver)
    cfg.set("Y_DRIVER_TYPE", m.driver)
    cfg.set("Z_DRIVER_TYPE", m.driver)   # asse Z presente ma inutilizzato: la penna è un servo (M280)

    # --- cinematica -------------------------------------------------------
    cfg.enable("MORGAN_SCARA")
    cfg.set("SCARA_LINKAGE_1", fmt(m.L1))
    cfg.set("SCARA_LINKAGE_2", fmt(m.L2))
    cfg.set("SCARA_OFFSET_X", fmt(m.tower[0]))
    cfg.set("SCARA_OFFSET_Y", fmt(m.tower[1]))
    cfg.set("DEFAULT_SEGMENTS_PER_SECOND", m.segs)
    cfg.set("MIDDLE_DEAD_ZONE_R", m.dead_zone)
    x_home, y_home = m.axes(*m.home_pose())
    travel1 = 1.25 * (m.t1hi - m.t1lo) + 10
    travel2 = 1.25 * ((m.t2hi - m.t2lo) + (0 if m.rel else (m.t1hi - m.t1lo))) + 10
    block = ["  // --- Plotter SCARA (generato da prepare.py) ---\n",
             "  #define SCARA_JOINT_HOMING\n",
             "  #define SCARA_THETA_CENTER %s\n" % fmt(m.theta_center, 3),
             "  #define SCARA_HOMING_TRAVEL { %s, %s }  // (deg) corsa massima cercando il finecorsa\n"
             % (fmt(travel1, 1), fmt(travel2, 1))]
    if m.park == "centro":
        block.append("  #define SCARA_HOME_PARK { %s, %s }  // (mm) a fine G28 il braccio va al centro del foglio\n"
                     % (fmt(m.pw / 2.0, 2), fmt(m.ph / 2.0, 2)))
    if m.rel:
        block.append("  #define SCARA_ELBOW_RELATIVE\n")
    if m.sign < 0:
        block.append("  #define SCARA_ELBOW_NEGATIVE\n")
    cfg.after("#define SCARA_OFFSET_Y", "".join(block))

    # --- area di lavoro (cornice del foglio) ------------------------------
    w, h = m.work_area()
    cfg.set("X_BED_SIZE", w)
    cfg.set("Y_BED_SIZE", h)
    cfg.set("X_MIN_POS", 0)
    cfg.set("Y_MIN_POS", 0)
    cfg.set("X_MAX_POS", w)
    cfg.set("Y_MAX_POS", h)
    cfg.set("MANUAL_X_HOME_POS", fmt(x_home, 3))   # per Morgan sono ANGOLI dei giunti (gradi)
    cfg.set("MANUAL_Y_HOME_POS", fmt(y_home, 3))
    cfg.set("MANUAL_Z_HOME_POS", 0)
    cfg.set("Z_HOME_DIR", 0)               # Z non si azzera: nessun finecorsa (i pin 18/19 servono alla seriale 1)
    cfg.set("X_HOME_DIR", -1 if m.home1 == "min" else 1)
    cfg.set("Y_HOME_DIR", -1 if m.home2 == "min" else 1)
    cfg.set("INVERT_X_DIR", "true" if m.inv1 else "false")
    cfg.set("INVERT_Y_DIR", "true" if m.inv2 else "false")
    cfg.enable("NO_MOTION_BEFORE_HOMING")

    # --- finecorsa (RAMPS: X min 3, X max 2, Y min 14, Y max 15; Z liberi per la seriale 1) ---
    cfg.enable("USE_XMIN_PLUG")
    cfg.enable("USE_YMIN_PLUG")
    cfg.disable("USE_ZMIN_PLUG")
    if m.use_max:
        cfg.enable("USE_XMAX_PLUG")
        cfg.enable("USE_YMAX_PLUG")
    inv = "false" if m.nc else "true"      # Marlin: COM a massa + NC al segnale = false (caso più comune)
    for n in ("X_MIN", "X_MAX", "Y_MIN", "Y_MAX", "Z_MIN", "Z_MAX"):
        cfg.set(n + "_ENDSTOP_INVERTING", inv)

    # --- passi, velocità, accelerazioni ----------------------------------
    s1, s2 = m.steps_per_deg()
    cfg.set("DEFAULT_AXIS_STEPS_PER_UNIT", "{ %s, %s, 400 }" % (fmt(s1), fmt(s2)))
    cfg.set("DEFAULT_MAX_FEEDRATE", "{ %s, %s, 20 }" % (fmt(m.vmax, 1), fmt(m.vmax, 1)))
    cfg.set("DEFAULT_MAX_ACCELERATION", "{ %s, %s, 100 }" % (fmt(m.amax, 1), fmt(m.amax, 1)))
    cfg.set("DEFAULT_ACCELERATION", fmt(m.adraw, 1))
    cfg.set("DEFAULT_TRAVEL_ACCELERATION", fmt(m.adraw, 1))
    cfg.enable("CLASSIC_JERK")
    cfg.set("HOMING_FEEDRATE_MM_M", "{ (%s*60), (%s*60), (4*60) }" % (fmt(m.vhome, 1), fmt(m.vhome, 1)))

    # --- penna: servo su SERVO0 (pin D11 della RAMPS) ------------------------
    cfg.set("NUM_SERVOS", 1)

    # --- avanzate ------------------------------------------------------------
    adv.enable("EMERGENCY_PARSER")
    adv.set("AXIS_RELATIVE_MODES", "{ false, false, false }")   # senza estrusore: solo X Y Z
    adv.set("MIN_STEPS_PER_SEGMENT", 2)      # i segmenti più corti vengono scartati: con pochi passi/grado il disegno perde precisione
    adv.set("HOMING_BUMP_MM", "{ 5, 5, 2 }")
    adv.set("HOMING_BUMP_DIVISOR", "{ 2, 2, 4 }")
    adv.enable("ENDSTOPS_ALWAYS_ON_DEFAULT")

    cfg.save()
    adv.save()
    return cfg.log + adv.log


# --------------------------------------------------------------------------- #
#  Albero di Marlin
# --------------------------------------------------------------------------- #
def run(cmd, **kw):
    return subprocess.run(cmd, check=True, **kw)


def ensure_marlin(out):
    marker = os.path.join(out, MARKER)
    if os.path.isdir(out) and not os.path.exists(marker):
        raise ConfigError("%s esiste già e non è stato creato da questo script: scegli un'altra cartella con --out" % out)
    if os.path.isdir(out):
        # ripristina i file di Marlin prima di riapplicare patch e configurazione
        run(["git", "-C", out, "checkout", "--", "."])
        return
    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
    print("Scarico Marlin %s ..." % MARLIN_TAG)
    run(["git", "clone", "--quiet", "--depth", "1", "--branch", MARLIN_TAG, MARLIN_URL, out])
    open(os.path.join(out, MARKER), "w").write("generato da prepare.py\n")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default=os.path.join(HERE, "machine.json"))
    ap.add_argument("--out", default=os.path.join(HERE, "build", "Marlin"))
    ap.add_argument("--check", action="store_true", help="verifica solo la geometria")
    ap.add_argument("--force", action="store_true", help="prosegui anche con errori di geometria")
    args = ap.parse_args()

    try:
        with open(args.config, encoding="utf-8") as f:
            cfg = json.load(f)
        m = Machine(cfg)
    except (OSError, ValueError, ConfigError) as e:
        print("Errore:", e, file=sys.stderr)
        return 2

    errors, warnings, info = check_geometry(m)
    s1, s2 = m.steps_per_deg()
    w, h = m.work_area()
    tx, ty = m.tower
    print("Torre (spalla) in coordinate G-code: X=%.2f Y=%.2f mm" % (tx, ty))
    print("Area di lavoro G-code: X 0…%d, Y 0…%d mm (origine = angolo in basso a sinistra del foglio)" % (w, h))
    print("Passi per grado: spalla %.4f, gomito %.4f" % (s1, s2))
    a, b = m.axes(*m.home_pose())
    print("Home: asse X (theta) = %.2f°, asse Y (%s) = %.2f°" % (a, "psi relativo" if m.rel else "psi+theta", b))
    for line in info:
        print(" -", line)
    for line in warnings:
        print(" ! ", line)
    for line in errors:
        print(" ERRORE:", line)
    if errors and not args.force:
        print("\nCorreggi machine.json (o usa lo studio interattivo per trovare una configurazione valida), "
              "oppure --force per procedere comunque.", file=sys.stderr)
        return 1
    if args.check:
        return 0

    try:
        ensure_marlin(args.out)
        run(["git", "-C", args.out, "apply", PATCH])
        log = apply_config(m, args.out)
    except (ConfigError, subprocess.CalledProcessError, OSError) as e:
        print("Errore:", e, file=sys.stderr)
        return 3
    print("\nMarlin pronto in %s (%d modifiche di configurazione)." % (args.out, len(log)))
    print("Compila e carica:  pio run -d %s -e mega2560 -t upload" % os.path.relpath(args.out))
    return 0


if __name__ == "__main__":
    sys.exit(main())
