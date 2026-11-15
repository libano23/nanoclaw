#!/usr/bin/env python3
"""
Esegue il firmware compilato su un Mega2560 simulato (simavr) e controlla
homing e cinematica con un braccio SCARA virtuale.

  python3 tests/run_sim.py --elf build/Marlin/.pio/build/mega2560/firmware.elf
  python3 tests/run_sim.py --elf fw.elf --gcode tests/sim/basic.gcode --config machine.json

Richiede simavr: sudo apt install simavr libsimavr-dev libelf-dev gcc
"""
import argparse
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import json
import prepare  # noqa: E402

BIG = 1e9


def build_sim(out):
    src = os.path.join(HERE, "sim", "sim.c")
    subprocess.run(["gcc", "-O2", "-o", out, src, "-lsimavr", "-lelf", "-lm"], check=True)


def write_auto_gcode(path, m):
    """Homing, cornice del foglio a 10 mm dal bordo, diagonali e ritorno al centro."""
    w, h = m.work_area()
    i = 10
    pts = [(i, i), (w - i, i), (w - i, h - i), (i, h - i), (i, i), (w - i, h - i), (i, h - i), (w - i, i)]
    lines = ["G28", "M114", "G90", "M400", "G1 X%g Y%g F3000" % (w / 2, h / 2), "M400"]
    for k, (x, y) in enumerate(pts):
        lines += ["%s X%g Y%g F%d" % ("G0" if k == 0 else "G1", x, y, 6000), "M400"]
    lines += ["G0 X%g Y%g" % (w / 2, h / 2), "M400", "M114"]
    open(path, "w").write("\n".join(lines) + "\n")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--elf", required=True)
    ap.add_argument("--gcode", default=None, help="file G-code (default: prova generata dall'area di lavoro)")
    ap.add_argument("--config", default=os.path.join(os.path.dirname(HERE), "machine.json"))
    ap.add_argument("--sim", default=os.path.join(os.path.dirname(HERE), "build", "sim"))
    ap.add_argument("--quiet", action="store_true")
    a = ap.parse_args()

    m = prepare.Machine(json.load(open(a.config, encoding="utf-8")))
    if a.gcode is None:
        a.gcode = os.path.join(os.path.dirname(a.sim), "auto.gcode")
        write_auto_gcode(a.gcode, m)
    os.makedirs(os.path.dirname(a.sim), exist_ok=True)
    build_sim(a.sim)

    off = m.theta_off
    h1, h2 = m.home_pose()
    s1, s2 = m.steps_per_deg()
    # finecorsa fisici (nello spazio degli assi di Marlin)
    elo = [-BIG, -BIG]
    ehi = [BIG, BIG]
    if m.home1 == "min" or m.use_max:
        elo[0] = m.t1lo + off
    if m.home1 == "max" or m.use_max:
        ehi[0] = m.t1hi + off
    a_home, b_home = m.axes(h1, h2)
    if m.home2 == "min":
        elo[1] = b_home
    else:
        ehi[1] = b_home
    if m.rel and m.use_max:
        other = m.axes(0, m.t2hi if m.home2 == "min" else m.t2lo)[1]
        if m.home2 == "min":
            ehi[1] = other
        else:
            elo[1] = other
    # il braccio parte "da qualche parte" lontano dai finecorsa di homing
    startA = a_home + (25 if m.home1 == "min" else -25)
    startB = b_home + (20 if m.home2 == "min" else -20)
    args = [a.sim, a.elf, a.gcode, m.tower[0], m.tower[1], m.L1, m.L2, 1 if m.rel else 0, s1, s2,
            startA, startB, elo[0], ehi[0], elo[1], ehi[1], 1 if m.nc else 0, 1 if m.inv1 else 0, 1 if m.inv2 else 0]
    args = [str(x) for x in args]
    if not a.quiet:
        print("Avvio simulazione:", " ".join(args[3:]))
    r = subprocess.run(args, text=True, capture_output=True)
    out = r.stdout + r.stderr
    for line in out.splitlines():
        if a.quiet and not line.startswith(("CHECK", "FWPOS", "RISULTATO", "FATAL", "FALLITO")):
            continue
        print(line)
    return r.returncode


if __name__ == "__main__":
    sys.exit(main())
