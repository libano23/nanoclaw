#!/usr/bin/env python3
"""
Invia un file G-code al plotter SCARA e aspetta che finisca.

  USB   :  python3 tools/gcode_stream.py disegno.gcode --port /dev/ttyACM0
           python3 tools/gcode_stream.py disegno.gcode --port COM5 --baud 115200
  WiFi  :  python3 tools/gcode_stream.py disegno.gcode --tcp scara-plotter.local
           python3 tools/gcode_stream.py disegno.gcode --tcp 192.168.1.50:8888

Il protocollo è quello di Marlin: ogni riga riceve un "ok" quando entra nella coda. Si tengono in volo
al massimo --window righe e --rx-bytes byte (il buffer di ricezione di Marlin è 128 byte), così la coda
dei movimenti resta piena anche con la latenza del WiFi e i tratti corti non si fermano a scatti.

Ctrl-C ferma subito i movimenti (M410). Serve pyserial solo per --port:  pip install pyserial
"""
import argparse
import re
import socket
import sys
import time

DEFAULT_TCP_PORT = 8888


class LinkError(Exception):
    pass


class TcpLink:
    def __init__(self, target, timeout=10.0):
        host, _, port = target.partition(":")
        try:
            self.sock = socket.create_connection((host, int(port or DEFAULT_TCP_PORT)), timeout=timeout)
        except OSError as e:
            raise LinkError("connessione a %s non riuscita: %s" % (target, e))
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.sock.settimeout(0.05)
        self.buf = b""

    def write(self, data):
        try:
            self.sock.sendall(data)
        except OSError as e:
            raise LinkError("connessione persa: %s" % e)

    def read_line(self):
        """Una riga se disponibile, altrimenti None (attesa breve)."""
        while b"\n" not in self.buf:
            try:
                chunk = self.sock.recv(4096)
            except socket.timeout:
                return None
            except OSError as e:
                raise LinkError("connessione persa: %s" % e)
            if not chunk:
                raise LinkError("il bridge ha chiuso la connessione")
            self.buf += chunk
        line, _, self.buf = self.buf.partition(b"\n")
        return line.decode("ascii", "replace").strip()

    def close(self):
        try:
            self.sock.close()
        except OSError:
            pass


class SerialLink:
    def __init__(self, port, baud):
        try:
            import serial
        except ImportError:
            raise LinkError("per la porta USB serve pyserial:  pip install pyserial")
        try:
            self.ser = serial.Serial(port, baud, timeout=0.05)
        except (OSError, serial.SerialException) as e:
            raise LinkError("porta %s non apribile: %s" % (port, e))
        self.buf = b""

    def write(self, data):
        self.ser.write(data)

    def read_line(self):
        while b"\n" not in self.buf:
            chunk = self.ser.read(256)
            if not chunk:
                return None
            self.buf += chunk
        line, _, self.buf = self.buf.partition(b"\n")
        return line.decode("ascii", "replace").strip()

    def close(self):
        self.ser.close()


def clean(line):
    """Toglie commenti ';' e '( )' e spazi; restituisce '' per le righe vuote."""
    line = re.sub(r"\([^)]*\)", "", line.split(";", 1)[0])
    return line.strip()


def load_lines(path):
    with open(path, encoding="utf-8", errors="replace") as f:
        return [c for c in (clean(l) for l in f) if c]


class Streamer:
    def __init__(self, link, window=4, rx_bytes=120, verbose=False, log=print):
        self.link, self.window, self.rx_bytes, self.verbose, self.log = link, window, rx_bytes, verbose, log
        self.inflight = []          # lunghezza in byte di ogni riga in attesa di "ok"
        self.sent = 0

    def _handle(self, line):
        """Elabora una riga dalla macchina. Ritorna True se era un 'ok'."""
        if line.startswith("ok"):
            if self.inflight:
                self.inflight.pop(0)
            return True
        if line.startswith(("Error", "error")) or "Printer halted" in line:
            raise LinkError("errore dalla macchina: %s" % line)
        if self.verbose and line and not line.startswith("echo:busy"):
            self.log("< " + line)
        return False

    def pump(self, wait=0.0):
        end = time.monotonic() + wait
        while True:
            line = self.link.read_line()
            if line is None:
                if time.monotonic() >= end:
                    return
                continue
            self._handle(line)
            end = 0   # una riga letta: svuota quello che c'è e poi esci

    def wait_ready(self, timeout=20.0):
        """Via USB la scheda si riavvia all'apertura della porta: aspetta che risponda a M115."""
        deadline = time.monotonic() + timeout
        last_probe = 0.0
        while time.monotonic() < deadline:
            if time.monotonic() - last_probe > 1.0:
                self.link.write(b"M115\n")
                last_probe = time.monotonic()
            line = self.link.read_line()
            if line and line.startswith("ok"):
                return
        raise LinkError("la macchina non risponde (nessun 'ok' a M115 in %.0f s)" % timeout)

    def send(self, cmd, on_progress=None):
        data = (cmd + "\n").encode("ascii")
        while len(self.inflight) >= self.window or sum(self.inflight) + len(data) > self.rx_bytes:
            line = self.link.read_line()
            if line is not None:
                self._handle(line)
        self.link.write(data)
        self.inflight.append(len(data))
        self.sent += 1
        if self.verbose:
            self.log("> " + cmd)
        if on_progress:
            on_progress(self.sent)

    def drain(self, timeout=600.0):
        """Aspetta tutti gli 'ok' e la fine dei movimenti (M400)."""
        self.send("M400")
        deadline = time.monotonic() + timeout
        while self.inflight:
            if time.monotonic() > deadline:
                raise LinkError("timeout in attesa della fine dei movimenti")
            line = self.link.read_line()
            if line is not None:
                self._handle(line)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("file", nargs="?", help="file G-code da inviare")
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--port", help="porta seriale USB (es. /dev/ttyACM0, COM5)")
    g.add_argument("--tcp", help="bridge ESP32: indirizzo[:porta]")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--window", type=int, default=4, help="righe in volo (BUFSIZE di Marlin = 4)")
    ap.add_argument("--rx-bytes", type=int, default=120, help="byte in volo (buffer RX di Marlin = 128)")
    ap.add_argument("--init", action="append", default=[], metavar="GCODE",
                    help="riga da inviare prima del file, ripetibile (es. --init G28)")
    ap.add_argument("--dry-run", action="store_true", help="legge il file e stampa le righe senza collegarsi")
    ap.add_argument("-v", "--verbose", action="store_true", help="mostra le risposte della macchina")
    a = ap.parse_args(argv)

    lines = load_lines(a.file) if a.file else []
    if a.dry_run:
        for l in a.init + lines:
            print(l)
        print("; %d righe" % (len(a.init) + len(lines)), file=sys.stderr)
        return 0
    if not a.file and not a.init:
        ap.error("indica un file G-code o almeno una riga con --init")

    link = None
    try:
        link = TcpLink(a.tcp) if a.tcp else SerialLink(a.port, a.baud)
        st = Streamer(link, a.window, a.rx_bytes, a.verbose)
        print("Collegamento…")
        st.wait_ready()
        todo = a.init + lines
        t0 = time.monotonic()
        total = len(todo)

        def progress(n):
            if n % 200 == 0 or n == total:
                print("  %d / %d righe  (%.0f s)" % (n, total, time.monotonic() - t0))

        try:
            for cmd in todo:
                st.send(cmd, progress)
            st.drain()
        except KeyboardInterrupt:
            print("\nInterrotto: fermo i movimenti (M410)")
            link.write(b"M410\n")
            return 130
        print("Finito: %d righe in %.1f s" % (total, time.monotonic() - t0))
        return 0
    except LinkError as e:
        print("Errore:", e, file=sys.stderr)
        return 1
    finally:
        if link:
            link.close()


if __name__ == "__main__":
    sys.exit(main())
