#!/usr/bin/env python3
"""Test dello streamer contro una finta macchina Marlin su TCP."""
import os
import socket
import sys
import threading
import time
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "tools"))
import gcode_stream as gs  # noqa: E402


class FakeMarlin(threading.Thread):
    """Risponde 'ok' a ogni riga con un po' di ritardo; ricorda quante righe erano in volo."""

    def __init__(self, error_on=None, ok_delay=0.002):
        super().__init__(daemon=True)
        self.srv = socket.socket()
        self.srv.bind(("127.0.0.1", 0))
        self.srv.listen(1)
        self.port = self.srv.getsockname()[1]
        self.received = []
        self.max_unacked_bytes = 0
        self.error_on = error_on
        self.ok_delay = ok_delay

    def run(self):
        try:
            self._serve()
        finally:
            self.srv.close()

    def _serve(self):
        conn, _ = self.srv.accept()
        buf = b""
        pending = []   # byte ricevuti e non ancora confermati
        while True:
            try:
                data = conn.recv(256)
            except OSError:
                conn.close()
                return
            if not data:
                conn.close()
                return
            buf += data
            while b"\n" in buf:
                line, _, buf = buf.partition(b"\n")
                text = line.decode()
                self.received.append(text)
                pending.append(len(line) + 1)
                self.max_unacked_bytes = max(self.max_unacked_bytes, sum(pending) + len(buf))
                time.sleep(self.ok_delay)
                if self.error_on and text.startswith(self.error_on):
                    conn.sendall(b"Error:Printer halted. kill() called!\n")
                    continue
                if text.startswith("G28"):
                    conn.sendall(b"echo:busy: processing\n")
                pending.pop(0)
                conn.sendall(b"ok\n")


class StreamTests(unittest.TestCase):
    def run_stream(self, lines, **kw):
        fake = FakeMarlin(**kw)
        fake.start()
        link = gs.TcpLink("127.0.0.1:%d" % fake.port)
        st = gs.Streamer(link, window=4, rx_bytes=120, log=lambda *_: None)
        try:
            st.wait_ready(timeout=5)
            for l in lines:
                st.send(l)
            st.drain(timeout=10)
        finally:
            link.close()
        return fake, st

    def test_sends_everything_in_order_and_respects_buffer(self):
        lines = ["G1 X%d Y%d F3000" % (i % 90, (i * 7) % 90) for i in range(300)]
        fake, st = self.run_stream(lines)
        sent = [l for l in fake.received if l.startswith("G1")]
        self.assertEqual(sent, lines)
        self.assertEqual(fake.received[-1], "M400")
        self.assertLessEqual(fake.max_unacked_bytes, 128)   # buffer RX di Marlin
        self.assertEqual(st.inflight, [])

    def test_error_from_machine_aborts(self):
        with self.assertRaises(gs.LinkError):
            self.run_stream(["G28", "G1 X10 Y10", "G1 X20 Y20", "G1 X30 Y30"], error_on="G1 X20")

    def test_clean_strips_comments(self):
        self.assertEqual(gs.clean("G1 X10 ; muovi"), "G1 X10")
        self.assertEqual(gs.clean("(commento) G0 Y5"), "G0 Y5")
        self.assertEqual(gs.clean("   ; solo commento"), "")

    def test_dry_run(self):
        import io
        import tempfile
        from contextlib import redirect_stdout
        with tempfile.NamedTemporaryFile("w", suffix=".gcode", delete=False) as f:
            f.write("; titolo\nG28\n\nG1 X1 Y2 ; x\n")
            path = f.name
        try:
            out = io.StringIO()
            with redirect_stdout(out):
                self.assertEqual(gs.main([path, "--tcp", "x", "--dry-run", "--init", "M115"]), 0)
            self.assertEqual(out.getvalue().split(), ["M115", "G28", "G1", "X1", "Y2"])
        finally:
            os.unlink(path)


if __name__ == "__main__":
    unittest.main()
