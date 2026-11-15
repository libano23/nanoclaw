#!/usr/bin/env python3
"""Test del generatore (senza Marlin e senza hardware):  python3 -m unittest discover -s tests -v"""
import copy
import json
import math
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import prepare  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
D2R = math.pi / 180


def load(path):
    with open(path, encoding="utf-8") as f:
        return json.load(f)


def studio_ik(L1, L2, mount, x, y, sign):
    """IK come nello studio interattivo (index.html): base in (0,0), theta1 relativo al montaggio."""
    c = (x * x + y * y - L1 * L1 - L2 * L2) / (2 * L1 * L2)
    t2 = sign * math.acos(max(-1, min(1, c)))
    t1 = math.atan2(y, x) - math.atan2(L2 * math.sin(t2), L1 + L2 * math.cos(t2))
    return (t1 / D2R - mount, t2 / D2R)


class Geometry(unittest.TestCase):
    def machines(self):
        yield "machine", load(os.path.join(ROOT, "machine.json"))
        for name in sorted(os.listdir(os.path.join(HERE, "configs"))):
            yield name, load(os.path.join(HERE, "configs", name))

    def test_valid_configs_pass_the_check(self):
        for name, cfg in self.machines():
            with self.subTest(name):
                errors, _, _ = prepare.check_geometry(prepare.Machine(cfg))
                self.assertEqual(errors, [])

    def test_frame_matches_studio(self):
        """Un punto del foglio, convertito in coordinate G-code, dà gli stessi angoli dello studio."""
        for name, cfg in self.machines():
            m = prepare.Machine(cfg)
            f = cfg["foglio"]
            c, s = math.cos(m.prot * D2R), math.sin(m.prot * D2R)
            for fx, fy in ((0.1, 0.1), (0.5, 0.5), (0.9, 0.2), (0.2, 0.9), (0.95, 0.95)):
                lx, ly = (fx - 0.5) * m.pw, (fy - 0.5) * m.ph           # coordinate locali del foglio
                wx, wy = m.pc[0] + lx * c - ly * s, m.pc[1] + lx * s + ly * c  # mondo (base in 0,0)
                want = studio_ik(m.L1, m.L2, m.mount, wx, wy, m.sign)
                got = m.ik(fx * m.pw, fy * m.ph)                          # G-code: origine angolo foglio
                with self.subTest(name=name, p=(fx, fy)):
                    self.assertAlmostEqual(got[0], want[0], places=6)
                    self.assertAlmostEqual(got[1], want[1], places=6)

    def test_fk_inverts_ik(self):
        for name, cfg in self.machines():
            m = prepare.Machine(cfg)
            for x, y in ((10, 10), (m.pw - 10, 10), (m.pw / 2, m.ph / 2), (10, m.ph - 10)):
                q = m.ik(x, y)
                fx, fy = m.fk(*q)
                with self.subTest(name=name, p=(x, y)):
                    self.assertAlmostEqual(fx, x, places=6)
                    self.assertAlmostEqual(fy, y, places=6)

    def test_theta_stays_continuous_across_minus_x(self):
        """Il braccio non salta di 360 gradi quando il bersaglio attraversa l'asse -X."""
        cfg = load(os.path.join(ROOT, "machine.json"))
        cfg["rotazione_montaggio_deg"] = 180
        cfg["foglio"].update({"centro_mm": [-235, 0]})
        cfg["finecorsa_deg"]["theta1"] = [-100, 100]
        m = prepare.Machine(cfg)
        prev = None
        for i in range(0, 101):
            q = m.ik(10 + i * 2.0, m.ph / 2)
            if q and prev is not None:
                self.assertLess(abs(q[0] - prev), 20)
            prev = q[0] if q else prev


class Rejections(unittest.TestCase):
    def cfg(self):
        return load(os.path.join(ROOT, "machine.json"))

    def errors(self, cfg):
        return prepare.check_geometry(prepare.Machine(cfg))[0]

    def test_home_inside_work_range(self):
        cfg = self.cfg()
        cfg["hardware"]["homing"] = {"spalla": "min", "gomito": "max", "posa": {"theta1": -135, "theta2": 150}}
        self.assertTrue(any("homing" in e for e in self.errors(cfg)))

    def test_home_pose_wrong_elbow_side(self):
        cfg = self.cfg()
        cfg["hardware"]["homing"] = {"spalla": "min", "gomito": "min", "posa": {"theta1": -135, "theta2": -100}}
        self.assertTrue(any("verso del gomito" in e for e in self.errors(cfg)))

    def test_sheet_too_far(self):
        cfg = self.cfg()
        cfg["foglio"]["centro_mm"] = [0, 400]
        self.assertTrue(any("fuori zona sicura" in e for e in self.errors(cfg)))

    def test_auto_elbow_is_refused(self):
        cfg = self.cfg()
        cfg["gomito"] = "auto"
        with self.assertRaises(prepare.ConfigError):
            prepare.Machine(cfg)


class ConfFile(unittest.TestCase):
    def test_set_enable_disable(self):
        import tempfile
        with tempfile.NamedTemporaryFile("w", suffix=".h", delete=False) as f:
            f.write("//#define A 1\n#define B 2 // c\n  #define C\n * #define D 9\n")
            path = f.name
        try:
            c = prepare.Conf(path)
            c.set("A", 5)
            c.set("B", "x")
            c.disable("C")
            self.assertEqual(c.text, "#define A 5\n#define B x\n  //#define C\n * #define D 9\n")
            c.enable("C")
            self.assertIn("  #define C\n", c.text)
            with self.assertRaises(prepare.ConfigError):
                c.set("D", 1)    # compare solo dentro un commento: non va toccato
        finally:
            os.unlink(path)


if __name__ == "__main__":
    unittest.main()
