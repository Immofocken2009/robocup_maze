"""Tests für Motors.float, die Kalibrierprogramme und call.py."""
import io
import json
import sys
import tempfile
import types
import unittest
from contextlib import redirect_stdout
from pathlib import Path
from unittest import mock

import call
import config
import hardware.motors
from hardware.motors import Motors
from hardware.ultrasonic import load_calibration
from tools.calibrate_color import saved_colors
from tools.calibrate_ultrasonic import fit_line
from tests.fakes import FakeBrickPi3


class MotorsFloatTest(unittest.TestCase):
    def make(self, lose: int) -> Motors:
        FakeBrickPi3.lose = 0
        with mock.patch.object(hardware.motors, "brickpi3", types.SimpleNamespace(BrickPi3=FakeBrickPi3)):
            motors = Motors()
        motors.bp.power = {p: 50 for p in motors.bp.power}       # beide Motoren laufen
        motors.bp.sent.clear()
        FakeBrickPi3.lose = lose
        return motors

    def test_both_ports_in_one_message(self):
        motors = self.make(lose=0)
        motors.float()
        both = sum(motors.ports.values())
        self.assertEqual(motors.bp.sent, [(both, FakeBrickPi3.MOTOR_FLOAT)])

    def test_lost_message_is_repeated(self):
        motors = self.make(lose=2)
        motors.float()
        self.assertEqual(len(motors.bp.sent), 3)
        self.assertTrue(all(motors.bp.power[p] == FakeBrickPi3.MOTOR_FLOAT for p in motors.ports.values()))

    def test_warning_when_never_floating(self):
        motors = self.make(lose=99)
        with self.assertLogs("motors", "WARNING"):
            motors.float()
        self.assertEqual(len(motors.bp.sent), config.MOTOR_FLOAT_TRIES)


class UltrasonicCalibrationTest(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.TemporaryDirectory()
        self.path = Path(self.dir.name) / "ultrasonic.json"

    def tearDown(self):
        self.dir.cleanup()

    def test_fit_line(self):
        points = [(m, 0.98 * m - 0.7) for m in (5.6, 10.8, 21.1, 41.5)]
        factor, offset, error = fit_line(points)
        self.assertAlmostEqual(factor, 0.98, places=6)
        self.assertAlmostEqual(offset, -0.7, places=6)
        self.assertAlmostEqual(error, 0.0, places=6)

    def test_missing_file_warns(self):
        values, warning = load_calibration(self.path)
        self.assertIsNone(values)
        self.assertIn("nicht kalibriert", warning)

    def test_load(self):
        data = {side: {"factor": 1.01, "offset": -0.5} for side in ("front", "left", "right")}
        self.path.write_text(json.dumps(data))
        values, warning = load_calibration(self.path)
        self.assertEqual(values["left"], (1.01, -0.5))
        self.assertIsNone(warning)

    def test_partial_and_broken(self):
        self.path.write_text(json.dumps({"front": {"factor": 1.0, "offset": 0.2}}))
        values, warning = load_calibration(self.path)
        self.assertEqual(set(values), {"front"})
        self.assertIn("links, rechts", warning)
        self.path.write_text("{kaputt")
        values, warning = load_calibration(self.path)
        self.assertIsNone(values)
        self.assertIn("unlesbar", warning)

    def test_without_file_no_warning(self):
        self.assertEqual(load_calibration(None), (None, None))


class ColorCalibrationTest(unittest.TestCase):
    def test_saved_colors_only_with_same_settings(self):
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / "colors.json"
            self.assertIsNone(saved_colors(path))
            settings = {"integration_ms": config.COLOR_INTEGRATION_MS, "gain": config.COLOR_GAIN}
            path.write_text(json.dumps({"settings": settings, "colors": {"white": [1, 2, 3, 4]}}))
            self.assertEqual(saved_colors(path), {"white": (1, 2, 3, 4)})
            path.write_text(json.dumps({"settings": {**settings, "gain": -1}, "colors": {"white": [1, 2, 3, 4]}}))
            self.assertIsNone(saved_colors(path))


class CallTest(unittest.TestCase):
    """call.py ruft die richtigen Kalibrierungen auf (die Kalibrierungen selbst sind nachgebaut)."""

    def call(self, *argv, service=False):
        with mock.patch.object(sys, "argv", ["call.py", *argv]), \
                mock.patch.object(call, "service_running", return_value=service), \
                mock.patch.object(call.calibrate_ultrasonic, "run") as us, \
                mock.patch.object(call.calibrate_color, "run") as rgb, redirect_stdout(io.StringIO()):
            call.main()
        return us, rgb

    def test_ultrasonic_sides(self):
        us, rgb = self.call("--ultrasonicR", "--ultrasonicL")
        us.assert_called_once_with(["left", "right"])
        rgb.assert_not_called()

    def test_rgb_all_or_some(self):
        _, rgb = self.call("--rgb")
        rgb.assert_called_once_with(None)
        _, rgb = self.call("--rgb", "Silber", "weiss")
        rgb.assert_called_once_with(["silver", "white"])

    def test_refuses_while_service_runs(self):
        us, rgb = self.call("--rgb", "--ultrasonicV", service=True)
        us.assert_not_called()
        rgb.assert_not_called()

