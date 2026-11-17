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
from hardware.color_sensor import STATS_KEY, build_calibration, classify, features
from hardware.color_sensor import load_calibration as load_color_calibration
from hardware.ultrasonic import Ultrasonic, load_calibration
import tools.calibrate_color as calibrate_color
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

    def test_unreadable_file_gives_warning_instead_of_exception(self):
        """Kaputte colors.json: Warnung, ohne Kalibrierung weiterfahren (sonst startet der Roboter nie)."""
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / "colors.json"
            for text in ("{kaputt", '{"settings": {}}', '{"colors": 5}', '{"colors": {"white": 7}}'):
                path.write_text(text)
                colors, warning = load_color_calibration(path)
                self.assertIsNone(colors, text)
                self.assertIn("unlesbar", warning)
            settings = {"integration_ms": config.COLOR_INTEGRATION_MS, "gain": config.COLOR_GAIN}
            colors = {"white": [1, 2, 3, 4], "black": [0, 0, 0, 1]}
            expected = {"white": (1, 2, 3, 4), "black": (0, 0, 0, 1)}
            # gültig, aber ohne Streuung (alte Kalibrierung): läuft mit dem alten Verfahren, Warnung zum Neueinmessen
            path.write_text(json.dumps({"settings": settings, "colors": colors}))
            self.assertEqual(load_color_calibration(path), (expected, "Farbkalibrierung ohne Streuung, bitte neu einmessen"))
            # gültig mit Streuung: keine Warnung
            stats = {n: {"mean": [0.1, 0.2, 0.3, 1.0], "std": [0.01] * 4} for n in colors}
            path.write_text(json.dumps({"settings": settings, "colors": colors, "stats": stats}))
            cal, warning = load_color_calibration(path)
            self.assertIsNone(warning)
            self.assertEqual({n: v for n, v in cal.items() if n != STATS_KEY}, expected)
            self.assertIn(STATS_KEY, cal)


RAW_COLORS = {"white": (900, 850, 700, 2500), "black": (60, 60, 50, 170), "blue": (150, 300, 600, 1000),
              "silver": (1300, 1250, 1050, 3700), "red": (900, 250, 200, 1300)}


def color_stats(spread: float = 0.01) -> dict:
    """Merkmalsstatistik der RAW_COLORS mit gleicher kleiner Streuung für alle Merkmale."""
    white_c = RAW_COLORS["white"][3]
    return {n: {"mean": list(features(r, white_c)), "std": [spread] * 4} for n, r in RAW_COLORS.items()}


class ColorClassifyTest(unittest.TestCase):
    def setUp(self):
        self.cal = build_calibration(RAW_COLORS, color_stats())

    def test_with_stats(self):
        self.assertIn(STATS_KEY, self.cal)
        self.assertEqual(classify((880, 830, 690, 2450), self.cal), "white")
        self.assertEqual(classify((50, 55, 45, 150), self.cal), "black")
        self.assertEqual(classify((140, 280, 620, 1050), self.cal), "blue")
        self.assertEqual(classify((950, 260, 190, 1350), self.cal), "red")

    def test_rejects_far_value(self):
        # grün: Farbanteile passen zu keiner Farbe
        self.assertIsNone(classify((100, 1000, 100, 1500), self.cal))

    def test_brighter_white_stays_white(self):
        # 10 % heller als Weiß bleibt Weiß (Helligkeit: Mindeststreuung 0,05, Silber liegt bei 1,48 relativ)
        self.assertEqual(classify((990, 935, 770, 2750), self.cal), "white")
        self.assertEqual(classify((1300, 1250, 1050, 3700), self.cal), "silver")
        # sehr viel heller als alles Kalibrierte: unsicher
        self.assertIsNone(classify((2600, 2450, 2000, 7400), self.cal))

    def test_wide_spread_of_white_covers_brighter(self):
        stats = color_stats()
        stats["white"]["std"] = [0.01, 0.01, 0.01, 0.15]
        cal = build_calibration(RAW_COLORS, stats)
        self.assertEqual(classify((1100, 1040, 860, 3050), cal), "white")

    def test_without_stats_old_method(self):
        cal = build_calibration(RAW_COLORS, None)
        self.assertNotIn(STATS_KEY, cal)
        self.assertEqual(classify((880, 830, 690, 2450), cal), "white")
        self.assertEqual(classify((100, 1000, 100, 1500), cal) is not None, True)   # nie None
        partial = color_stats()
        del partial["red"]
        self.assertNotIn(STATS_KEY, build_calibration(RAW_COLORS, partial))

    def test_min_std_floor(self):
        # Streuung 0 gemessen: Mindeststreuung verhindert unendlichen Abstand
        cal = build_calibration(RAW_COLORS, color_stats(0.0))
        self.assertEqual(classify((880, 830, 690, 2450), cal), "white")

    def test_load_calibration_file(self):
        settings = {"integration_ms": config.COLOR_INTEGRATION_MS, "gain": config.COLOR_GAIN}
        colors = {n: list(v) for n, v in RAW_COLORS.items()}
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / "colors.json"
            path.write_text(json.dumps({"settings": settings, "colors": colors}))
            cal, warning = load_color_calibration(path)
            self.assertIn("ohne Streuung", warning)
            self.assertNotIn(STATS_KEY, cal)
            path.write_text(json.dumps({"settings": settings, "colors": colors, "stats": color_stats()}))
            cal, warning = load_color_calibration(path)
            self.assertIsNone(warning)
            self.assertIn(STATS_KEY, cal)
            self.assertEqual(classify((50, 55, 45, 150), cal), "black")

    def test_write_and_read_back_with_fake_sensor(self):
        class FakeSensor:
            integration_s = 0.0
            calibration = None
            closed = False
            level = 1000
            n = 0

            def read_raw(self):
                self.n += 1
                return (self.level + self.n % 3, 100, 100, 2 * self.level)

            def close(self):
                self.closed = True

        sensor = FakeSensor()
        settings = {"integration_ms": config.COLOR_INTEGRATION_MS, "gain": config.COLOR_GAIN}
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / "colors.json"
            with mock.patch.object(config, "COLOR_CALIBRATION_FILE", path), \
                    mock.patch.object(config, "COLOR_CAL_SAMPLES", 6), \
                    mock.patch.object(calibrate_color, "ColorSensor", return_value=sensor), \
                    mock.patch.object(calibrate_color, "live"), \
                    mock.patch.object(calibrate_color.time, "sleep"), \
                    mock.patch("builtins.input", return_value=""), \
                    redirect_stdout(io.StringIO()):
                calibrate_color.run(None)                         # alle fünf
                data = json.loads(path.read_text())
                self.assertEqual(set(data["stats"]), set(config.COLOR_NAMES))
                self.assertEqual(data["settings"], settings)
                self.assertEqual(len(data["stats"]["white"]["mean"]), 4)
                self.assertEqual(len(data["colors"]["white"]), 4)
                self.assertGreater(data["stats"]["white"]["std"][0], 0.0)
                # nur Blau neu, mit anderer Helligkeit: die anderen Statistiken bleiben
                before = {n: json.dumps(v) for n, v in data["stats"].items() if n != "blue"}
                sensor.level = 400
                calibrate_color.run(["blue"])
                data2 = json.loads(path.read_text())
                self.assertEqual({n: json.dumps(v) for n, v in data2["stats"].items() if n != "blue"}, before)
                self.assertNotEqual(data2["stats"]["blue"], data["stats"]["blue"])
                self.assertEqual(data2["colors"]["blue"][3], 800)
            cal, warning = load_color_calibration(path)
            self.assertIsNone(warning)
            self.assertIn(STATS_KEY, cal)

    def test_merge_rescales_brightness_when_white_changes(self):
        colors, stats = calibrate_color.merge(
            RAW_COLORS, color_stats(), {"white": [(900, 850, 700, 1250)] * 3})    # Weiß halb so hell wie vorher
        self.assertEqual(colors["white"][3], 1250)
        self.assertAlmostEqual(stats["black"]["mean"][3], 2 * RAW_COLORS["black"][3] / 2500, places=4)
        self.assertAlmostEqual(stats["white"]["mean"][3], 1.0, places=4)

    def test_warn_close(self):
        stats = color_stats()
        stats["silver"]["mean"] = list(stats["white"]["mean"])
        stats["silver"]["mean"][3] += 0.02
        out = io.StringIO()
        with redirect_stdout(out):
            calibrate_color.warn_close(stats)
        self.assertIn("Weiß und Silber", out.getvalue())
        out = io.StringIO()
        with redirect_stdout(out):
            calibrate_color.warn_close(color_stats())
        self.assertEqual(out.getvalue(), "")


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


class MedianFastTest(unittest.TestCase):
    class Dummy:
        median_fast = Ultrasonic.median_fast
        median = Ultrasonic.median

        def __init__(self, values):
            self.values = list(values)
            self.last = {"front": None}
            self.calls = 0

        def measure(self, side):
            self.calls += 1
            return self.values.pop(0)

    def test_two_steady_values_are_enough(self):
        d = self.Dummy([10.0, 10.5, 99.0])
        self.assertAlmostEqual(d.median_fast("front"), 10.25)
        self.assertEqual(d.calls, 2)
        self.assertAlmostEqual(d.last["front"], 10.25)

    def test_unsteady_values_continue_to_median(self):
        d = self.Dummy([10.0, 14.0, 10.2])
        self.assertAlmostEqual(d.median_fast("front"), 10.2)
        self.assertEqual(d.calls, 3)

    def test_no_echo_follows_majority_rule(self):
        d = self.Dummy([None, None, 10.0])
        self.assertIsNone(d.median_fast("front"))
        self.assertEqual(d.calls, 3)
        d = self.Dummy([None, 10.0, 10.1])
        self.assertAlmostEqual(d.median_fast("front"), 10.05)
        d = self.Dummy([None, None, None])
        self.assertIsNone(d.median_fast("front"))

    def test_same_result_as_median_when_noisy(self):
        self.assertEqual(self.Dummy([5.0, 9.0, 7.0]).median_fast("front"), self.Dummy([5.0, 9.0, 7.0]).median("front"))
