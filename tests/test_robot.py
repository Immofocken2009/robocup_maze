"""Tests für hardware/robot.py (und das Abschalten in hardware/motors.py, die Kalibrierprogramme, call.py)
mit nachgebauter Hardware (nichts bewegt sich).

Der nachgebaute Roboter fährt auf einer Geraden: x = Weg der Robotermitte in cm ab der Mitte
der Startkachel. Die Zeit läuft im Nachbau SPEEDUP-mal so schnell, damit die Tests kurz bleiben.
"""
import io
import json
import math
import sys
import tempfile
import time
import unittest
from contextlib import redirect_stdout
from pathlib import Path
from unittest import mock

import call
import config
import hardware.motors
from hardware.color_sensor import STATS_KEY, build_calibration, classify, features, load_calibration as load_color_calibration
from hardware.motors import Motors
from hardware.robot import (HeadingEstimate, centering_plan, LackOfProgress, RealRobot, RunStopped, SensorsBlind, Steering,
                            arrival_fix, arrival_target, fit_slope, stop_distance, stop_median)
from hardware.ultrasonic import Ultrasonic, load_calibration
from navigation.maze_map import TileType
from navigation.robot_interface import MoveStatus
import tools.calibrate_color as calibrate_color
from tools.calibrate_color import saved_colors
from tools.calibrate_ultrasonic import fit_line
from tools.test_motors import angle_from_offset, new_trim

SPEEDUP = 4.0
TILE = config.TILE_SIZE_CM
FACE = (TILE - config.WALL_THICKNESS_CM) / 2   # Wandfläche von der Kachelmitte


class Body:
    """Gerade Strecke mit einer Wand vorne (Abstand ihrer Fläche von der Mitte der Startkachel)."""

    def __init__(self, wall_cm: float | None, floors: list[TileType] | None = None):
        self.wall_cm = wall_cm
        self.floors = floors or [TileType.NORMAL] * 10
        self.x = 0.0
        self.furthest = 0.0

    def max_x(self) -> float:
        """Weiter kommt der Roboter nicht (Vorderkante an der Wand)."""
        return float("inf") if self.wall_cm is None else self.wall_cm - config.ROBOT_LENGTH_CM / 2


class FakeMotors:
    def __init__(self, body: Body):
        self.body = body
        self.pos = [0.0, 0.0]
        self.dps = [0.0, 0.0]
        self.t = time.monotonic()

    def _advance(self):
        now = time.monotonic()
        dt = (now - self.t) * SPEEDUP
        self.t = now
        step = [d * dt for d in self.dps]
        fwd = (step[0] + step[1]) / 2 / config.MOTOR_DEG_PER_CM
        if fwd > 0 and self.body.x + fwd > self.body.max_x():     # an der Wand: Ketten stehen
            step = [0.0, 0.0]
            fwd = 0.0
        self.pos = [p + s for p, s in zip(self.pos, step)]
        self.body.x += fwd
        self.body.furthest = max(self.body.furthest, self.body.x)

    def set_dps(self, left, right):
        self._advance()
        self.dps = [left, right]

    def positions(self):
        self._advance()
        return tuple(self.pos)

    def stop(self):
        self.set_dps(0, 0)

    float = stop


class FakeUltrasonic:
    def __init__(self, body: Body):
        self.body = body
        self.last = {"front": None, "left": None, "right": None}

    def ready(self):
        return True

    def measure(self, side):
        if side == "front":
            cm = None
            if self.body.wall_cm is not None:
                cm = self.body.wall_cm - self.body.x - (config.ROBOT_LENGTH_CM / 2 - config.US_FRONT_INSET_CM)
                cm = cm if cm <= config.US_MAX_DISTANCE_CM else None
        else:
            cm = config.SIDE_CENTER_CM
        self.last[side] = cm
        return cm

    def median(self, side, n=3):
        return self.measure(side)

    def median_fast(self, side, n=3):
        return self.measure(side)

    def read_all(self, n=1):
        return {side: self.measure(side) for side in self.last}


class FakeColor:
    integration_s = 0.0

    def __init__(self, body: Body):
        self.body = body
        self.last_raw = None
        self.last_color = None

    def read_tile(self):
        sensor = self.body.x + config.ROBOT_LENGTH_CM / 2 - config.COLOR_INSET_CM
        return self.body.floors[int((sensor + TILE / 2) // TILE)]

    def read_floor(self):
        return self.read_tile()

    def read_color(self):
        return None


def make_robot(body: Body) -> RealRobot:
    return RealRobot(FakeMotors(body), FakeUltrasonic(body), FakeColor(body))


class BlindUltrasonic(FakeUltrasonic):
    """Alle drei Sensoren ohne Echo (z.B. Versorgung weg)."""
    def measure(self, side):
        self.last[side] = None
        return None


class SensorsBlindTest(unittest.TestCase):
    def make(self, ultrasonic=BlindUltrasonic):
        body = Body(wall_cm=None)
        return RealRobot(FakeMotors(body), ultrasonic(body), FakeColor(body))

    def test_stops_after_repeated_blind_measurements(self):
        robot = self.make()
        robot.start_clock()
        robot.read_walls()                  # einmal ganz ohne Echo: noch kein Abbruch, aber Warnung
        self.assertTrue(robot.us_blind())
        with self.assertRaises(SensorsBlind):
            robot.read_walls()

    def test_one_sensor_with_echo_is_fine(self):
        robot = self.make(FakeUltrasonic)   # Seiten messen, vorne ohne Wand
        robot.start_clock()
        for _ in range(5):
            robot.read_walls()
        self.assertEqual(robot.blind_streak, 0)
        self.assertFalse(robot.us_blind())

    def test_streak_resets_with_echo(self):
        robot = self.make()
        robot.start_clock()
        robot.read_walls()
        robot.us = FakeUltrasonic(Body(wall_cm=None))
        robot.read_walls()
        robot.us = BlindUltrasonic(Body(wall_cm=None))
        robot.read_walls()                  # wieder nur 1 in Folge
        self.assertEqual(robot.blind_streak, 1)

    def test_warning_when_idle(self):
        robot = self.make()
        robot.poll_sensors()                # im Stand (Lauf nicht gestartet)
        self.assertTrue(robot.us_blind())

    def test_clock_with_and_without_calibration_time(self):
        robot = self.make(FakeUltrasonic)
        robot.start_clock()
        self.assertAlmostEqual(robot.time(), config.CALIBRATION_TIME_S, delta=0.5)
        robot.start_clock(0.0)
        self.assertAlmostEqual(robot.time(), 0.0, delta=0.5)

    def test_double_press_detection(self):
        import queue
        import threading
        import main
        program = main.RobotProgram.__new__(main.RobotProgram)
        program.presses = queue.Queue()
        with mock.patch.object(config, "START_DOUBLE_S", 0.3):
            self.assertFalse(program.double_press())                 # einmal K1: nichts kommt nach
            threading.Timer(0.1, program.presses.put, ["start"]).start()
            self.assertTrue(program.double_press())                  # zweites K1 schnell danach
            threading.Timer(0.1, program.presses.put, ["stop"]).start()
            self.assertFalse(program.double_press())                 # anderer Taster zählt nicht

    def test_competition_stops_with_message(self):
        from main import competition
        from navigation.explorer import Explorer
        robot = self.make()
        explorer = Explorer(robot)
        self.assertFalse(competition(robot, explorer, lambda: "stop"))
        self.assertIn("Ultraschall", explorer.mode)


class SettingsTest(unittest.IsolatedAsyncioTestCase):
    """Lenk-Einstellungen der Webseite (data/settings.json)."""

    def setUp(self):
        self.dir = tempfile.TemporaryDirectory()
        self.file = Path(self.dir.name) / "settings.json"
        for name in config.STEER_SETTINGS:      # config wird verändert: nachher zurücksetzen
            patcher = mock.patch.object(config, name, getattr(config, name))
            patcher.start()
            self.addCleanup(patcher.stop)
        self.addCleanup(self.dir.cleanup)

    def test_read_ignores_missing_file_unknown_names_and_bad_values(self):
        self.assertEqual(config.read_settings(self.file), {})
        self.file.write_text("kaputt")
        self.assertEqual(config.read_settings(self.file), {})
        self.file.write_text(json.dumps({"STEER_KP": 3, "STEER_KD": 99999, "STEER_MAX_DPS": "x",
                                         "STEER_SLEW_DPS_PER_CM": True, "ANDERES": 1}))
        self.assertEqual(config.read_settings(self.file), {"STEER_KP": 3.0})

    def test_write_and_read(self):
        config.write_settings({"STEER_KP": 4.5}, self.file)
        self.assertEqual(config.read_settings(self.file), {"STEER_KP": 4.5})
        self.assertFalse(self.file.with_name("settings.json.tmp").exists())

    def test_robot_applies_values_before_driving_and_restores_defaults(self):
        default_kd = config.STEER_KD
        config.write_settings({"STEER_KP": 2.0, "STEER_KD": 50.0}, self.file)
        body = Body(wall_cm=None)
        robot = RealRobot(FakeMotors(body), FakeUltrasonic(body), FakeColor(body), settings_file=self.file)
        robot.forward(set())
        self.assertEqual((config.STEER_KP, config.STEER_KD), (2.0, 50.0))
        config.write_settings({"STEER_KP": 3.0}, self.file)         # KD nicht mehr gesetzt = Standard
        robot.forward(set())
        self.assertEqual((config.STEER_KP, config.STEER_KD), (3.0, default_kd))

    def test_robot_without_file_keeps_config(self):
        body = Body(wall_cm=None)
        with mock.patch.object(config, "STEER_KP", 1.23):
            make_robot(body).forward(set())
            self.assertEqual(config.STEER_KP, 1.23)

    async def test_server_saves_only_differences_and_validates(self):
        from web import server

        class Request:
            def __init__(self, data):
                self.data = data

            async def json(self):
                return self.data

        read, write = config.read_settings, config.write_settings
        with mock.patch.object(config, "read_settings", lambda path=self.file: read(path)), \
                mock.patch.object(config, "write_settings", lambda values, path=self.file: write(values, path)):
            ok = await server.settings_save(Request({"values": {"STEER_KP": 9.0, "STEER_KD": config.STEER_KD}}))
            self.assertEqual(ok.status, 200)
            self.assertEqual(read(self.file), {"STEER_KP": 9.0})            # Standardwert wird nicht gespeichert
            shown = {s["name"]: s["value"] for s in json.loads(ok.text)["settings"]}
            self.assertEqual(shown["STEER_KP"], 9.0)
            bad = await server.settings_save(Request({"values": {"STEER_KP": 1000}}))
            self.assertEqual(bad.status, 400)
            self.assertEqual(read(self.file), {"STEER_KP": 9.0})            # nichts verändert
            await server.settings_save(Request({"reset": True}))
            self.assertEqual(read(self.file), {})


class ForwardTest(unittest.TestCase):
    def test_stops_by_front_distance(self):
        body = Body(wall_cm=FACE + 2 * TILE)            # Wand am Ende der übernächsten Kachel
        result = make_robot(body).forward(set())
        self.assertIs(result.status, MoveStatus.OK)
        self.assertAlmostEqual(body.x, TILE, delta=1.5)

    def test_stops_by_encoder_without_wall(self):
        body = Body(wall_cm=None)
        make_robot(body).forward(set())
        self.assertAlmostEqual(body.x, TILE, delta=1.5)

    def test_front_distance_corrects_position(self):
        """Steht der Roboter 4 cm zu weit hinten, fährt er nach Abstand weiter als 30 cm."""
        body = Body(wall_cm=FACE + TILE + 4)
        make_robot(body).forward(set())
        self.assertAlmostEqual(body.x, TILE + 4, delta=1.5)

    def test_wall_ahead_blocks(self):
        body = Body(wall_cm=FACE)
        result = make_robot(body).forward(set())
        self.assertIs(result.status, MoveStatus.BLOCKED)
        self.assertEqual(body.x, 0.0)

    def test_black_tile_avoided_before_half(self):
        body = Body(wall_cm=None, floors=[TileType.NORMAL, TileType.BLACK, TileType.NORMAL])
        result = make_robot(body).forward({TileType.BLACK})
        self.assertIs(result.status, MoveStatus.AVOIDED)
        self.assertIs(result.floor, TileType.BLACK)
        self.assertLess(body.furthest, TILE / 2)     # nie mehr als halb auf der schwarzen Kachel
        self.assertAlmostEqual(body.x, 0.0, delta=1.5)

    def test_reports_floor_of_new_tile(self):
        body = Body(wall_cm=None, floors=[TileType.NORMAL, TileType.BLUE, TileType.NORMAL])
        result = make_robot(body).forward({TileType.BLACK})
        self.assertIs(result.floor, TileType.BLUE)


class TrimTest(unittest.TestCase):
    """Kettenfaktor MOTOR_TRIM_RIGHT: nur beim Geradeausfahren, nicht beim Drehen."""

    def recorded(self, robot):
        calls = []
        original = robot.motors.set_dps
        robot.motors.set_dps = lambda l, r: (calls.append((l, r)), original(l, r))[1]
        return calls

    def test_formula(self):
        # 10° nach rechts auf 100 cm, Ketten 15 cm auseinander: rechte Kette 0,1745 · 15 / 100 = 2,6 % mehr Weg
        self.assertAlmostEqual(new_trim(1.0, 10.0, 100.0, 15.0), 1.026, places=3)
        self.assertEqual(new_trim(1.037, 0.0, 100.0, 15.0), 1.037)
        self.assertLess(new_trim(1.0, -5.0, 100.0, 15.0), 1.0)          # nach links gedriftet: rechts langsamer
        # Versatz 17,6 cm auf 100 cm = atan(0,352) = 19,4°; Versatz 8,8 cm = 10°
        self.assertAlmostEqual(angle_from_offset(8.8, 100.0), 9.98, delta=0.05)
        self.assertAlmostEqual(new_trim(1.0, angle_from_offset(8.8, 100.0), 100.0, 15.0), 1.026, places=3)

    def test_forward_commands_right_track_faster_and_keeps_encoders_in_step(self):
        with mock.patch.object(config, "MOTOR_TRIM_RIGHT", 1.05):
            robot = make_robot(Body(wall_cm=None))
            calls = self.recorded(robot)
            robot.forward(set())
        self.assertTrue(calls)
        moving = [(l, r) for l, r in calls if l > 0 and r > 0]
        self.assertAlmostEqual(sum(r / l for l, r in moving) / len(moving), 1.05, delta=0.02)
        left, right = robot.motors.positions()
        self.assertAlmostEqual(left, right / 1.05, delta=0.03 * left)   # wirksame Encoder im Gleichlauf

    def test_drive_cm_in_step_with_trim(self):
        with mock.patch.object(config, "MOTOR_TRIM_RIGHT", 1.05):
            robot = make_robot(Body(wall_cm=None))
            robot.drive_cm(40)
        left, right = robot.motors.positions()
        self.assertAlmostEqual(right / left, 1.05, delta=0.02)
        self.assertAlmostEqual(left / config.MOTOR_DEG_PER_CM, 40, delta=2.0)

    def test_turn_ignores_trim(self):
        with mock.patch.object(config, "MOTOR_TRIM_RIGHT", 1.05):
            robot = make_robot(Body(wall_cm=None))
            calls = self.recorded(robot)
            robot.turn_right()
        self.assertTrue(all(abs(l) == abs(r) == config.TURN_SPEED_DPS for l, r in calls if l or r))


class TurnAndButtonsTest(unittest.TestCase):
    def test_turn_uses_encoder_target(self):
        body = Body(wall_cm=None)
        robot = make_robot(body)
        robot.turn_right()
        left, right = robot.motors.positions()
        self.assertAlmostEqual(left, config.MOTOR_DEG_PER_90, delta=15)
        self.assertAlmostEqual(right, -config.MOTOR_DEG_PER_90, delta=15)

    def test_interrupts(self):
        robot = make_robot(Body(wall_cm=None))
        robot.interrupt("lop")
        with self.assertRaises(LackOfProgress):
            robot.wait(5)
        robot.interrupt("stop")
        with self.assertRaises(RunStopped):
            robot.forward(set())


class HelperTest(unittest.TestCase):
    def test_stop_distance(self):
        c = config.FRONT_CENTER_CM
        self.assertEqual(stop_distance(c + TILE, 0), TILE)              # Wand hinter der nächsten Kachel
        self.assertEqual(stop_distance(c + 2 * TILE + 3, 0), TILE + 3)  # eine Kachel weiter, 3 cm zu weit hinten
        self.assertEqual(stop_distance(c + 2 * TILE - 10, 10), TILE)    # nach 10 cm Fahrt gemessen
        # Echo springt auf eine weiter entfernte Wand: Haltepunkt bleibt bei einer Kachel
        self.assertAlmostEqual(stop_distance(c + 3 * TILE - 13, 13), TILE)
        self.assertIsNone(stop_distance(None, 0))
        self.assertIsNone(stop_distance(config.FRONT_TRUST_CM + 1, 0))

    def test_heading_estimate(self):
        """Schräg (4° nach rechts) durch den Gang, dabei lenkt er sich laut Encoder um 3° zurück.
        Die Seitensensoren sitzen vor der Drehmitte und wandern beim Lenken mit."""
        h = HeadingEstimate()
        deg_per_degree = config.MOTOR_DEG_PER_90 / 90 * 2     # Encoderunterschied pro Grad Drehung
        offset = 0.5                                        # Drehmitte, seitlich
        for i in range(13):
            s = 2.0 * i
            angle = 4.0 - 3.0 * s / 24                      # wahrer Schrägstand
            h.track(s, (-deg_per_degree * (4.0 - angle), 0.0))
            h.sample(offset + config.US_SIDE_AHEAD_CM * math.sin(math.radians(angle)))
            offset += math.sin(math.radians(angle)) * 2.0
        self.assertAlmostEqual(h.degrees(), 1.0, delta=0.1)

    def test_heading_needs_enough_samples(self):
        h = HeadingEstimate()
        for s in range(config.HEADING_MIN_SAMPLES - 1):
            h.track(5.0 * s, (0.0, 0.0))
            h.sample(0.3 * s)
        self.assertIsNone(h.degrees())

    def test_classify_nearest_color(self):
        cal = {"white": (900, 850, 700, 2500), "black": (60, 60, 50, 170),
               "blue": (150, 300, 600, 1000), "silver": (1300, 1250, 1050, 3700),
               "red": (900, 250, 200, 1300)}
        self.assertEqual(classify((880, 830, 690, 2450), cal), "white")
        self.assertEqual(classify((50, 55, 45, 150), cal), "black")
        self.assertEqual(classify((140, 280, 620, 1050), cal), "blue")
        self.assertEqual(classify((950, 260, 190, 1350), cal), "red")


class FakeBrickPi3:
    """BrickPi3 ohne Hardware: Port-Bits wie im Original, die ersten `lose` Schreibbefehle gehen verloren."""
    PORT_A, PORT_B, PORT_C, PORT_D = 1, 2, 4, 8
    MOTOR_FLOAT = -128
    lose = 0

    def __init__(self):
        self.power = {p: 0 for p in (1, 2, 4, 8)}
        self.sent = []

    def set_motor_power(self, ports: int, power: int):
        self.sent.append((ports, power))
        if FakeBrickPi3.lose > 0:
            FakeBrickPi3.lose -= 1
            return
        for p in self.power:
            if ports & p:
                self.power[p] = power

    def get_motor_status(self, port: int):
        return [0, self.power[port], 0, 0]


class MotorsFloatTest(unittest.TestCase):
    def make(self, lose: int) -> Motors:
        FakeBrickPi3.lose = 0
        with mock.patch.object(hardware.motors.brickpi3, "BrickPi3", FakeBrickPi3):
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


def drive(steering: Steering, left: list, right: list, step_cm: float = 2.0) -> list[float]:
    """Seitenmessungen abwechselnd links/rechts einspeisen, je step_cm Fahrt dazwischen."""
    corrections, travelled = [], 0.0
    for l, r in zip(left, right):
        for side, cm in (("left", l), ("right", r)):
            travelled += step_cm
            corrections.append(steering.side(side, cm, travelled, (0.0, 0.0)))
    return corrections


C = config.SIDE_CENTER_CM          # Abstand Sensor–Wand in der Kachelmitte
FAR = C + TILE                      # Wand eine Kachel weiter


class SteeringTest(unittest.TestCase):
    def test_equal_sides_is_center(self):
        s = Steering()
        corr = drive(s, [C] * 5, [C] * 5)
        self.assertEqual(s.mode, "both")
        self.assertEqual(corr[-1], 0.0)

    def test_steers_towards_equal_sides(self):
        # näher an der rechten Wand -> nach links lenken (positiv), und umgekehrt
        self.assertGreater(drive(Steering(), [C + 2] * 4, [C - 2] * 4)[-1], 0)
        self.assertLess(drive(Steering(), [C - 2] * 4, [C + 2] * 4)[-1], 0)

    def test_lateral_drift_keeps_both_walls(self):
        """Seitliches Wandern ändert links + rechts nicht: kein Fehlalarm des Wächters."""
        s = Steering()
        corr = drive(s, [C, C, C - .5, C - 1, C - 1.5, C - 2], [C, C, C + .5, C + 1, C + 1.5, C + 2])
        self.assertEqual(s.mode, "both")
        self.assertLess(corr[-1], 0)      # zu weit links -> nach rechts lenken

    def test_wall_breaking_away_does_not_swerve(self):
        """Links endet die Wand (Wert kriecht erst hoch, dann Sprung), rechts geht weiter."""
        s = Steering()
        corr = drive(s, [C, C, C, C + .4, C + 1.6, C + 4, FAR, FAR, FAR], [C] * 9)
        self.assertEqual(s.mode, "right")
        self.assertLess(max(corr), 25)            # nur kurz leicht nach links, kein Ausscheren
        self.assertAlmostEqual(corr[-1], 0.0, delta=3)   # praktisch gerade (< 0,15 cm daneben)

    def test_single_far_wall_breaking_away(self):
        """Nur links eine Wand, eine Kachel weiter, sie bricht weg: kriecht langsam hoch."""
        s = Steering()
        corr = drive(s, [FAR] * 4 + [FAR + .3, FAR + 1.2, FAR + 2.5, None, None], [None] * 9)
        self.assertEqual(s.mode, "encoder")
        self.assertLess(max(corr), 30)

    def test_single_near_wall_breaking_away(self):
        s = Steering()
        corr = drive(s, [C] * 4 + [C + .4, C + 1.6, C + 4, None, None], [None] * 9)
        self.assertLess(max(corr), 15)

    def test_returning_wall_needs_two_readings(self):
        s = Steering()
        drive(s, [C, C, FAR, FAR], [C, C, C, C])
        self.assertEqual(s.mode, "right")
        s.side("left", C + .2, 20, (0, 0))
        self.assertEqual(s.mode, "right")          # eine Messung reicht nicht
        s.side("left", C, 22, (0, 0))
        self.assertEqual(s.mode, "both")

    def test_single_wall_keeps_half_width(self):
        s = Steering()
        drive(s, [C + 1] * 3, [C - 1] * 3)         # Gang 2·C breit
        corr = drive(s, [None] * 4, [C] * 4)       # links weg: rechts auf die halbe Breite halten
        self.assertEqual(s.mode, "right")
        self.assertAlmostEqual(corr[-1], 0.0, delta=1)

    def test_wide_room_uses_nearer_wall(self):
        """Foyer: rechts eine Kachel weiter offen. 'Gleich' wäre die Kachelgrenze, er bleibt auf seiner Kachel."""
        s = Steering()
        corr = drive(s, [C] * 4, [FAR] * 4)
        self.assertEqual(s.mode, "left")
        self.assertAlmostEqual(corr[-1], 0.0, delta=1)

    def test_only_far_wall_counts_whole_tiles(self):
        s = Steering()
        corr = drive(s, [FAR] * 4, [None] * 4)
        self.assertEqual(s.mode, "left")
        self.assertAlmostEqual(corr[-1], 0.0, delta=1)

    def test_no_wall_keeps_tracks_equal(self):
        s = Steering()
        drive(s, [None, None], [None, None])
        self.assertEqual(s.mode, "encoder")
        # linke Kette ist weiter gefahren -> nach links lenken
        self.assertGreater(s.hold(12, (110, 100)), 0)

    def test_correction_changes_gradually(self):
        s = Steering()
        corr = drive(s, [C + 6] * 3, [C - 6] * 3)  # 6 cm daneben: voll wären 120 dps
        steps = [b - a for a, b in zip([0.0] + corr, corr)]
        self.assertLessEqual(max(steps), config.STEER_SLEW_DPS_PER_CM * 2 + 1e-9)

    def test_no_d_with_far_opposite_wall(self):
        """Nur eine Wand nah, die Gegenwand fern (über Raster): D = 0, auch wenn sich die Abweichung ändert."""
        for near, far in (("left", "right"), ("right", "left")):
            s = Steering()
            sides = {near: [C + .5 * i for i in range(8)], far: [FAR] * 8}
            drive(s, sides["left"], sides["right"])
            self.assertEqual(s.mode, near)
            self.assertEqual(s.slope, 0.0)
            self.assertEqual(s.values()["d"], 0.0)

    def test_d_in_corridor(self):
        """Zwei nahe Wände (B <= CORRIDOR_MAX_CM): D wie bisher."""
        s = Steering()
        drive(s, [C + .5 * i for i in range(8)], [C - .5 * i for i in range(8)])
        self.assertEqual(s.mode, "both")
        self.assertLessEqual(s.width, config.CORRIDOR_MAX_CM)
        self.assertNotEqual(s.slope, 0.0)

    def test_grid_offset_limited(self):
        """Ferne Wand mit 7 cm Rasterabweichung: auf STEER_GRID_MAX_CM begrenzt, nahe Wand nicht."""
        s = Steering()
        drive(s, [FAR + 7] * 4, [None] * 4)
        self.assertEqual(s.mode, "left")
        self.assertEqual(s.offset, config.STEER_GRID_MAX_CM)
        s = Steering()
        drive(s, [None] * 4, [FAR + 7] * 4)
        self.assertEqual(s.offset, -config.STEER_GRID_MAX_CM)
        s = Steering()
        drive(s, [C + 7] * 4, [None] * 4)      # nahe Wand: unbegrenzt
        self.assertEqual(s.offset, 7)

    def test_contact_trusted_at_once_without_slew_limit(self):
        """Seitenwert 1,9 cm (Kante an der Wand) bei bis dahin ferner Wand: gilt sofort, Lenkung weg von der Wand,
        ohne die Begrenzung der Lenkänderung."""
        s = Steering()
        drive(s, [FAR] * 3, [C] * 3)
        before = s.corr
        corr = s.side("left", 1.9, 13.0, (0.0, 0.0))
        self.assertTrue(s.trusted["left"])
        self.assertEqual(s.contact, "left")
        self.assertEqual(s.values()["kontakt"], "left")
        self.assertLess(corr, 0)                                   # zu weit links -> nach rechts lenken
        self.assertGreater(before - corr, config.STEER_SLEW_DPS_PER_CM * 1 + 1)    # mehr als die Begrenzung erlaubt
        self.assertAlmostEqual(corr, -min(config.STEER_MAX_DPS, abs(s.raw)), places=6)
        s.side("right", C, 15.0, (0.0, 0.0))
        self.assertIsNone(s.contact)
        self.assertEqual(s.corridor, config.CORRIDOR_CM)           # schleifende Seite verfälscht B nicht

    def test_contact_right_steers_left(self):
        s = Steering()
        drive(s, [C] * 3, [FAR] * 3)
        self.assertGreater(s.side("right", 1.9, 13.0, (0.0, 0.0)), 0)


class WallContactTest(unittest.TestCase):
    """Schleifen an einer Seitenwand: Seitenwert dauerhaft 1,9 cm, die Encoder laufen normal."""

    class Scrape(FakeUltrasonic):
        def __init__(self, body, side, until_x=float("inf")):
            super().__init__(body)
            self.side, self.until_x = side, until_x

        def measure(self, side):
            cm = super().measure(side)
            if side == self.side and self.body.x < self.until_x and cm is not None:
                cm = 1.9
                self.last[side] = cm
            return cm

    def run_forward(self, side, until_x):
        body = Body(wall_cm=None)
        robot = RealRobot(FakeMotors(body), self.Scrape(body, side, until_x), FakeColor(body))
        seen = []
        robot.trace = lambda what, **v: seen.append((what, v))
        return body, robot, seen, robot.forward(set())

    def test_backs_off_turns_away_and_continues(self):
        # Bei x < 8 schleift er rechts. Er hängt ab > 10 cm Strecke. Der Schleifwert endet nach dem Zurücksetzen.
        body = Body(wall_cm=None)
        us = self.Scrape(body, "right")
        robot = RealRobot(FakeMotors(body), us, FakeColor(body))
        seen = []
        robot.trace = lambda what, **v: seen.append((what, v))
        original = robot._rotate

        def rotate(deg):
            us.until_x = -1e9                     # nach dem Wegdrehen schleift nichts mehr
            return original(deg)
        robot._rotate = rotate
        result = robot.forward(set())
        self.assertIs(result.status, MoveStatus.OK)
        hits = [v for what, v in seen if what == "schleift"]
        self.assertEqual(len(hits), 1)
        self.assertEqual(hits[0]["seite"], "right")
        self.assertGreater(hits[0]["gefahren"], config.STEER_CONTACT_DIST_CM)
        self.assertFalse(any(what == "blockiert" for what, _ in seen))
        self.assertAlmostEqual(body.x, TILE, delta=2.5)           # Ziel bleibt die Kachelmitte (Strecke berücksichtigt)
        turns = [v["winkel"] for what, v in seen if what == "drehen"]
        self.assertTrue(turns and all(w == -config.CONTACT_ANGLE_DEG for w in turns))   # rechts schleift: nach links

    def test_left_turns_right(self):
        body = Body(wall_cm=None)
        us = self.Scrape(body, "left")
        robot = RealRobot(FakeMotors(body), us, FakeColor(body))
        seen = []
        robot.trace = lambda what, **v: seen.append((what, v))
        original = robot._rotate
        robot._rotate = lambda deg: (setattr(us, "until_x", -1e9), original(deg))[1]
        with mock.patch.object(config, "STEER_CONTACT_DIST_CM", 5.0):
            robot.forward(set())
        self.assertTrue(any(v["winkel"] == config.CONTACT_ANGLE_DEG for what, v in seen if what == "drehen"))

    def test_gives_up_after_max_tries(self):
        body, robot, seen, result = self.run_forward("right", float("inf"))
        self.assertIs(result.status, MoveStatus.BLOCKED)
        self.assertEqual(len([1 for what, _ in seen if what == "schleift"]), config.CONTACT_MAX_TRIES)
        self.assertTrue(any(what == "blockiert" for what, _ in seen))
        self.assertLess(body.x, 2.5)                              # zurück am Anfang


if __name__ == "__main__":
    unittest.main()


class CenteringPlanTest(unittest.TestCase):
    """Entscheidung vor der Drehung: vorne zurück ja/nein, seitlich ja/nein."""

    def test_front_close_only(self):
        back, side = centering_plan(4.0, 6.25, 6.25)
        self.assertAlmostEqual(back, config.FRONT_CENTER_CM - 4.0)
        self.assertEqual(side, 0.0)

    def test_side_close_with_room_on_the_other_side(self):
        back, side = centering_plan(12.0, 4.0, 9.0)             # links nah, rechts Platz: nach rechts, nur bis mittig
        self.assertEqual(back, 0.0)
        self.assertAlmostEqual(side, -2.5)
        back, side = centering_plan(12.0, 9.0, 4.0)
        self.assertAlmostEqual(side, 2.5)

    def test_open_area_or_no_echo_does_nothing(self):
        self.assertEqual(centering_plan(40.0, 35.0, None), (0.0, 0.0))
        self.assertEqual(centering_plan(None, None, None), (0.0, 0.0))
        self.assertEqual(centering_plan(12.0, 6.25, 6.25), (0.0, 0.0))     # mittig im Gang: nichts


class RearRoomTest(unittest.TestCase):
    """Rückraum: nie blind zurücksetzen (kein Sensor hinten, die Ketten rutschen an einer Wand, die Encoder zählen weiter)."""

    def make(self, rear):
        robot = make_robot(Body(wall_cm=None))
        robot.start_clock()
        robot.rear_free_cm = rear
        self.moves, self.trace = [], []
        robot.drive_cm = lambda cm: self.moves.append(("drive", round(cm, 2)))
        robot._rotate = lambda deg: self.moves.append(("rotate", round(deg, 2)))
        robot.trace = lambda what, **v: self.trace.append((what, v))
        return robot

    def test_known_after_forward_unknown_after_turn(self):
        robot = make_robot(Body(wall_cm=None))
        robot.start_clock()
        self.assertIs(robot.forward(set()).status, MoveStatus.OK)
        self.assertAlmostEqual(robot.rear_free_cm, TILE, delta=2.5)         # er kam von dort
        robot._rotate(90)
        self.assertIsNone(robot.rear_free_cm)
        robot.rear_free_cm = 20.0
        robot._rotate(8)                                                    # kleine Korrekturdrehung: bleibt
        self.assertEqual(robot.rear_free_cm, 20.0)
        robot.drive_cm(-5)
        self.assertAlmostEqual(robot.rear_free_cm, 15.0, delta=0.01)
        robot.drive_cm(-50)
        self.assertEqual(robot.rear_free_cm, 0.0)

    def test_estimate_from_front_distance_after_turn(self):
        robot = make_robot(Body(wall_cm=None))
        self.assertAlmostEqual(robot._rear_room(8.9), 2.1, places=6)        # 28 - 16,5 - 8,9 - 0,5
        self.assertEqual(robot._rear_room(20.0), 0.0)
        self.assertEqual(robot._rear_room(None), 0.0)                       # unbekannt: nie blind zurück
        robot.rear_free_cm = 30.0
        self.assertEqual(robot._rear_room(8.9), 30.0)                       # bekannt: dieser Wert

    def test_straight_back_limited(self):
        robot = self.make(1.0)
        robot._center_before_turn({"front": 4.0, "left": 6.25, "right": 6.25})
        self.assertEqual(self.moves, [("drive", -1.0)])
        self.assertEqual(self.trace[-1][0], "zentriert_vor_drehung")
        self.assertEqual(self.trace[-1][1]["rueckraum"], 1.0)

    def test_slanted_back_limited_and_dropped_when_it_gains_too_little(self):
        values = {"front": 12.0, "left": 4.0, "right": 9.0}                 # braucht 2,5 cm seitlich = 18 cm schräg
        robot = self.make(8.0)
        robot._center_before_turn(values)
        self.assertEqual([m[0] for m in self.moves], ["rotate", "drive", "rotate", "drive"])      # zuletzt wieder vor
        self.assertEqual(self.moves[1], ("drive", -8.0))                    # statt 12
        robot = self.make(5.0)                                              # 5 cm · sin 8° = 0,7 cm < 1 cm: entfällt
        robot._center_before_turn(values)
        self.assertEqual(self.moves, [])

    def test_second_turn_of_a_u_turn_does_not_reverse_into_the_wall(self):
        """Nach der ersten Drehung unbekannt, vorne 8,9 cm: nur 2,1 cm Platz hinten, die Seitenkorrektur entfällt."""
        robot = self.make(None)
        robot._center_before_turn({"front": 8.9, "left": 4.5, "right": None})
        self.assertEqual(self.moves, [])


class StickingShareTest(unittest.TestCase):
    """`sticking`: Anteil der Messungen im Kontakt statt ununterbrochen (Ausreißer setzt nichts zurück)."""

    def feed(self, values, step=1.25, side="left"):
        s, t, hit = Steering(), 0.0, None
        for cm in values:
            s.side(side, cm, t, (0.0, 0.0))
            hit = s.sticking(t)
            t += step
        return s, hit

    def test_single_outlier_does_not_reset(self):
        s, hit = self.feed([1.9] * 4 + [2.9] + [1.9] * 4, step=1.3)                  # 9 Messungen über 10 cm
        self.assertEqual(hit, "left")

    def test_short_touch_then_free_does_not_trigger(self):
        s, hit = self.feed([1.9, 1.9] + [C] * 5, step=2.5)                  # 2 Kontakte, dann 5 cm frei
        self.assertIsNone(hit)
        s, hit = self.feed([1.9, 1.9, 1.9] + [C] * 6, step=2.5)
        self.assertIsNone(hit)

    def test_too_few_measurements_or_too_short(self):
        self.assertIsNone(self.feed([1.9, 1.9], step=6.0)[1])               # nur 2 Messungen
        self.assertIsNone(self.feed([1.9] * 4, step=1.0)[1])                # erst 3 cm beobachtet

    def test_right_side(self):
        self.assertEqual(self.feed([1.9] * 10, side="right", step=1.3)[1], "right")


class FrontSlipTest(unittest.TestCase):
    """Festfahren vorne: Abstand sinkt viel weniger als die Encoderstrecke."""

    class Slippy(FakeUltrasonic):
        """Abstand vorne = 120 cm − factor · Weg (Wand weit weg): factor < 1 = die Ketten rutschen."""
        def __init__(self, body, factor):
            super().__init__(body)
            self.factor = factor

        def measure(self, side):
            cm = super().measure(side)
            if side == "front":
                cm = 120.0 - self.factor * self.body.x
                self.last[side] = cm
            return cm

    def run_forward(self, factor, free_after_back=False):
        body = Body(wall_cm=None)
        us = self.Slippy(body, factor)
        robot = RealRobot(FakeMotors(body), us, FakeColor(body))
        seen, actions = [], []
        robot.trace = lambda what, **v: seen.append((what, v))
        original = robot.set_action
        robot.set_action = lambda a, d="": (actions.append(a), original(a, d))[1]
        if free_after_back:                       # nach dem Zurücksetzen packen die Ketten wieder
            drive = robot.drive_cm
            robot.drive_cm = lambda cm: (setattr(us, "factor", 1.0) if cm < 0 else None, drive(cm))[1]
        return body, robot, seen, actions

    def test_slipping_backs_off_and_continues(self):
        config_front = mock.patch.object(config, "FRONT_TRUST_CM", 200.0)
        with config_front:
            body, robot, seen, actions = self.run_forward(0.2, free_after_back=True)
            result = robot.forward(set())
        self.assertIn("Rutscht vorne", actions)
        hits = [v for what, v in seen if what == "rutscht"]
        self.assertEqual(len(hits), 1)
        self.assertLess(hits[0]["gemessen"], config.FRONT_STUCK_RATIO * hits[0]["erwartet"])
        self.assertFalse(any(what == "schleift" for what, _ in seen))
        self.assertFalse(any(v["winkel"] for what, v in seen if what == "drehen"))      # kein Wegdrehen
        self.assertIs(result.status, MoveStatus.OK)

    def test_normal_drive_with_slip_factor_does_not_trigger(self):
        with mock.patch.object(config, "FRONT_TRUST_CM", 200.0):
            body, robot, seen, actions = self.run_forward(0.87)
            robot.forward(set())
        self.assertNotIn("Rutscht vorne", actions)
        self.assertFalse(any(what == "rutscht" for what, _ in seen))

    def test_gives_up_after_max_tries(self):
        with mock.patch.object(config, "FRONT_TRUST_CM", 200.0):
            body, robot, seen, actions = self.run_forward(0.2)
            result = robot.forward(set())
        self.assertIs(result.status, MoveStatus.BLOCKED)
        self.assertEqual(actions.count("Rutscht vorne"), config.CONTACT_MAX_TRIES)
        self.assertTrue(any(what == "blockiert" for what, _ in seen))


class FrontSpikeUltrasonic(FakeUltrasonic):
    """Vorne ein einzelner Störimpuls (3 cm), sobald der Roboter `at_x` cm gefahren ist (nur in der Fahrschleife)."""

    def __init__(self, body, at_x):
        super().__init__(body)
        self.at_x, self.fired = at_x, False

    def measure(self, side):
        if side == "front" and not self.fired and self.body.x >= self.at_x:
            self.fired = True
            self.last[side] = 3.0
            return 3.0
        return super().measure(side)


class StopMedianTest(unittest.TestCase):
    def test_median_function(self):
        self.assertIsNone(stop_median([]))
        self.assertEqual(stop_median([30.0]), 30.0)
        self.assertEqual(stop_median([30.0, 22.0]), 26.0)           # bei 2 Werten der Median dieser
        self.assertEqual(stop_median([30.0, 30.0, 22.0]), 30.0)     # Ausreißer verändert nichts
        self.assertEqual(stop_median([22.0, 30.0, 30.5]), 30.0)

    def drive_with_spike(self, back=False):
        body = Body(wall_cm=FACE + 2 * TILE)
        us = FrontSpikeUltrasonic(body, 23.0)
        robot = RealRobot(FakeMotors(body), us, FakeColor(body))
        seen = []
        robot.trace = lambda what, **v: seen.append((what, v))
        result = robot.forward(set())
        return body, result, seen

    def test_spike_does_not_stop_early(self):
        """Störimpuls 3 cm bei 23 cm Fahrt: ohne Median wäre der Halt auf 22 cm gesetzt (5 cm zu kurz)."""
        body, result, seen = self.drive_with_spike()
        self.assertIs(result.status, MoveStatus.OK)
        self.assertAlmostEqual(body.x, TILE, delta=1.5)
        spike = [v for what, v in seen if what == "vorne" and v["cm"] == 3.0][0]
        self.assertAlmostEqual(spike["raster"], 22.0, delta=0.01)    # der Rasterwert selbst ist verfälscht ...
        self.assertAlmostEqual(spike["median"], TILE, delta=1.5)     # ... der Haltepunkt nicht

    def test_without_median_it_would_stop_early(self):
        with mock.patch.object(config, "FRONT_STOP_MEDIAN", 1), mock.patch.object(config, "ARRIVE_FIX_MAX_CM", 0.0):
            body, _, _ = self.drive_with_spike()
        self.assertLess(body.x, 26.0)           # stoppt sofort, 7 cm zu kurz


class RelocatedTest(unittest.TestCase):
    def test_clears_values_of_the_old_place(self):
        robot = make_robot(Body(wall_cm=None))
        robot._wall_values = {"front": 10.0, "left": 6.0, "right": 6.0}
        robot.rear_free_cm, robot.front_free_cm, robot._front_fresh = 20.0, 10.0, 12.0
        robot.blind_streak = 1
        robot.relocated()
        self.assertIsNone(robot._wall_values)
        self.assertIsNone(robot.rear_free_cm)
        self.assertIsNone(robot.front_free_cm)
        self.assertIsNone(robot._front_fresh)
        self.assertEqual(robot.blind_streak, 1)

    def test_competition_calls_it_after_lack_of_progress(self):
        import main
        calls = []
        robot = make_robot(Body(wall_cm=None))
        robot.relocated = lambda: calls.append("relocated")
        robot.clear_interrupt = lambda: calls.append("clear")

        class Explorer:
            mode = ""
            runs = 0

            def run(self):
                raise LackOfProgress

            def resume(self):
                return None

            def lack_of_progress(self):
                calls.append("lop")

        main.competition(robot, Explorer(), lambda: "lop")
        self.assertEqual(calls[:3], ["clear", "relocated", "lop"])


class ArrivalFixTest(unittest.TestCase):
    FC = config.FRONT_CENTER_CM

    def test_target_is_nearest_tile_center(self):
        self.assertAlmostEqual(arrival_target(self.FC + 4), self.FC)
        self.assertAlmostEqual(arrival_target(self.FC + TILE - 3), self.FC + TILE)
        self.assertAlmostEqual(arrival_target(self.FC - 3), self.FC)       # n nie negativ

    def test_thresholds(self):
        self.assertIsNone(None if arrival_fix(None) == 0.0 else 1)
        self.assertEqual(arrival_fix(self.FC + 2.0), 0.0)                  # Rauschen
        self.assertAlmostEqual(arrival_fix(self.FC + 4.0), 4.0)            # zu weit weg: vor
        self.assertAlmostEqual(arrival_fix(self.FC - 3.5), -3.5)           # zu nah: zurück
        self.assertAlmostEqual(arrival_fix(self.FC + TILE + 9.0), 9.0)     # mehrere Kacheln weiter
        self.assertEqual(arrival_fix(self.FC + 12.0), 0.0)                 # über ARRIVE_FIX_MAX_CM: anderes Echo
        self.assertEqual(arrival_fix(self.FC + TILE / 2 + 0.1), arrival_fix(self.FC + TILE / 2 + 0.1))

    def test_robot_settles_when_tolerance_leaves_overshoot(self):
        """Wand 12 cm weiter als die Kachelmitte: der Halt wird auf 38 cm begrenzt (4 cm zu kurz), dann setzt er nach vorn."""
        body = Body(wall_cm=FACE + TILE + 12)
        robot = make_robot(body)
        seen = []
        robot.trace = lambda what, **v: seen.append((what, v))
        result = robot.forward(set())
        self.assertIs(result.status, MoveStatus.OK)
        fixes = [v for what, v in seen if what == "nachgesetzt"]
        self.assertEqual(len(fixes), 1)
        self.assertAlmostEqual(fixes[0]["strecke"], 4.0, delta=1.0)
        self.assertAlmostEqual(fixes[0]["danach"], config.FRONT_CENTER_CM, delta=1.0)
        self.assertAlmostEqual(body.x, TILE + 12, delta=1.5)
        self.assertAlmostEqual(robot._front_fresh, config.FRONT_CENTER_CM, delta=1.0)

    def test_no_extra_measurement_without_wall_or_when_in_place(self):
        body = Body(wall_cm=None)
        robot = make_robot(body)
        robot.forward(set())
        self.assertIsNone(robot._front_fresh)                               # keine Wand in Reichweite: nichts gemessen
        body = Body(wall_cm=FACE + TILE)
        robot = make_robot(body)
        seen = []
        robot.trace = lambda what, **v: seen.append(what)
        robot.forward(set())
        self.assertNotIn("nachgesetzt", seen)
        self.assertIsNotNone(robot._front_fresh)                            # trotzdem für read_walls / nächste Fahrt gemerkt

    def test_moving_clears_fresh_value(self):
        robot = make_robot(Body(wall_cm=None))
        robot._front_fresh = 8.0
        robot._rotate(10)
        self.assertIsNone(robot._front_fresh)
        robot._front_fresh = 8.0
        robot.drive_cm(1)
        self.assertIsNone(robot._front_fresh)
        robot._front_fresh = 8.0
        robot.reset()
        self.assertIsNone(robot._front_fresh)


class StillTimeTest(unittest.TestCase):
    """Stillstandszeit sparen: gültige Messwerte nicht noch einmal messen."""

    class Counting(FakeUltrasonic):
        def __init__(self, body):
            super().__init__(body)
            self.calls = []

        def measure(self, side):
            self.calls.append(side)
            return super().measure(side)

    def make(self, wall_cm):
        body = Body(wall_cm=wall_cm)
        us = self.Counting(body)
        robot = RealRobot(FakeMotors(body), us, FakeColor(body))
        robot.trace = lambda what, **v: self.seen.append((what, v))
        self.seen = []
        return body, us, robot

    def test_read_walls_takes_front_from_fresh_value(self):
        body, us, robot = self.make(FACE + TILE)
        robot.forward(set())
        us.calls.clear()
        robot._front_fresh = 7.5
        walls = robot.read_walls()
        self.assertNotIn("front", us.calls)
        self.assertEqual(us.calls.count("left"), 1)                         # (Fake: median_fast = eine Messung)
        self.assertEqual([v for what, v in self.seen if what == "wände"][-1]["werte"]["front"], 7.5)
        self.assertTrue(walls["front"])

    def test_forward_uses_wall_values_without_measuring(self):
        body, us, robot = self.make(None)
        robot.read_walls()
        us.calls.clear()
        robot.forward(set())
        first_loop = us.calls[0]                                            # erste Messung ist die der Fahrschleife
        start = [v for what, v in self.seen if what == "vorwärts"][0]
        self.assertEqual(start["vorne_quelle"], "wände")
        seiten = [v for what, v in self.seen if what == "seite"][:2]
        self.assertEqual([v["cm"] for v in seiten], [config.SIDE_CENTER_CM] * 2)
        self.assertIn(first_loop, ("front", "left", "right"))
        self.assertLessEqual(us.calls[:2].count("left") + us.calls[:2].count("right"), 1)   # nicht schon am Start beide Seiten

    def test_forward_sources(self):
        body, us, robot = self.make(None)
        robot.forward(set())
        self.assertEqual([v for what, v in self.seen if what == "vorwärts"][0]["vorne_quelle"], "gemessen")
        body, us, robot = self.make(FACE + 2 * TILE)
        robot._front_fresh = 40.0
        robot.forward(set())
        self.assertEqual([v for what, v in self.seen if what == "vorwärts"][0]["vorne_quelle"], "frisch")
        self.assertNotIn("front", us.calls[:1])                             # Start ohne Messung vorne

    def test_backward_clears_values_too(self):
        body, us, robot = self.make(None)
        robot.read_walls()
        robot._front_fresh = 9.0
        robot.backward()
        self.assertEqual([v for what, v in self.seen if what == "rückwärts"][0]["vorne_quelle"], "wände")


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


class FitSlopeTest(unittest.TestCase):
    def test_exact_line(self):
        self.assertAlmostEqual(fit_slope([(0, 1.0), (1, 1.2), (2, 1.4), (3, 1.6)]), 0.2)
        self.assertEqual(fit_slope([(0, 1.0)]), 0.0)
        self.assertEqual(fit_slope([(2, 1.0), (2, 3.0)]), 0.0)

    def test_fit_is_more_accurate_than_endpoint_difference(self):
        import random
        rng = random.Random(5)
        err_fit = err_end = 0.0
        for _ in range(300):
            pts = [(t / 2, 0.1 * t / 2 + rng.gauss(0, 0.4)) for t in range(9)]
            err_fit += (fit_slope(pts) - 0.1) ** 2
            err_end += ((pts[-1][1] - pts[0][1]) / 4.0 - 0.1) ** 2
        self.assertLess(err_fit, 0.7 * err_end)

    def test_steering_needs_three_samples(self):
        s = Steering()
        drive(s, [C + 1.0, C + 2.0], [C - 1.0, C - 2.0], step_cm=1.0)       # nur 4 Proben über kurze Strecke, aber Fenster gefüllt
        s2 = Steering()
        s2.mode = "both"
        s2.samples.append((0.0, 0.0))
        s2.last = {"left": C, "right": C}
        s2.trusted = {"left": True, "right": True}
        s2.side("left", C + 3.0, 3.0, (0.0, 0.0))                           # nur 2 Proben, Spanne 3 >= 2: früher Steigung, jetzt 0
        self.assertEqual(s2.slope, 0.0)
