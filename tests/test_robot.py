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
from hardware.color_sensor import classify
from hardware.motors import Motors
from hardware.robot import HeadingEstimate, LackOfProgress, RealRobot, RunStopped, SensorsBlind, Steering, stop_distance
from hardware.ultrasonic import load_calibration
from navigation.maze_map import TileType
from navigation.robot_interface import MoveStatus
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

if __name__ == "__main__":
    unittest.main()
