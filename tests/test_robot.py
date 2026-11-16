"""Tests für RealRobot (hardware/robot.py): vorwärts, Taster, Einstellungen, blinde Sensoren, Kettenfaktor; mit nachgebauter
Hardware aus tests/fakes.py (nichts bewegt sich). Lenkung: test_steering.py, Zentrieren: test_centering.py, Motoren/Kalibrierung: test_hardware.py.

Der nachgebaute Roboter fährt auf einer Geraden: x = Weg der Robotermitte in cm ab der Mitte
der Startkachel. Die Zeit läuft im Nachbau SPEEDUP-mal so schnell, damit die Tests kurz bleiben.
"""
import json
import math
import tempfile
import unittest
from pathlib import Path
from unittest import mock

import config
from hardware.color_sensor import classify
from hardware.robot import HeadingEstimate, LackOfProgress, RealRobot, RunStopped, SensorsBlind, stop_distance
from navigation.maze_map import TileType
from navigation.robot_interface import MoveStatus
from tools.test_motors import angle_from_offset, new_trim
from tests.fakes import TILE, FACE, Body, FakeMotors, FakeUltrasonic, FakeColor, make_robot, BlindUltrasonic


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


class ProgramTest(unittest.TestCase):
    """main.RobotProgram ohne Hardware: Warten auf Taster, Anzeige-Zustand (nichts läuft im Hintergrund)."""

    def make(self):
        import asyncio
        import main
        loop = asyncio.new_event_loop()
        self.addCleanup(loop.close)
        program = main.RobotProgram(loop)
        body = Body(wall_cm=None)
        program.robot = RealRobot(FakeMotors(body), FakeUltrasonic(body), FakeColor(body))
        program.robot.motors.battery = lambda: 11.5
        program.robot.motors.power = lambda: (0, 0)
        program.robot.us.warning = program.robot.color.warning = None
        return program

    def test_sensor_error_while_waiting_does_not_kill_the_thread(self):
        import threading
        program = self.make()
        calls = []

        def poll():
            calls.append(1)
            if len(calls) == 1:
                raise OSError("I2C weg")
        program.robot.poll_sensors = poll
        with mock.patch.object(config, "IDLE_SENSOR_S", 0.05):
            threading.Timer(0.3, program.presses.put, ["start"]).start()
            self.assertEqual(program.wait_for("start"), "start")
        self.assertGreater(len(calls), 1)                       # nach dem Fehler weiter gelesen
        self.assertIsNone(program.error)                        # und der Fehler ist wieder weg

    def test_closing_ends_the_lack_of_progress_wait_like_k2(self):
        program = self.make()
        program.closing = True
        with mock.patch.object(config, "IDLE_SENSOR_S", 0.05):
            self.assertEqual(program.wait_for("lop", "stop"), "stop")

    def test_state_msg_survives_brickpi_read_error(self):
        program = self.make()
        msg = program.state_msg()
        self.assertEqual(msg["sensors"]["battery"], 11.5)
        program._battery_t = 0.0
        program.robot.motors.battery = mock.Mock(side_effect=OSError("SPI"))
        with mock.patch.object(config, "BATTERY_READ_S", 0.0):
            msg = program.state_msg()
        self.assertEqual(msg["sensors"]["battery"], 11.5)       # letzter Wert bleibt
        self.assertTrue(any("BrickPi3" in w for w in msg["warnings"]))
        program.battery = None                                  # noch nie gelesen: kein Absturz
        msg = program.state_msg()
        self.assertIsNone(msg["sensors"]["battery"])
        self.assertFalse(msg["battery_low"])


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

