"""Tests für die Fahrphysik (simulation/sim_hardware.py) mit dem echten Fahrcode."""
import random
import unittest
from unittest import mock

import config
from hardware.robot import TimeUp
from navigation.explorer import Explorer
from navigation.maze_map import TileType
from navigation.robot_interface import MoveStatus
from simulation.sim_hardware import TILE, SimDrivingRobot, SimHardware
from simulation.sim_robot import SimWorld


def corridor(length: int, floors: dict | None = None) -> SimWorld:
    """Gerader Gang nach Norden, Start unten."""
    floors = floors or {}
    tiles = [{"x": 0, "y": y, "walls": "EW", "floor": floors.get(y, "normal")} for y in range(length)]
    return SimWorld.from_dict({"width": 1, "height": length, "start": [0, 0], "tiles": tiles})


def explore(world: SimWorld, noise=False, slip=False, seed=0):
    hw = SimHardware(world, noise=noise, slip=slip)
    hw.body.rng = random.Random(seed)
    hw.ultrasonic.rng = random.Random(seed + 1000)
    robot = SimDrivingRobot(hw)
    explorer = Explorer(robot)
    robot.start_clock()
    try:
        explorer.run()
    except TimeUp:
        explorer.mode = "Zeit abgelaufen"
    return hw, explorer


class SensorTest(unittest.TestCase):
    def test_side_sensors_in_corridor(self):
        hw = SimHardware(corridor(3))
        for side in ("left", "right"):
            self.assertAlmostEqual(hw.body.distance(side), config.SIDE_CENTER_CM, places=6)
        self.assertAlmostEqual(hw.body.distance("front"), config.FRONT_CENTER_CM + 2 * TILE, places=6)

    def test_reading_creeps_at_wall_end(self):
        """Hinter einem Wandende kriecht der Wert langsam hoch, dann springt er."""
        # Wand nur zwischen Spalte 0 und 1 auf Höhe der unteren Kachel, Roboter in Spalte 1
        tiles = [{"x": x, "y": y, "walls": "W" if (x, y) == (1, 0) else ""} for x in range(3) for y in range(2)]
        hw = SimHardware(SimWorld.from_dict({"width": 3, "height": 2, "start": [1, 0], "tiles": tiles}))
        values = []
        for i in range(100):
            hw.body.y = 15 + i * 0.25
            values.append(hw.body.distance("left"))
        steps = [b - a for a, b in zip(values, values[1:])]
        self.assertTrue(all(s >= -1e-9 for s in steps))        # wird nur größer
        self.assertGreaterEqual(sum(1 for s in steps if 0.02 < s < 1.0), 2)   # erst kriechen ...
        self.assertGreater(max(values), config.SIDE_CENTER_CM + 10)       # ... dann Sprung

    def test_grazing_side_walls_give_no_front_echo(self):
        hw = SimHardware(corridor(6))
        self.assertAlmostEqual(hw.body.distance("front"), config.FRONT_CENTER_CM + 5 * TILE, places=6)


class CollisionTest(unittest.TestCase):
    def test_wall_end_cap_collides(self):
        """Das Wandende hat eine runde Kappe (Radius WALL_HALF): 0,5 cm Abstand stößt, 1,5 cm nicht."""
        # freistehende Wand zwischen Kachel (0, 0) und (1, 0); Wandspitze bei y = TILE, Roboter in Kachel (2, 1)
        tiles = [{"x": x, "y": y, "walls": "W" if (x, y) == (1, 0) else ""} for x in range(3) for y in range(2)]
        hw = SimHardware(SimWorld.from_dict({"width": 3, "height": 2, "start": [2, 0], "tiles": tiles}))
        body = hw.body
        tip_x, tip_y = TILE, TILE                           # oberes Ende der Wand
        half_w, half_l = config.ROBOT_WIDTH_CM / 2, config.ROBOT_LENGTH_CM / 2
        # Roboter nach Norden, seine linke Kante steht neben der Wandspitze (Spitze liegt zwischen Stützpunkten)
        for gap, expected in ((0.5, True), (1.5, False)):
            x = tip_x + half_w + gap
            y = tip_y + 0.4 * half_l                         # Spitze etwas hinter der Mitte
            self.assertEqual(body._collides(x, y, 0.0) is not None, expected, f"Abstand {gap} cm")


class DrivingTest(unittest.TestCase):
    def test_forward_ends_on_tile_center(self):
        hw = SimHardware(corridor(4))
        result = SimDrivingRobot(hw).forward({TileType.BLACK})
        self.assertIs(result.status, MoveStatus.OK)
        self.assertAlmostEqual(hw.body.y, 1.5 * TILE, delta=1.0)
        self.assertAlmostEqual(hw.body.x, 0.5 * TILE, delta=0.5)

    def test_black_tile_detected_before_half(self):
        hw = SimHardware(corridor(3, {1: "black"}))
        result = SimDrivingRobot(hw).forward({TileType.BLACK})
        self.assertIs(result.status, MoveStatus.AVOIDED)
        self.assertEqual(hw.body.lop, 0)
        self.assertAlmostEqual(hw.body.y, 0.5 * TILE, delta=1.5)

    def test_forward_straightens_in_corridor(self):
        """Schräg durch den Gang (Lenkung aus, damit der Schrägstand bis zum Kachelende bleibt): die Schätzung
        aus den Seitenwänden trifft ihn, am Kachelende dreht er sich um genau diesen Winkel zurück."""
        hw = SimHardware(corridor(4))
        hw.body.heading = 4.0
        robot = SimDrivingRobot(hw)
        seen = []
        robot.trace = lambda what, **v: seen.append(v) if what == "schräglage" else None
        with mock.patch.object(config, "STEER_KP", 0.0), mock.patch.object(config, "STEER_KD", 0.0):
            result = robot.forward({TileType.BLACK})
        self.assertIs(result.status, MoveStatus.OK)
        self.assertAlmostEqual(seen[0]["geschätzt"], 4.0, delta=0.2)
        self.assertAlmostEqual(seen[0]["dreht"], -seen[0]["geschätzt"])

    def test_trim_cancels_drift_in_corridor(self):
        """Drift nach rechts (DRIFT_BIAS 4,3 % weniger Weg rechts, mit Kettenschlupf): ohne Kettenfaktor muss die Lenkung
        in jeder Kachel dagegen halten, mit Faktor 1/(1 − 0,043) läuft er ohne Lenkung fast gerade. Verglichen wird der
        Richtungsfehler nach einer Kachel ohne Lenkung."""
        def run(trim):
            hw = SimHardware(corridor(4), slip=True)
            hw.body.rng = random.Random(1)
            robot = SimDrivingRobot(hw)
            robot.start_clock()
            with mock.patch.object(config, "MOTOR_TRIM_RIGHT", trim), mock.patch.object(config, "STEER_KP", 0.0), \
                    mock.patch.object(config, "STEER_KD", 0.0), mock.patch.object(config, "HEADING_FIX_MIN_DEG", 999.0):
                result = robot.forward({TileType.BLACK})
            self.assertIs(result.status, MoveStatus.OK)
            return hw.body.heading
        # Ohne Lenkung (KP = KD = 0) wirkt nur die Drift.
        without = run(1.0)
        with_trim = run(1 / (1 - 0.043))
        # 4,3 % auf 30 cm = 1,29 cm Wegunterschied / 15 cm Kettenabstand = 0,086 rad = 4,9 Grad (gemessen 4,98),
        # mit Faktor bleibt nur der zufällige Schlupf je Kette (gemessen 1,38 Grad, seed 1)
        self.assertGreater(abs(without), 4.0)
        self.assertLess(abs(with_trim), abs(without) / 2)

    def test_blocked_tracks_do_not_hang(self):
        """Schräg gegen die Seitenwand: die Fahrt endet als 'blockiert' statt endlos zu drücken."""
        hw = SimHardware(corridor(4))
        hw.body.heading = 50.0
        robot = SimDrivingRobot(hw)
        robot.start_clock()
        result = robot.forward(set())
        self.assertIn(result.status, (MoveStatus.BLOCKED, MoveStatus.OK))
        self.assertLess(robot.time(), config.TIME_LIMIT_S)

    def test_off_center_and_skewed_towards_wall_arrives(self):
        """2,5 cm außermittig und 6° schräg zur Wand hin (Kante nahe der Wand): die Fahrt kommt ohne Abbruch an
        (Lenkung bei Wandkontakt sofort, sonst Zurücksetzen und Wegdrehen)."""
        for sign in (1, -1):                # rechte bzw. linke Wand
            with self.subTest(wand="rechts" if sign > 0 else "links"):
                hw = SimHardware(corridor(4))
                hw.body.x += sign * 2.5
                hw.body.heading = sign * 6.0
                robot = SimDrivingRobot(hw)
                robot.start_clock()
                result = robot.forward({TileType.BLACK})
                self.assertIs(result.status, MoveStatus.OK)
                self.assertAlmostEqual(hw.body.y, 1.5 * TILE, delta=3.0)
                self.assertLess(abs(hw.body.x - 0.5 * TILE), 4.0)


class BackwardDrivingTest(unittest.TestCase):
    """Rückwärtsfahren (RealRobot.backward) in der Fahrphysik."""

    def setup(self, world: SimWorld, x=None, y=None, heading=0.0, slip=False, noise=False, seed=0):
        hw = SimHardware(world, noise=noise, slip=slip)
        hw.body.rng = random.Random(seed)
        hw.ultrasonic.rng = random.Random(seed + 1000)
        if x is not None:
            hw.body.x = x
        if y is not None:
            hw.body.y = y
        hw.body.heading = heading
        robot = SimDrivingRobot(hw)
        robot.start_clock()
        return hw, robot

    def test_off_center_and_skewed_arrives_on_tile_center(self):
        """1,5 cm außermittig und 3° schräg (Kettenfaktor 1/(1 − 0,043), Rauschen, Schlupf): kommt an der Mitte der
        Kachel hinter ihm an, ohne Kollision und ohne Abbruch."""
        trim = 1 / (1 - 0.043)
        for off, ang in ((1.5, 3.0), (-1.5, 3.0), (1.5, -3.0), (-1.5, -3.0)):
            with self.subTest(off=off, ang=ang), mock.patch.object(config, "MOTOR_TRIM_RIGHT", trim):
                hw, robot = self.setup(corridor(4), x=0.5 * TILE + off, y=2.5 * TILE, heading=ang, slip=True, noise=True)
                result = robot.backward()
                self.assertIs(result.status, MoveStatus.OK)
                self.assertAlmostEqual(hw.body.y, 1.5 * TILE, delta=3.0)
                self.assertAlmostEqual(hw.body.x, 0.5 * TILE, delta=3.0)
                self.assertLess(abs(hw.body.heading), 4.0)
                self.assertEqual(hw.body.lop, 0)
                self.assertEqual(hw.body.tile, (0, 1))

    def test_noise_free_in_corridor_is_exact(self):
        hw, robot = self.setup(corridor(4), y=2.5 * TILE)
        self.assertIs(robot.backward().status, MoveStatus.OK)
        self.assertAlmostEqual(hw.body.y, 1.5 * TILE, delta=1.0)
        self.assertLess(hw.body.heading, 1.5)

    def test_stops_by_front_distance_not_by_encoder(self):
        """Wand vorne in Reichweite: der Halt folgt dem wachsenden Abstand vorne. Mit starkem Schlupf (Encoder zählen
        13 % mehr als er fährt) hält er trotzdem auf der Kachelmitte, nach Encoder allein käme er zu kurz."""
        with mock.patch("simulation.sim_hardware.DRIVE_SLIP", 0.25):
            hw, robot = self.setup(corridor(4), y=2.5 * TILE, slip=True)
            seen = []
            robot.trace = lambda what, **v: seen.append(v) if what == "rückwärts" else None
            robot.backward()
            self.assertEqual(seen[0]["halt_nach"], "Halt nach Abstand vorne")
            by_front = hw.body.y
            self.assertAlmostEqual(by_front, 1.5 * TILE, delta=2.5)
            # Zum Vergleich ohne Wand in Reichweite (nur Encoder): bleibt wegen des Schlupfs kürzer
            with mock.patch.object(config, "FRONT_TRUST_CM", 1.0):
                hw2, robot2 = self.setup(corridor(4), y=2.5 * TILE, slip=True)
                robot2.backward()
            self.assertGreater(hw2.body.y, by_front + 3.0)          # weiter vorn = zu kurz zurückgefahren

    def test_front_stop_distance_back_formula(self):
        from hardware.robot import stop_distance, stop_distance_back
        # Mitte einer Kachel, Wand vorne in der eigenen Kachel (FRONT_CENTER_CM): eine Kachel zurück = 30 cm
        self.assertAlmostEqual(stop_distance_back(config.FRONT_CENTER_CM, 0.0), TILE)
        # nach 10 cm ist der Abstand um 10 größer: Haltepunkt bleibt bei 30
        self.assertAlmostEqual(stop_distance_back(config.FRONT_CENTER_CM + 10, 10.0), TILE)
        # mehrere Kacheln bis zur Wand: die Mitte einer Kachel, die eine Kachellänge weiter weg liegt
        self.assertAlmostEqual(stop_distance_back(config.FRONT_CENTER_CM + 2 * TILE, 0.0), TILE)
        # ohne Wand in Reichweite: keine Aussage
        self.assertIsNone(stop_distance_back(None, 0.0))
        self.assertIsNone(stop_distance_back(config.FRONT_TRUST_CM + 1, 0.0))
        # Gegenstück vorwärts
        self.assertAlmostEqual(stop_distance(config.FRONT_CENTER_CM + TILE, 0.0), TILE)

    def test_backward_into_wall_is_blocked_and_returns(self):
        """Die Karte irrt: hinter dem Roboter ist eine Wand. Ketten blockiert, er fährt zur Startlage zurück (vorwärts),
        Status BLOCKED (der Explorer trägt die Wand hinten ein)."""
        hw, robot = self.setup(corridor(3), y=0.5 * TILE)       # unterste Kachel: hinten Randwand
        result = robot.backward()
        self.assertIs(result.status, MoveStatus.BLOCKED)
        self.assertAlmostEqual(hw.body.y, 0.5 * TILE, delta=3.0)
        self.assertEqual(hw.body.lop, 0)
        self.assertLess(robot.time(), config.TIME_LIMIT_S)

    def test_arrival_counts_as_visit(self):
        hw, robot = self.setup(corridor(3, {1: "blue"}), y=2.5 * TILE)
        robot.backward()
        self.assertEqual(hw.body.tile, (0, 1))
        self.assertEqual(hw.body.visits[(0, 1)], 1)

    def test_explorer_dead_end_without_turning_around(self):
        """Sackgasse oben, Start unten: der Heimweg ist rückwärts, der Roboter wendet nie."""
        turns = []
        world = corridor(3)
        hw = SimHardware(world)
        robot = SimDrivingRobot(hw)
        for name in ("turn_left", "turn_right"):
            orig = getattr(robot, name)
            setattr(robot, name, lambda n=name, o=orig: (turns.append(n), o())[1])
        explorer = Explorer(robot)
        robot.start_clock()
        explorer.run()
        self.assertEqual(explorer.mode, "fertig")
        self.assertEqual(hw.body.tile, (0, 0))
        self.assertEqual(turns, ["turn_right", "turn_left"])    # nur die Messdrehung am Start und zurück, kein Wenden
        self.assertEqual(hw.body.lop, 0)


class ExploreTest(unittest.TestCase):
    def check(self, world: SimWorld, hw: SimHardware, explorer: Explorer):
        visited = {world.to_world(p) for p in explorer.maze.visited}
        self.assertEqual(explorer.mode, "fertig")
        self.assertEqual(visited, world.reachable_tiles())
        self.assertEqual(hw.body.tile, world.start)
        self.assertEqual(hw.body.lop, 0)

    def test_exact(self):
        for name in ("einfach", "sonderboeden"):
            with self.subTest(name):
                world = SimWorld.load(name)
                self.check(world, *explore(world))

    def test_noise_and_slip(self):
        """Mit der gemessenen Drift nach rechts (DRIFT_BIAS), ausgeglichen durch den Kettenfaktor."""
        world = SimWorld.load("einfach")
        with mock.patch.object(config, "MOTOR_TRIM_RIGHT", 1 / (1 - 0.043)):
            for seed in range(2):
                with self.subTest(seed=seed):
                    self.check(world, *explore(world, noise=True, slip=True, seed=seed))


if __name__ == "__main__":
    unittest.main()


class DeadEndTest(unittest.TestCase):
    """Sackgasse: vor dem Wenden mittig zurücksetzen (Ecken schwenken auf 12,8 cm, Wand 14 cm von der Mitte)."""

    def setup_dead_end(self, x_offset: float, front_cm: float):
        tiles = [{"x": 0, "y": 0, "walls": "EW", "floor": "normal"}, {"x": 0, "y": 1, "walls": "EWN", "floor": "normal"}]
        world = SimWorld.from_dict({"width": 1, "height": 2, "start": [0, 1], "tiles": tiles})
        hw = SimHardware(world)
        robot = SimDrivingRobot(hw)
        robot.start_clock()
        hw.body.x = TILE / 2 + x_offset
        hw.body.y = 2 * TILE - 1 - (front_cm + config.ROBOT_LENGTH_CM / 2)   # Innenseite der Nordwand bei 2·TILE - 1
        robot.rear_free_cm = TILE               # er ist von hinten gekommen (offene Kachel dahinter)
        return hw, robot

    def measure(self, hw):
        return {side: hw.body.distance(side) for side in ("front", "left", "right")}

    def test_backs_up_to_tile_centre_distance(self):
        hw, robot = self.setup_dead_end(0.0, 3.5)
        robot.read_walls()
        d = self.measure(hw)
        self.assertGreaterEqual(d["front"], config.FRONT_CENTER_CM - 0.8)
        self.assertAlmostEqual(hw.body.heading, 0.0, delta=1.5)

    def test_side_offset_is_reduced_without_turning_the_robot(self):
        for offset in (2.5, -2.5):
            hw, robot = self.setup_dead_end(offset, 4.0)
            before = abs(hw.body.x - TILE / 2)
            robot.read_walls()
            self.assertLess(abs(hw.body.x - TILE / 2), before - 0.8)         # deutlich mittiger
            self.assertAlmostEqual(hw.body.heading, 0.0, delta=1.5)           # wieder gerade

    def test_turn_180_works_with_centering_but_hangs_without(self):
        """Die Ecken streifen beim Drehen die Wand (Ketten rutschen, er dreht nicht): wie am echten Roboter."""
        for offset, front in ((2.7, 4.0), (-2.7, 4.0)):
            hw, robot = self.setup_dead_end(offset, front)
            robot.read_walls()
            robot.turn_left()
            robot.turn_left()
            self.assertAlmostEqual(hw.body.heading % 360, 180.0, delta=3.0)
            hw, robot = self.setup_dead_end(offset, front)
            with mock.patch.object(type(robot), "_center_before_turn", lambda self, values, dead_end=False: None):
                robot.read_walls()
                robot.turn_left()
                robot.turn_left()
            self.assertGreater(abs((hw.body.heading % 360) - 180.0), 10.0)

    def test_u_turn_second_turn_does_not_push_into_the_old_side_wall(self):
        """Sackgasse, zwei Drehungen: vor der zweiten steht hinten die alte Seitenwand (kein Sensor dort). Er setzt dort
        nicht blind zurück (Läufe 2026-11-11: 12 cm gegen 5 cm Luft, danach 6-9 cm zu weit vor und schräg)."""
        for offset, front in ((2.7, 4.0), (-2.7, 4.0), (0.0, 4.0), (1.5, 5.5)):
            hw, robot = self.setup_dead_end(offset, front)
            robot.read_walls()
            robot.turn_left()
            y_before = hw.body.y
            rooms, back = [], []
            room, drive = robot._rear_room, robot.drive_cm
            robot._rear_room = lambda front: rooms.append(room(front)) or rooms[-1]
            robot.drive_cm = lambda cm: (back.append(-cm) if cm < 0 else None, drive(cm))[1]
            robot.turn_left()
            self.assertLessEqual(sum(back), (max(rooms) if rooms else 0.0) + 0.01)      # nie mehr als hinten frei ist
            self.assertAlmostEqual(hw.body.heading % 360, 180.0, delta=4.0)
            self.assertAlmostEqual(hw.body.x, TILE / 2, delta=3.0)
            self.assertAlmostEqual(hw.body.y, y_before, delta=3.0)      # die zweite Drehung schiebt ihn nicht weg

    def test_centered_robot_stays_put(self):
        hw, robot = self.setup_dead_end(0.0, config.FRONT_CENTER_CM)
        x, y = hw.body.x, hw.body.y
        robot.read_walls()
        self.assertAlmostEqual(hw.body.y, y, delta=0.8)
        self.assertAlmostEqual(hw.body.x, x, delta=0.5)

    def test_open_side_does_not_trigger(self):
        hw = SimHardware(corridor(3))
        robot = SimDrivingRobot(hw)
        robot.start_clock()
        y = hw.body.y
        robot.read_walls()
        self.assertEqual(hw.body.y, y)


class TurnCenteringTest(unittest.TestCase):
    """Vor jeder 90°-Drehung zentrieren, wenn eine Ecke anstoßen würde (nicht nur in der Sackgasse)."""

    def setup(self, tiles, width, x_offset, front_cm):
        world = SimWorld.from_dict({"width": width, "height": 2, "start": [0, 1], "tiles": tiles})
        hw = SimHardware(world)
        robot = SimDrivingRobot(hw)
        robot.start_clock()
        hw.body.x = TILE / 2 + x_offset
        hw.body.y = 2 * TILE - 1 - (front_cm + config.ROBOT_LENGTH_CM / 2)
        robot.rear_free_cm = TILE               # er ist von hinten gekommen
        return hw, robot

    def test_two_walls_off_centre_and_close_in_front(self):
        for offset in (2.7, -2.7):
            tiles = [{"x": 0, "y": 0, "walls": "EW", "floor": "normal"}, {"x": 0, "y": 1, "walls": "EWN", "floor": "normal"}]
            hw, robot = self.setup(tiles, 1, offset, 4.0)
            robot.turn_left()                                   # ohne read_walls: misst selbst
            self.assertAlmostEqual(hw.body.heading % 360, 270.0, delta=4.0)

    def test_one_side_wall_only(self):
        tiles = [{"x": x, "y": y, "walls": "WN" if (x, y) == (0, 1) else "", "floor": "normal"}
                 for x in range(2) for y in range(2)]
        hw, robot = self.setup(tiles, 2, -2.7, 4.0)             # Ecke links: Wand links 2,3 cm von der Mitte weg gerückt
        robot.turn_left()
        self.assertAlmostEqual(hw.body.heading % 360, 270.0, delta=4.0)

    def test_after_backward_centres_forward_when_rear_unknown(self):
        """Nach dem Rückwärtsfahren ist hinten nichts bekannt (`rear_free_cm` None), vorne ist frei: der Roboter steht 2,7 cm
        zu nah an der Wand und zentriert schräg nach VORNE (Trace `vorwärts`), statt mit der Ecke zu streifen."""
        tiles = [{"x": x, "y": y, "walls": "S" if y == 0 else "", "floor": "normal"} for x in range(3) for y in range(3)]
        world = SimWorld.from_dict({"width": 3, "height": 3, "start": [1, 0], "tiles": tiles})
        hw = SimHardware(world)
        robot = SimDrivingRobot(hw)
        robot.start_clock()
        hw.body.heading = 90.0                                   # nach Osten, Südwand rechts
        hw.body.x, hw.body.y = 1.5 * TILE, 0.5 * TILE - 2.7      # 2,7 cm zu nah an der Südwand
        robot.rear_free_cm, robot.front_free_cm = None, TILE
        seen = []
        robot.trace = lambda what, **v: seen.append(v) if what == "zentriert_vor_drehung" else None
        robot.turn_left()
        self.assertEqual(seen and seen[0].get("vorwärts"), True)
        self.assertLess(abs((hw.body.heading + 180) % 360 - 180), 4.0)      # nach links gedreht: Norden
        self.assertGreater(hw.body.y, 0.5 * TILE - 1.5)          # näher an der Mitte als vorher

    def test_open_area_does_not_centre(self):
        hw = SimHardware(SimWorld.from_dict({"width": 3, "height": 3, "start": [1, 1],
                                             "tiles": [{"x": x, "y": y, "walls": "", "floor": "normal"}
                                                       for x in range(3) for y in range(3)]}))
        robot = SimDrivingRobot(hw)
        robot.start_clock()
        calls = []
        original = hw.motors.set_dps
        hw.motors.set_dps = lambda l, r: (calls.append((l, r)), original(l, r))[1]
        robot.turn_left()
        self.assertTrue(calls)
        self.assertTrue(all(l == -r for l, r in calls))         # nur Drehen auf der Stelle, kein Zurücksetzen
