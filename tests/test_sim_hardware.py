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
            with mock.patch.object(type(robot), "_center_dead_end", lambda self, values: None):
                robot.read_walls()
                robot.turn_left()
                robot.turn_left()
            self.assertGreater(abs((hw.body.heading % 360) - 180.0), 10.0)

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
