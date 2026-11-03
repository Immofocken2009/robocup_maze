"""Tests für die Fahrphysik (simulation/sim_hardware.py) mit dem echten Fahrcode."""
import random
import unittest

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
        for y in range(15, 40):
            hw.body.y = y * 1.0
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

    def test_align_squares_up_at_front_wall(self):
        hw = SimHardware(corridor(1))
        hw.body.heading = 6.0                       # 6° schräg vor der Wand
        SimDrivingRobot(hw).align(front_wall=True, back_wall=False)
        self.assertAlmostEqual(hw.body.heading, 0.0, delta=1.0)
        self.assertAlmostEqual(hw.body.y, 0.5 * TILE, delta=1.5)

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
        world = SimWorld.load("einfach")
        for seed in range(2):
            with self.subTest(seed=seed):
                self.check(world, *explore(world, noise=True, slip=True, seed=seed))


if __name__ == "__main__":
    unittest.main()
