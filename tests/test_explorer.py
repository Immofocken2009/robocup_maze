import unittest

import config
from navigation.explorer import START, Explorer
from navigation.maze_map import Dir, TileType
from simulation.maze_generator import generate
from simulation.sim_robot import SimRobot, SimWorld, list_mazes

NO_LIMIT = 10**6


def explore(world: SimWorld, robot_class=SimRobot, time_limit_s=NO_LIMIT, **kwargs):
    robot = robot_class(world, time_limit_s=max(time_limit_s, config.TIME_LIMIT_S))
    explorer = Explorer(robot, time_limit_s=time_limit_s, **kwargs)
    explorer.run()
    return robot, explorer


def visited_in_world(world: SimWorld, explorer: Explorer) -> set:
    return {world.to_world(p) for p in explorer.maze.visited}


class ExplorerTest(unittest.TestCase):
    def test_saved_mazes_fully_explored(self):
        for name in list_mazes():
            with self.subTest(maze=name):
                world = SimWorld.load(name)
                robot, explorer = explore(world)
                self.assertEqual(visited_in_world(world, explorer), world.reachable_tiles())
                self.assertEqual(robot.pos, world.start)
                self.assertEqual(explorer.pos, START)
                self.assertEqual(explorer.mode, "fertig")
                self.assertTrue(robot.exit_done)
                self.assertEqual(robot.lop, 0)

    def test_map_matches_reality(self):
        for name in list_mazes():
            with self.subTest(maze=name):
                world = SimWorld.load(name)
                _, explorer = explore(world)
                for pos, d, present in explorer.maze.walls():
                    wpos, wd = world.to_world(pos), world.to_world_dir(d)
                    self.assertEqual(present, world.wall(wpos, wd), f"Wand {wpos} {wd.name}")
                for pos, kind in explorer.maze.tiles.items():
                    self.assertEqual(kind, world.tiles[world.to_world(pos)])

    def test_random_mazes_all_red_modes(self):
        for seed in range(25):
            world = SimWorld.from_dict(generate(8, 7, zones=1, seed=seed)[0])
            for mode in config.RED_MODES:
                with self.subTest(seed=seed, mode=mode):
                    robot, explorer = explore(world, red_mode=mode)
                    if mode == "meiden":
                        expected = world.reachable_tiles(frozenset({TileType.BLACK, TileType.RED}))
                    else:
                        expected = world.reachable_tiles()
                    self.assertEqual(visited_in_world(world, explorer), expected)
                    self.assertEqual(robot.pos, world.start)
                    self.assertEqual(robot.lop, 0)
                    self.assertTrue(robot.exit_done)

    def test_red_last_enters_zone_after_everything_else(self):
        world = SimWorld.from_dict(generate(8, 7, zones=1, seed=3)[0])
        red = next(p for p, k in world.tiles.items() if k is TileType.RED)
        outside = world.reachable_tiles(frozenset({TileType.BLACK, TileType.RED}))
        order = []

        class Recorder(SimRobot):
            def forward(self, avoid):
                result = super().forward(avoid)
                order.append(self.pos)
                return result

        explore(world, robot_class=Recorder, red_mode="zuletzt")
        first_red = order.index(red)
        self.assertTrue(outside <= set(order[:first_red]) | {world.start})

    def test_time_limit_returns_home_in_time(self):
        world = SimWorld.load("gross")
        robot, explorer = explore(world, time_limit_s=150)
        self.assertEqual(robot.pos, world.start)
        self.assertTrue(robot.exit_done)
        self.assertLessEqual(robot.sim_time, 150)
        self.assertLess(len(explorer.maze.visited), len(world.reachable_tiles()))

    def test_recovers_from_wrong_wall_reading(self):
        class LyingRobot(SimRobot):
            """Meldet bei der ersten Messung vorne fälschlich 'offen'."""
            lied = False

            def read_walls(self):
                walls = super().read_walls()
                if not self.lied:
                    self.lied = True
                    walls["front"] = False
                return walls

        world = SimWorld.load("einfach")
        robot, explorer = explore(world, robot_class=LyingRobot)
        self.assertEqual(visited_in_world(world, explorer), world.reachable_tiles())
        self.assertEqual(robot.pos, world.start)


class Interrupted(Exception):
    """Steht im Test für den Druck auf K3."""


class LackOfProgressTest(unittest.TestCase):
    def run_with_lop(self, world: SimWorld, robot_class) -> tuple[SimRobot, Explorer]:
        robot = robot_class(world, time_limit_s=NO_LIMIT)
        explorer = Explorer(robot, time_limit_s=NO_LIMIT)
        with self.assertRaises(Interrupted):
            explorer.run()
        explorer.lack_of_progress()
        # Der Schiedsrichter setzt den Roboter dorthin, wo der Explorer ihn jetzt vermutet
        robot.pos = world.to_world(explorer.pos)
        robot.dir = world.to_world_dir(explorer.heading)
        explorer.resume()
        return robot, explorer

    def test_resumes_from_last_checkpoint(self):
        class CheckpointThenStuck(SimRobot):
            """Zwei Kacheln nach dem ersten Checkpoint: K3."""
            after_checkpoint = None

            def forward(self, avoid):
                if self.after_checkpoint is not None:
                    self.after_checkpoint += 1
                    if self.after_checkpoint == 3:
                        raise Interrupted
                result = super().forward(avoid)
                if self.after_checkpoint is None and self.world.tiles[self.pos] is TileType.SILVER:
                    self.after_checkpoint = 0
                return result

        world = SimWorld.load("sonderboeden")
        robot, explorer = self.run_with_lop(world, CheckpointThenStuck)
        self.assertIs(world.tiles[world.to_world(explorer.last_checkpoint)], TileType.SILVER)
        self.assertEqual(visited_in_world(world, explorer), world.reachable_tiles())
        self.assertEqual(robot.pos, world.start)
        self.assertEqual(explorer.mode, "fertig")

    def test_without_checkpoint_back_to_start(self):
        class StuckAtOnce(SimRobot):
            stuck = False

            def forward(self, avoid):
                if not self.stuck:
                    self.stuck = True
                    raise Interrupted
                return super().forward(avoid)

        world = SimWorld.load("einfach")
        robot, explorer = self.run_with_lop(world, StuckAtOnce)
        self.assertEqual(visited_in_world(world, explorer), world.reachable_tiles())
        self.assertEqual(robot.pos, world.start)


class ScoreTest(unittest.TestCase):
    def test_blue_revisit_reduces_points(self):
        world = SimWorld.load("sonderboeden")
        robot = SimRobot(world)
        blue = [p for p, k in world.tiles.items() if k is TileType.BLUE]
        robot.visits[blue[0]] = 1
        robot.visits[blue[1]] = 3
        score = robot.score()
        self.assertEqual(score["blue"], 30 + 10)
        self.assertEqual(score["reliability"], 10)   # nur eine blaue Kachel genau einmal besucht
        self.assertEqual(score["exit"], 0)

    def test_checkpoints_and_exit(self):
        world = SimWorld.load("sonderboeden")
        robot, _ = explore(world)
        silver = sum(1 for k in world.tiles.values() if k is TileType.SILVER)
        score = robot.score()
        self.assertEqual(score["checkpoints"], 10 * silver)
        self.assertTrue(robot.exit_done)



class AbortModeTest(unittest.TestCase):
    def corridor(self) -> SimWorld:
        tiles = [{"x": 0, "y": y, "walls": "EW", "floor": "normal"} for y in range(3)]
        return SimWorld.from_dict({"width": 1, "height": 3, "start": [0, 0], "tiles": tiles})

    def test_complete_run_is_fertig(self):
        _, explorer = explore(self.corridor())
        self.assertEqual(explorer.mode, "fertig")
        self.assertIsNone(explorer.stop_reason)

    def test_start_not_reachable_is_abgebrochen(self):
        class BlockedWay(SimRobot):
            explorer = None
            calls = 0

            def forward(self, avoid):
                result = super().forward(avoid)
                self.calls += 1
                if self.calls == 2:     # Karte widerspricht jetzt: Rückweg zum Start gesperrt
                    self.explorer.maze.set_wall((0, 0), Dir.N, True)
                return result

        robot = BlockedWay(self.corridor(), time_limit_s=NO_LIMIT)
        explorer = Explorer(robot, time_limit_s=NO_LIMIT)
        robot.explorer = explorer
        explorer.run()
        self.assertTrue(explorer.mode.startswith("abgebrochen:"), explorer.mode)
        self.assertIn("Start nicht erreicht", explorer.mode)

    def test_detected_black_on_visited_silver_is_contradiction(self):
        from navigation.robot_interface import MoveResult, MoveStatus

        class Avoids(SimRobot):
            def forward(self, avoid):
                return MoveResult(MoveStatus.AVOIDED, TileType.BLACK)

        explorer = Explorer(Avoids(self.corridor(), time_limit_s=NO_LIMIT), time_limit_s=NO_LIMIT)
        explorer.maze.mark_visited((0, 1), TileType.SILVER)
        explorer._forward()                      # Ziel (0, 1): Roboter meldet „schwarz“
        self.assertIs(explorer.maze.kind((0, 1)), TileType.SILVER)
        self.assertEqual(explorer.contradictions, 1)


if __name__ == "__main__":
    unittest.main()
