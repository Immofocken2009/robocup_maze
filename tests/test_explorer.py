import unittest

from navigation.explorer import START, Explorer
from simulation.sim_robot import SimRobot, SimWorld, list_mazes


def explore(name: str, robot_class=SimRobot, **kwargs):
    world = SimWorld.load(name)
    robot = robot_class(world)
    explorer = Explorer(robot, **kwargs)
    explorer.run()
    return world, robot, explorer


class ExplorerTest(unittest.TestCase):
    def test_all_mazes_fully_explored(self):
        for name in list_mazes():
            with self.subTest(maze=name):
                world, robot, explorer = explore(name)
                visited = {world.to_world(p) for p in explorer.maze.visited}
                self.assertEqual(visited, world.reachable_tiles())
                self.assertEqual(robot.pos, world.start)
                self.assertEqual(explorer.pos, START)
                self.assertEqual(explorer.mode, "fertig")

    def test_map_matches_reality(self):
        for name in list_mazes():
            with self.subTest(maze=name):
                world, _, explorer = explore(name)
                for pos, d, present in explorer.maze.walls():
                    wpos, wd = world.to_world(pos), world.to_world_dir(d)
                    self.assertEqual(present, world.wall(wpos, wd), f"Wand {wpos} {wd.name}")
                for pos, kind in explorer.maze.tiles.items():
                    self.assertEqual(kind, world.tiles[world.to_world(pos)])

    def test_time_limit_returns_home_in_time(self):
        world, robot, explorer = explore("gross", time_limit_s=90)
        self.assertEqual(robot.pos, world.start)
        self.assertLessEqual(robot.sim_time, 90)
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

        world, robot, explorer = explore("einfach", robot_class=LyingRobot)
        visited = {world.to_world(p) for p in explorer.maze.visited}
        self.assertEqual(visited, world.reachable_tiles())
        self.assertEqual(robot.pos, world.start)


if __name__ == "__main__":
    unittest.main()
