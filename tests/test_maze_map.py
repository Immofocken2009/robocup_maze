import unittest

from navigation.maze_map import Dir, MazeMap, TileType, step


class DirTest(unittest.TestCase):
    def test_rotations(self):
        self.assertEqual(Dir.N.right(), Dir.E)
        self.assertEqual(Dir.N.left(), Dir.W)
        self.assertEqual(Dir.W.right(), Dir.N)
        self.assertEqual(Dir.E.back(), Dir.W)

    def test_step(self):
        self.assertEqual(step((0, 0), Dir.N), (0, 1))
        self.assertEqual(step((0, 0), Dir.E), (1, 0))
        self.assertEqual(step((0, 0), Dir.S), (0, -1))
        self.assertEqual(step((0, 0), Dir.W), (-1, 0))


class MazeMapTest(unittest.TestCase):
    def test_wall_is_shared_by_both_tiles(self):
        m = MazeMap()
        m.set_wall((0, 0), Dir.E, True)
        self.assertTrue(m.wall((1, 0), Dir.W))
        self.assertIsNone(m.wall((0, 0), Dir.N))

    def test_walls_listing(self):
        m = MazeMap()
        m.set_wall((0, 0), Dir.W, False)
        self.assertEqual(m.walls(), [((-1, 0), Dir.E, False)])

    def test_frontier(self):
        m = MazeMap()
        m.mark_visited((0, 0), TileType.NORMAL)
        m.set_wall((0, 0), Dir.N, False)
        m.set_wall((0, 0), Dir.E, False)
        m.set_wall((0, 0), Dir.S, True)
        self.assertEqual(m.frontier(), {(0, 1), (1, 0)})

    def test_black_tile_is_not_frontier(self):
        m = MazeMap()
        m.mark_visited((0, 0), TileType.NORMAL)
        m.set_wall((0, 0), Dir.N, False)
        m.mark_detected((0, 1), TileType.BLACK)
        self.assertEqual(m.frontier(), set())

    def test_detected_does_not_overwrite_visited_tile(self):
        m = MazeMap()
        m.mark_visited((0, 1), TileType.SILVER)
        self.assertFalse(m.mark_detected((0, 1), TileType.BLACK))
        self.assertIs(m.kind((0, 1)), TileType.SILVER)
        self.assertTrue(m.mark_detected((0, 2), TileType.BLACK))
        self.assertFalse(m.mark_detected((0, 2), TileType.RED))
        self.assertIs(m.kind((0, 2)), TileType.BLACK)


if __name__ == "__main__":
    unittest.main()
