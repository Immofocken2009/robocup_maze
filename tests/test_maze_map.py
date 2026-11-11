import unittest

import config
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


class VoteTest(unittest.TestCase):
    def test_votes_decide(self):
        m = MazeMap()
        self.assertIsNone(m.wall((0, 0), Dir.N))
        m.vote_wall((0, 0), Dir.N, True)
        self.assertIs(m.wall((0, 0), Dir.N), True)
        self.assertIs(m.wall((0, 1), Dir.S), True)          # dieselbe Kante

    def test_two_opposite_votes_flip(self):
        m = MazeMap()
        self.assertFalse(m.vote_wall((0, 0), Dir.N, True))
        self.assertFalse(m.vote_wall((0, 0), Dir.N, False))     # Gleichstand: bisheriger Wert bleibt (nicht „unbekannt“)
        self.assertIs(m.wall((0, 0), Dir.N), True)
        self.assertTrue(m.vote_wall((0, 0), Dir.N, False))      # kippt: Wand -> offen
        self.assertIs(m.wall((0, 0), Dir.N), False)
        self.assertFalse(m.vote_wall((0, 0), Dir.N, False))     # kein weiterer Wechsel

    def test_votes_are_clamped(self):
        m = MazeMap()
        for _ in range(10):
            m.vote_wall((0, 0), Dir.E, True)
        for _ in range(config.WALL_VOTE_HARD):
            m.vote_wall((0, 0), Dir.E, False)
        self.assertIs(m.wall((0, 0), Dir.E), True)              # Gleichstand: Wand bleibt
        self.assertTrue(m.vote_wall((0, 0), Dir.E, False))      # nicht 7 Gegenstimmen nötig: die nächste kippt
        self.assertIs(m.wall((0, 0), Dir.E), False)

    def test_hard_set_needs_several_opposite_votes(self):
        m = MazeMap()
        m.set_wall((0, 0), Dir.N, True)
        for _ in range(config.WALL_VOTE_HARD):
            m.vote_wall((0, 0), Dir.N, False)
        self.assertIs(m.wall((0, 0), Dir.N), True)              # Gleichstand: bleibt
        self.assertTrue(m.vote_wall((0, 0), Dir.N, False))
        self.assertIs(m.wall((0, 0), Dir.N), False)

    def test_walls_listing_has_known_edges_only(self):
        m = MazeMap()
        m.vote_wall((0, 0), Dir.N, True)
        m.vote_wall((0, 0), Dir.E, True)
        m.vote_wall((0, 0), Dir.E, False)                       # Gleichstand: Wand bleibt bekannt
        self.assertEqual(sorted(m.walls()), [((0, 0), Dir.N, True), ((0, 0), Dir.E, True)])
        self.assertIsNone(m.wall((5, 5), Dir.N))                # nie gemessen: unbekannt


class BlockedTest(unittest.TestCase):
    def test_counter_reaches_limit(self):
        m = MazeMap()
        self.assertEqual(m.blocked((0, 0), Dir.N), 0)
        self.assertFalse(m.mark_blocked((0, 0), Dir.N))         # 1 von WALL_AFTER_BLOCKED (2)
        self.assertEqual(m.blocked((0, 1), Dir.S), 1)           # dieselbe Kante von der anderen Seite
        self.assertTrue(m.mark_blocked((0, 1), Dir.S))
        self.assertEqual(m.blocked((0, 0), Dir.N), 2)

    def test_clear(self):
        m = MazeMap()
        m.mark_blocked((0, 0), Dir.N)
        m.clear_blocked((0, 0), Dir.N)
        self.assertEqual(m.blocked((0, 0), Dir.N), 0)


if __name__ == "__main__":
    unittest.main()
