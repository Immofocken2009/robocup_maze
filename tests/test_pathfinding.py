import unittest

import config
from navigation.maze_map import Dir, MazeMap, TileType
from navigation.pathfinding import estimate_time, plan_path


def corridor(length: int) -> MazeMap:
    """Besuchter Gang von (0, 0) nach Norden, offen bis (0, length)."""
    m = MazeMap()
    for y in range(length):
        m.mark_visited((0, y), TileType.NORMAL)
        m.set_wall((0, y), Dir.N, False)
    return m


class PlanPathTest(unittest.TestCase):
    def test_straight(self):
        m = corridor(3)
        self.assertEqual(plan_path(m, (0, 0), Dir.N, {(0, 3)}), ["F", "F", "F"])

    def test_turns_around(self):
        m = corridor(2)
        actions = plan_path(m, (0, 1), Dir.S, {(0, 2)})
        self.assertEqual(actions.count("F"), 1)
        self.assertEqual(len(actions), 3)   # zwei Drehungen + vorwärts

    def test_already_there(self):
        self.assertEqual(plan_path(corridor(1), (0, 0), Dir.N, {(0, 0)}), [])

    def test_unreachable(self):
        m = corridor(1)
        self.assertIsNone(plan_path(m, (0, 0), Dir.N, {(5, 5)}))

    def test_does_not_drive_through_unvisited_tiles(self):
        m = corridor(1)
        m.set_wall((0, 1), Dir.N, False)    # (0, 1) ist offen, aber unbesucht
        self.assertIsNone(plan_path(m, (0, 0), Dir.N, {(0, 2)}))

    def test_avoids_blue_if_possible(self):
        # Zwei Wege von (0, 0) nach (2, 0): direkt über blaue (1, 0) oder oben herum.
        m = MazeMap()
        for pos in [(0, 0), (1, 0), (2, 0), (0, 1), (1, 1), (2, 1)]:
            m.mark_visited(pos, TileType.NORMAL)
        m.mark_visited((1, 0), TileType.BLUE)
        for pos, d in [((0, 0), Dir.E), ((1, 0), Dir.E), ((0, 0), Dir.N),
                       ((0, 1), Dir.E), ((1, 1), Dir.E), ((2, 1), Dir.S)]:
            m.set_wall(pos, d, False)
        old = config.PLAN_COST_BLUE
        try:
            config.PLAN_COST_BLUE = 100.0
            actions = plan_path(m, (0, 0), Dir.E, {(2, 0)})
        finally:
            config.PLAN_COST_BLUE = old
        self.assertNotEqual(actions, ["F", "F"])

    def test_estimate_time(self):
        m = corridor(2)
        expected = 2 * config.MOVE_TIME_S + config.TURN_TIME_S
        self.assertEqual(estimate_time(m, (0, 0), Dir.N, ["F", "F", "R"]), expected)


if __name__ == "__main__":
    unittest.main()
