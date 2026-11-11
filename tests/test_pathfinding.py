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

    def test_avoids_visited_blue_if_possible(self):
        # Zwei Wege von (0, 0) nach (2, 0): direkt über die besuchte blaue (1, 0) oder oben herum.
        m = MazeMap()
        for pos in [(0, 0), (1, 0), (2, 0), (0, 1), (1, 1), (2, 1)]:
            m.mark_visited(pos, TileType.NORMAL)
        m.mark_visited((1, 0), TileType.BLUE)
        for pos, d in [((0, 0), Dir.E), ((1, 0), Dir.E), ((0, 0), Dir.N),
                       ((0, 1), Dir.E), ((1, 1), Dir.E), ((2, 1), Dir.S)]:
            m.set_wall(pos, d, False)
        actions = plan_path(m, (0, 0), Dir.E, {(2, 0)})
        self.assertNotEqual(actions, ["F", "F"])

    def test_dead_end_one_tile_deep_goes_backward(self):
        """Sackgasse eine Kachel tief (besucht): zurück ist "B" (1,5) statt Wenden + vorwärts (3)."""
        m = corridor(2)                     # (0, 0) und (0, 1) besucht, (0, 1) hat vorne Wand
        m.set_wall((0, 1), Dir.N, True)
        m.set_wall((0, 0), Dir.S, False)    # hinter dem Start liegt eine (unbesuchte) Zielkachel
        self.assertEqual(plan_path(m, (0, 1), Dir.N, {(0, 0)}), ["B"])
        # Kosten in Sekunden: B (5,0) bleibt deutlich billiger als Wenden (2 Drehungen à 3,0) + vorwärts (3,5) = 9,5
        self.assertEqual(config.PLAN_COST_BACKWARD, config.MOVE_BACK_TIME_S + config.PLAN_BACK_PENALTY_S)
        self.assertLess(config.PLAN_COST_BACKWARD, 2 * config.PLAN_COST_TURN + config.PLAN_COST_FORWARD)

    def test_dead_end_two_tiles_deep_is_bb(self):
        m = corridor(3)
        m.set_wall((0, 2), Dir.N, True)
        self.assertEqual(plan_path(m, (0, 2), Dir.N, {(0, 0)}), ["B", "B"])

    def test_not_backward_onto_unvisited_tile(self):
        """Hinter dem Roboter liegt eine unbesuchte Kachel (Farbsensor sitzt vorne, Schwarz würde nicht erkannt):
        nicht rückwärts planen, sondern wenden und vorwärts."""
        m = corridor(1)
        m.set_wall((0, 0), Dir.S, False)    # offen, aber (0, -1) unbesucht
        actions = plan_path(m, (0, 0), Dir.N, {(0, -1)})
        self.assertNotIn("B", actions)
        self.assertEqual(actions.count("F"), 1)
        self.assertEqual(len(actions), 3)       # zwei Drehungen + vorwärts

    def test_backward_needs_open_wall(self):
        m = corridor(2)
        m.set_wall((0, 0), Dir.N, True)     # Wand zwischen (0, 0) und (0, 1)
        self.assertIsNone(plan_path(m, (0, 1), Dir.N, {(0, 0)}))

    def test_backward_over_blue_costs_like_forward(self):
        m = corridor(2)
        m.mark_visited((0, 0), TileType.BLUE)
        m.set_wall((0, 1), Dir.N, True)
        self.assertEqual(plan_path(m, (0, 1), Dir.N, {(0, 0)}), ["B"])   # einziger Weg (hier sonst nur Drehen)
        self.assertEqual(estimate_time(m, (0, 1), Dir.N, ["B"]), config.MOVE_BACK_TIME_S + config.BLUE_WAIT_S)

    def test_follow_backward(self):
        from navigation.pathfinding import follow
        self.assertEqual(follow((0, 2), Dir.N, ["B", "B"]), [((0, 2), Dir.N), ((0, 1), Dir.N), ((0, 0), Dir.N)])

    def square(self) -> MazeMap:
        """Vier besuchte Kacheln im Quadrat, alle Kanten offen: (0,0)-(1,0) direkt oder über (0,1) und (1,1) herum."""
        m = MazeMap()
        for pos in [(0, 0), (1, 0), (0, 1), (1, 1)]:
            m.mark_visited(pos, TileType.NORMAL)
        for pos, d in [((0, 0), Dir.E), ((0, 0), Dir.N), ((1, 0), Dir.N), ((0, 1), Dir.E)]:
            m.set_wall(pos, d, False)
        return m

    def test_blocked_edge_prefers_detour(self):
        m = self.square()
        self.assertEqual(plan_path(m, (0, 0), Dir.E, {(1, 0)}), ["F"])
        m.mark_blocked((0, 0), Dir.E)       # einmal blockiert: +30 s, der Umweg (3 Kacheln + 2 Drehungen = 19,5 s) ist billiger
        actions = plan_path(m, (0, 0), Dir.E, {(1, 0)})
        self.assertNotEqual(actions, ["F"])
        self.assertEqual(actions.count("F"), 3)

    def test_blocked_edge_without_detour_still_used(self):
        m = corridor(2)
        m.mark_blocked((0, 0), Dir.N)
        self.assertEqual(plan_path(m, (0, 0), Dir.N, {(0, 1)}), ["F"])

    def test_no_backward_over_blocked_edge(self):
        m = corridor(2)
        m.set_wall((0, 1), Dir.N, True)
        self.assertEqual(plan_path(m, (0, 1), Dir.N, {(0, 0)}), ["B"])
        m.mark_blocked((0, 0), Dir.N)
        actions = plan_path(m, (0, 1), Dir.N, {(0, 0)})
        self.assertNotIn("B", actions)
        self.assertEqual(actions, ["L", "L", "F"])

    def test_costs_are_seconds(self):
        self.assertEqual(config.PLAN_COST_FORWARD, config.MOVE_TIME_S)
        self.assertEqual(config.PLAN_COST_TURN, config.TURN_TIME_S + config.PLAN_TURN_PENALTY_S)

    def test_estimate_time_has_no_penalties(self):
        m = corridor(2)
        self.assertEqual(estimate_time(m, (0, 0), Dir.N, ["L", "F"]), config.TURN_TIME_S + config.MOVE_TIME_S)

    def test_estimate_time_with_measured_times(self):
        m = corridor(3)
        times = {"F": 5.0, "B": 6.0, "T": 4.0}
        self.assertEqual(estimate_time(m, (0, 0), Dir.N, ["F", "F", "R", "R", "B"], times), 10.0 + 8.0 + 6.0)
        self.assertEqual(estimate_time(m, (0, 0), Dir.N, ["F"], {"F": 5.0}), 5.0)    # fehlende Arten: config

    def test_estimate_time(self):
        m = corridor(2)
        expected = 2 * config.MOVE_TIME_S + 2 * config.TURN_TIME_S
        self.assertEqual(estimate_time(m, (0, 0), Dir.N, ["F", "R", "R", "F"]), expected)


if __name__ == "__main__":
    unittest.main()
