"""Tests des Zentrierens vor der Drehung (centering_plan, Rückraum)."""
import unittest

import config
from hardware.robot import centering_plan
from navigation.robot_interface import MoveStatus
from tests.fakes import TILE, Body, make_robot


class CenteringPlanTest(unittest.TestCase):
    """Entscheidung vor der Drehung: vorne zurück ja/nein, seitlich ja/nein."""

    def test_front_close_only(self):
        back, side = centering_plan(4.0, 6.25, 6.25)
        self.assertAlmostEqual(back, config.FRONT_CENTER_CM - 4.0)
        self.assertEqual(side, 0.0)

    def test_side_close_with_room_on_the_other_side(self):
        back, side = centering_plan(12.0, 4.0, 9.0)             # links nah, rechts Platz: nach rechts, nur bis mittig
        self.assertEqual(back, 0.0)
        self.assertAlmostEqual(side, -2.5)
        back, side = centering_plan(12.0, 9.0, 4.0)
        self.assertAlmostEqual(side, 2.5)

    def test_open_area_or_no_echo_does_nothing(self):
        self.assertEqual(centering_plan(40.0, 35.0, None), (0.0, 0.0))
        self.assertEqual(centering_plan(None, None, None), (0.0, 0.0))
        self.assertEqual(centering_plan(12.0, 6.25, 6.25), (0.0, 0.0))     # mittig im Gang: nichts


class RearRoomTest(unittest.TestCase):
    """Rückraum: nie blind zurücksetzen (kein Sensor hinten, die Ketten rutschen an einer Wand, die Encoder zählen weiter)."""

    def make(self, rear):
        robot = make_robot(Body(wall_cm=None))
        robot.start_clock()
        robot.rear_free_cm = rear
        self.moves, self.trace = [], []
        robot.drive_cm = lambda cm: self.moves.append(("drive", round(cm, 2)))
        robot._rotate = lambda deg: self.moves.append(("rotate", round(deg, 2)))
        robot.trace = lambda what, **v: self.trace.append((what, v))
        return robot

    def test_known_after_forward_unknown_after_turn(self):
        robot = make_robot(Body(wall_cm=None))
        robot.start_clock()
        self.assertIs(robot.forward(set()).status, MoveStatus.OK)
        self.assertAlmostEqual(robot.rear_free_cm, TILE, delta=2.5)         # er kam von dort
        robot._rotate(90)
        self.assertIsNone(robot.rear_free_cm)
        robot.rear_free_cm = 20.0
        robot._rotate(8)                                                    # kleine Korrekturdrehung: bleibt
        self.assertEqual(robot.rear_free_cm, 20.0)
        robot.drive_cm(-5)
        self.assertAlmostEqual(robot.rear_free_cm, 15.0, delta=0.01)
        robot.drive_cm(-50)
        self.assertEqual(robot.rear_free_cm, 0.0)

    def test_estimate_from_front_distance_after_turn(self):
        robot = make_robot(Body(wall_cm=None))
        self.assertAlmostEqual(robot._rear_room(8.9), 2.1, places=6)        # 28 - 16,5 - 8,9 - 0,5
        self.assertEqual(robot._rear_room(20.0), 0.0)
        self.assertEqual(robot._rear_room(None), 0.0)                       # unbekannt: nie blind zurück
        robot.rear_free_cm = 30.0
        self.assertEqual(robot._rear_room(8.9), 30.0)                       # bekannt: dieser Wert

    def test_straight_back_limited(self):
        robot = self.make(1.0)
        robot._center_before_turn({"front": 4.0, "left": 6.25, "right": 6.25})
        self.assertEqual(self.moves, [("drive", -1.0)])
        self.assertEqual(self.trace[-1][0], "zentriert_vor_drehung")
        self.assertEqual(self.trace[-1][1]["rueckraum"], 1.0)

    def test_slanted_back_limited_and_dropped_when_it_gains_too_little(self):
        values = {"front": 12.0, "left": 4.0, "right": 9.0}                 # braucht 2,5 cm seitlich = 18 cm schräg
        robot = self.make(8.0)
        robot._center_before_turn(values)
        self.assertEqual([m[0] for m in self.moves], ["rotate", "drive", "rotate", "drive"])      # zuletzt wieder vor
        self.assertEqual(self.moves[1], ("drive", -8.0))                    # statt 12
        robot = self.make(5.0)                                              # 5 cm · sin 8° = 0,7 cm < 1 cm: entfällt
        robot._center_before_turn(values)
        self.assertEqual(self.moves, [])

    def test_second_turn_of_a_u_turn_does_not_reverse_into_the_wall(self):
        """Nach der ersten Drehung unbekannt, vorne 8,9 cm: nur 2,1 cm Platz hinten, die Seitenkorrektur entfällt."""
        robot = self.make(None)
        robot._center_before_turn({"front": 8.9, "left": 4.5, "right": None})
        self.assertEqual(self.moves, [])

