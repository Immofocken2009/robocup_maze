"""Tests der Lenkung beim Geradeausfahren (Steering, Wandkontakt, Festfahren) und von stop_distance/HeadingEstimate."""
import unittest
from unittest import mock

import config
from hardware.robot import RealRobot, Steering, fit_slope
from navigation.robot_interface import MoveStatus
from tests.fakes import TILE, Body, FakeMotors, FakeUltrasonic, FakeColor, drive, C, FAR


class SteeringTest(unittest.TestCase):
    def test_equal_sides_is_center(self):
        s = Steering()
        corr = drive(s, [C] * 5, [C] * 5)
        self.assertEqual(s.mode, "both")
        self.assertEqual(corr[-1], 0.0)

    def test_steers_towards_equal_sides(self):
        # näher an der rechten Wand -> nach links lenken (positiv), und umgekehrt
        self.assertGreater(drive(Steering(), [C + 2] * 4, [C - 2] * 4)[-1], 0)
        self.assertLess(drive(Steering(), [C - 2] * 4, [C + 2] * 4)[-1], 0)

    def test_lateral_drift_keeps_both_walls(self):
        """Seitliches Wandern ändert links + rechts nicht: kein Fehlalarm des Wächters."""
        s = Steering()
        corr = drive(s, [C, C, C - .5, C - 1, C - 1.5, C - 2], [C, C, C + .5, C + 1, C + 1.5, C + 2])
        self.assertEqual(s.mode, "both")
        self.assertLess(corr[-1], 0)      # zu weit links -> nach rechts lenken

    def test_wall_breaking_away_does_not_swerve(self):
        """Links endet die Wand (Wert kriecht erst hoch, dann Sprung), rechts geht weiter."""
        s = Steering()
        corr = drive(s, [C, C, C, C + .4, C + 1.6, C + 4, FAR, FAR, FAR], [C] * 9)
        self.assertEqual(s.mode, "right")
        self.assertLess(max(corr), 25)            # nur kurz leicht nach links, kein Ausscheren
        self.assertAlmostEqual(corr[-1], 0.0, delta=3)   # praktisch gerade (< 0,15 cm daneben)

    def test_single_far_wall_breaking_away(self):
        """Nur links eine Wand, eine Kachel weiter, sie bricht weg: kriecht langsam hoch."""
        s = Steering()
        corr = drive(s, [FAR] * 4 + [FAR + .3, FAR + 1.2, FAR + 2.5, None, None], [None] * 9)
        self.assertEqual(s.mode, "encoder")
        self.assertLess(max(corr), 30)

    def test_single_near_wall_breaking_away(self):
        s = Steering()
        corr = drive(s, [C] * 4 + [C + .4, C + 1.6, C + 4, None, None], [None] * 9)
        self.assertLess(max(corr), 15)

    def test_returning_wall_needs_two_readings(self):
        s = Steering()
        drive(s, [C, C, FAR, FAR], [C, C, C, C])
        self.assertEqual(s.mode, "right")
        s.side("left", C + .2, 20, (0, 0))
        self.assertEqual(s.mode, "right")          # eine Messung reicht nicht
        s.side("left", C, 22, (0, 0))
        self.assertEqual(s.mode, "both")

    def test_single_wall_keeps_half_width(self):
        s = Steering()
        drive(s, [C + 1] * 3, [C - 1] * 3)         # Gang 2·C breit
        corr = drive(s, [None] * 4, [C] * 4)       # links weg: rechts auf die halbe Breite halten
        self.assertEqual(s.mode, "right")
        self.assertAlmostEqual(corr[-1], 0.0, delta=1)

    def test_wide_room_uses_nearer_wall(self):
        """Foyer: rechts eine Kachel weiter offen. 'Gleich' wäre die Kachelgrenze, er bleibt auf seiner Kachel."""
        s = Steering()
        corr = drive(s, [C] * 4, [FAR] * 4)
        self.assertEqual(s.mode, "left")
        self.assertAlmostEqual(corr[-1], 0.0, delta=1)

    def test_only_far_wall_counts_whole_tiles(self):
        s = Steering()
        corr = drive(s, [FAR] * 4, [None] * 4)
        self.assertEqual(s.mode, "left")
        self.assertAlmostEqual(corr[-1], 0.0, delta=1)

    def test_no_wall_keeps_tracks_equal(self):
        s = Steering()
        drive(s, [None, None], [None, None])
        self.assertEqual(s.mode, "encoder")
        # linke Kette ist weiter gefahren -> nach links lenken
        self.assertGreater(s.hold(12, (110, 100)), 0)

    def test_correction_changes_gradually(self):
        s = Steering()
        corr = drive(s, [C + 6] * 3, [C - 6] * 3)  # 6 cm daneben: voll wären 120 dps
        steps = [b - a for a, b in zip([0.0] + corr, corr)]
        self.assertLessEqual(max(steps), config.STEER_SLEW_DPS_PER_CM * 2 + 1e-9)

    def test_no_d_with_far_opposite_wall(self):
        """Nur eine Wand nah, die Gegenwand fern (über Raster): D = 0, auch wenn sich die Abweichung ändert."""
        for near, far in (("left", "right"), ("right", "left")):
            s = Steering()
            sides = {near: [C + .5 * i for i in range(8)], far: [FAR] * 8}
            drive(s, sides["left"], sides["right"])
            self.assertEqual(s.mode, near)
            self.assertEqual(s.slope, 0.0)
            self.assertEqual(s.values()["d"], 0.0)

    def test_d_in_corridor(self):
        """Zwei nahe Wände (B <= CORRIDOR_MAX_CM): D wie bisher."""
        s = Steering()
        drive(s, [C + .5 * i for i in range(8)], [C - .5 * i for i in range(8)])
        self.assertEqual(s.mode, "both")
        self.assertLessEqual(s.width, config.CORRIDOR_MAX_CM)
        self.assertNotEqual(s.slope, 0.0)

    def test_grid_offset_limited(self):
        """Ferne Wand mit 7 cm Rasterabweichung: auf STEER_GRID_MAX_CM begrenzt, nahe Wand nicht."""
        s = Steering()
        drive(s, [FAR + 7] * 4, [None] * 4)
        self.assertEqual(s.mode, "left")
        self.assertEqual(s.offset, config.STEER_GRID_MAX_CM)
        s = Steering()
        drive(s, [None] * 4, [FAR + 7] * 4)
        self.assertEqual(s.offset, -config.STEER_GRID_MAX_CM)
        s = Steering()
        drive(s, [C + 7] * 4, [None] * 4)      # nahe Wand: unbegrenzt
        self.assertEqual(s.offset, 7)

    def test_contact_trusted_at_once_without_slew_limit(self):
        """Seitenwert 1,9 cm (Kante an der Wand) bei bis dahin ferner Wand: gilt sofort, Lenkung weg von der Wand,
        ohne die Begrenzung der Lenkänderung."""
        s = Steering()
        drive(s, [FAR] * 3, [C] * 3)
        before = s.corr
        corr = s.side("left", 1.9, 13.0, (0.0, 0.0))
        self.assertTrue(s.trusted["left"])
        self.assertEqual(s.contact, "left")
        self.assertEqual(s.values()["kontakt"], "left")
        self.assertLess(corr, 0)                                   # zu weit links -> nach rechts lenken
        self.assertGreater(before - corr, config.STEER_SLEW_DPS_PER_CM * 1 + 1)    # mehr als die Begrenzung erlaubt
        self.assertAlmostEqual(corr, -min(config.STEER_MAX_DPS, abs(s.raw)), places=6)
        s.side("right", C, 15.0, (0.0, 0.0))
        self.assertIsNone(s.contact)
        self.assertEqual(s.corridor, config.CORRIDOR_CM)           # schleifende Seite verfälscht B nicht

    def test_contact_right_steers_left(self):
        s = Steering()
        drive(s, [C] * 3, [FAR] * 3)
        self.assertGreater(s.side("right", 1.9, 13.0, (0.0, 0.0)), 0)


class WallContactTest(unittest.TestCase):
    """Schleifen an einer Seitenwand: Seitenwert dauerhaft 1,9 cm, die Encoder laufen normal."""

    class Scrape(FakeUltrasonic):
        def __init__(self, body, side, until_x=float("inf")):
            super().__init__(body)
            self.side, self.until_x = side, until_x

        def measure(self, side):
            cm = super().measure(side)
            if side == self.side and self.body.x < self.until_x and cm is not None:
                cm = 1.9
                self.last[side] = cm
            return cm

    def run_forward(self, side, until_x):
        body = Body(wall_cm=None)
        robot = RealRobot(FakeMotors(body), self.Scrape(body, side, until_x), FakeColor(body))
        seen = []
        robot.trace = lambda what, **v: seen.append((what, v))
        return body, robot, seen, robot.forward(set())

    def test_backs_off_turns_away_and_continues(self):
        # Bei x < 8 schleift er rechts. Er hängt ab > 10 cm Strecke. Der Schleifwert endet nach dem Zurücksetzen.
        body = Body(wall_cm=None)
        us = self.Scrape(body, "right")
        robot = RealRobot(FakeMotors(body), us, FakeColor(body))
        seen = []
        robot.trace = lambda what, **v: seen.append((what, v))
        original = robot._rotate

        def rotate(deg):
            us.until_x = -1e9                     # nach dem Wegdrehen schleift nichts mehr
            return original(deg)
        robot._rotate = rotate
        result = robot.forward(set())
        self.assertIs(result.status, MoveStatus.OK)
        hits = [v for what, v in seen if what == "schleift"]
        self.assertEqual(len(hits), 1)
        self.assertEqual(hits[0]["seite"], "right")
        self.assertGreater(hits[0]["gefahren"], config.STEER_CONTACT_DIST_CM)
        self.assertFalse(any(what == "blockiert" for what, _ in seen))
        self.assertAlmostEqual(body.x, TILE, delta=2.5)           # Ziel bleibt die Kachelmitte (Strecke berücksichtigt)
        turns = [v["winkel"] for what, v in seen if what == "drehen"]
        self.assertTrue(turns and all(w == -config.CONTACT_ANGLE_DEG for w in turns))   # rechts schleift: nach links

    def test_left_turns_right(self):
        body = Body(wall_cm=None)
        us = self.Scrape(body, "left")
        robot = RealRobot(FakeMotors(body), us, FakeColor(body))
        seen = []
        robot.trace = lambda what, **v: seen.append((what, v))
        original = robot._rotate
        robot._rotate = lambda deg: (setattr(us, "until_x", -1e9), original(deg))[1]
        with mock.patch.object(config, "STEER_CONTACT_DIST_CM", 5.0):
            robot.forward(set())
        self.assertTrue(any(v["winkel"] == config.CONTACT_ANGLE_DEG for what, v in seen if what == "drehen"))

    def test_gives_up_after_max_tries(self):
        body, robot, seen, result = self.run_forward("right", float("inf"))
        self.assertIs(result.status, MoveStatus.BLOCKED)
        self.assertEqual(len([1 for what, _ in seen if what == "schleift"]), config.CONTACT_MAX_TRIES)
        self.assertTrue(any(what == "blockiert" for what, _ in seen))
        self.assertLess(body.x, 2.5)                              # zurück am Anfang


if __name__ == "__main__":
    unittest.main()


class StickingShareTest(unittest.TestCase):
    """`sticking`: Anteil der Messungen im Kontakt statt ununterbrochen (Ausreißer setzt nichts zurück)."""

    def feed(self, values, step=1.25, side="left"):
        s, t, hit = Steering(), 0.0, None
        for cm in values:
            s.side(side, cm, t, (0.0, 0.0))
            hit = s.sticking(t)
            t += step
        return s, hit

    def test_single_outlier_does_not_reset(self):
        s, hit = self.feed([1.9] * 4 + [2.9] + [1.9] * 4, step=1.3)                  # 9 Messungen über 10 cm
        self.assertEqual(hit, "left")

    def test_short_touch_then_free_does_not_trigger(self):
        s, hit = self.feed([1.9, 1.9] + [C] * 5, step=2.5)                  # 2 Kontakte, dann 5 cm frei
        self.assertIsNone(hit)
        s, hit = self.feed([1.9, 1.9, 1.9] + [C] * 6, step=2.5)
        self.assertIsNone(hit)

    def test_too_few_measurements_or_too_short(self):
        self.assertIsNone(self.feed([1.9, 1.9], step=6.0)[1])               # nur 2 Messungen
        self.assertIsNone(self.feed([1.9] * 4, step=1.0)[1])                # erst 3 cm beobachtet

    def test_right_side(self):
        self.assertEqual(self.feed([1.9] * 10, side="right", step=1.3)[1], "right")


class FrontSlipTest(unittest.TestCase):
    """Festfahren vorne: Abstand sinkt viel weniger als die Encoderstrecke."""

    class Slippy(FakeUltrasonic):
        """Abstand vorne = 120 cm − factor · Weg (Wand weit weg): factor < 1 = die Ketten rutschen."""
        def __init__(self, body, factor):
            super().__init__(body)
            self.factor = factor

        def measure(self, side):
            cm = super().measure(side)
            if side == "front":
                cm = 120.0 - self.factor * self.body.x
                self.last[side] = cm
            return cm

    def run_forward(self, factor, free_after_back=False):
        body = Body(wall_cm=None)
        us = self.Slippy(body, factor)
        robot = RealRobot(FakeMotors(body), us, FakeColor(body))
        seen, actions = [], []
        robot.trace = lambda what, **v: seen.append((what, v))
        original = robot.set_action
        robot.set_action = lambda a, d="": (actions.append(a), original(a, d))[1]
        if free_after_back:                       # nach dem Zurücksetzen packen die Ketten wieder
            drive = robot.drive_cm
            robot.drive_cm = lambda cm: (setattr(us, "factor", 1.0) if cm < 0 else None, drive(cm))[1]
        return body, robot, seen, actions

    def test_slipping_backs_off_and_continues(self):
        config_front = mock.patch.object(config, "FRONT_TRUST_CM", 200.0)
        with config_front:
            body, robot, seen, actions = self.run_forward(0.2, free_after_back=True)
            result = robot.forward(set())
        self.assertIn("Rutscht vorne", actions)
        hits = [v for what, v in seen if what == "rutscht"]
        self.assertEqual(len(hits), 1)
        self.assertLess(hits[0]["gemessen"], config.FRONT_STUCK_RATIO * hits[0]["erwartet"])
        self.assertFalse(any(what == "schleift" for what, _ in seen))
        self.assertFalse(any(v["winkel"] for what, v in seen if what == "drehen"))      # kein Wegdrehen
        self.assertIs(result.status, MoveStatus.OK)

    def test_normal_drive_with_slip_factor_does_not_trigger(self):
        with mock.patch.object(config, "FRONT_TRUST_CM", 200.0):
            body, robot, seen, actions = self.run_forward(0.87)
            robot.forward(set())
        self.assertNotIn("Rutscht vorne", actions)
        self.assertFalse(any(what == "rutscht" for what, _ in seen))

    def test_gives_up_after_max_tries(self):
        with mock.patch.object(config, "FRONT_TRUST_CM", 200.0):
            body, robot, seen, actions = self.run_forward(0.2)
            result = robot.forward(set())
        self.assertIs(result.status, MoveStatus.BLOCKED)
        self.assertEqual(actions.count("Rutscht vorne"), config.CONTACT_MAX_TRIES)
        self.assertTrue(any(what == "blockiert" for what, _ in seen))


class FitSlopeTest(unittest.TestCase):
    def test_exact_line(self):
        self.assertAlmostEqual(fit_slope([(0, 1.0), (1, 1.2), (2, 1.4), (3, 1.6)]), 0.2)
        self.assertEqual(fit_slope([(0, 1.0)]), 0.0)
        self.assertEqual(fit_slope([(2, 1.0), (2, 3.0)]), 0.0)

    def test_fit_is_more_accurate_than_endpoint_difference(self):
        import random
        rng = random.Random(5)
        err_fit = err_end = 0.0
        for _ in range(300):
            pts = [(t / 2, 0.1 * t / 2 + rng.gauss(0, 0.4)) for t in range(9)]
            err_fit += (fit_slope(pts) - 0.1) ** 2
            err_end += ((pts[-1][1] - pts[0][1]) / 4.0 - 0.1) ** 2
        self.assertLess(err_fit, 0.7 * err_end)

    def test_steering_needs_three_samples(self):
        s = Steering()
        drive(s, [C + 1.0, C + 2.0], [C - 1.0, C - 2.0], step_cm=1.0)       # nur 4 Proben über kurze Strecke, aber Fenster gefüllt
        s2 = Steering()
        s2.mode = "both"
        s2.samples.append((0.0, 0.0))
        s2.last = {"left": C, "right": C}
        s2.trusted = {"left": True, "right": True}
        s2.side("left", C + 3.0, 3.0, (0.0, 0.0))                           # nur 2 Proben, Spanne 3 >= 2: früher Steigung, jetzt 0
        self.assertEqual(s2.slope, 0.0)
