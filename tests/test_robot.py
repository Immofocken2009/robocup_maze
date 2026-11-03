"""Tests für hardware/robot.py mit nachgebauter Hardware (nichts bewegt sich).

Der nachgebaute Roboter fährt auf einer Geraden: x = Weg der Robotermitte in cm ab der Mitte
der Startkachel. Die Zeit läuft im Nachbau SPEEDUP-mal so schnell, damit die Tests kurz bleiben.
"""
import math
import time
import unittest

import config
from hardware.color_sensor import classify
from hardware.robot import HeadingEstimate, LackOfProgress, RealRobot, RunStopped, Steering, stop_distance
from navigation.maze_map import TileType
from navigation.robot_interface import MoveStatus

SPEEDUP = 4.0
TILE = config.TILE_SIZE_CM
FACE = (TILE - config.WALL_THICKNESS_CM) / 2   # Wandfläche von der Kachelmitte


class Body:
    """Gerade Strecke mit einer Wand vorne (Abstand ihrer Fläche von der Mitte der Startkachel)."""

    def __init__(self, wall_cm: float | None, floors: list[TileType] | None = None):
        self.wall_cm = wall_cm
        self.floors = floors or [TileType.NORMAL] * 10
        self.x = 0.0
        self.furthest = 0.0

    def max_x(self) -> float:
        """Weiter kommt der Roboter nicht (Vorderkante an der Wand)."""
        return float("inf") if self.wall_cm is None else self.wall_cm - config.ROBOT_LENGTH_CM / 2


class FakeMotors:
    def __init__(self, body: Body):
        self.body = body
        self.pos = [0.0, 0.0]
        self.dps = [0.0, 0.0]
        self.t = time.monotonic()

    def _advance(self):
        now = time.monotonic()
        dt = (now - self.t) * SPEEDUP
        self.t = now
        step = [d * dt for d in self.dps]
        fwd = (step[0] + step[1]) / 2 / config.MOTOR_DEG_PER_CM
        if fwd > 0 and self.body.x + fwd > self.body.max_x():     # an der Wand: Ketten stehen
            step = [0.0, 0.0]
            fwd = 0.0
        self.pos = [p + s for p, s in zip(self.pos, step)]
        self.body.x += fwd
        self.body.furthest = max(self.body.furthest, self.body.x)

    def set_dps(self, left, right):
        self._advance()
        self.dps = [left, right]

    def positions(self):
        self._advance()
        return tuple(self.pos)

    def stop(self):
        self.set_dps(0, 0)

    float = stop


class FakeUltrasonic:
    def __init__(self, body: Body):
        self.body = body
        self.last = {"front": None, "left": None, "right": None}

    def ready(self):
        return True

    def measure(self, side):
        if side == "front":
            cm = None
            if self.body.wall_cm is not None:
                cm = self.body.wall_cm - self.body.x - (config.ROBOT_LENGTH_CM / 2 - config.US_FRONT_INSET_CM)
                cm = cm if cm <= config.US_MAX_DISTANCE_CM else None
        else:
            cm = config.SIDE_CENTER_CM
        self.last[side] = cm
        return cm

    def median(self, side, n=3):
        return self.measure(side)

    def read_all(self, n=1):
        return {side: self.measure(side) for side in self.last}


class FakeColor:
    integration_s = 0.0

    def __init__(self, body: Body):
        self.body = body
        self.last_raw = None
        self.last_color = None

    def read_tile(self):
        sensor = self.body.x + config.ROBOT_LENGTH_CM / 2 - config.COLOR_INSET_CM
        return self.body.floors[int((sensor + TILE / 2) // TILE)]

    def read_floor(self):
        return self.read_tile()

    def read_color(self):
        return None


def make_robot(body: Body) -> RealRobot:
    return RealRobot(FakeMotors(body), FakeUltrasonic(body), FakeColor(body))


class ForwardTest(unittest.TestCase):
    def test_stops_by_front_distance(self):
        body = Body(wall_cm=FACE + 2 * TILE)            # Wand am Ende der übernächsten Kachel
        result = make_robot(body).forward(set())
        self.assertIs(result.status, MoveStatus.OK)
        self.assertAlmostEqual(body.x, TILE, delta=1.5)

    def test_stops_by_encoder_without_wall(self):
        body = Body(wall_cm=None)
        make_robot(body).forward(set())
        self.assertAlmostEqual(body.x, TILE, delta=1.5)

    def test_front_distance_corrects_position(self):
        """Steht der Roboter 4 cm zu weit hinten, fährt er nach Abstand weiter als 30 cm."""
        body = Body(wall_cm=FACE + TILE + 4)
        make_robot(body).forward(set())
        self.assertAlmostEqual(body.x, TILE + 4, delta=1.5)

    def test_wall_ahead_blocks(self):
        body = Body(wall_cm=FACE)
        result = make_robot(body).forward(set())
        self.assertIs(result.status, MoveStatus.BLOCKED)
        self.assertEqual(body.x, 0.0)

    def test_black_tile_avoided_before_half(self):
        body = Body(wall_cm=None, floors=[TileType.NORMAL, TileType.BLACK, TileType.NORMAL])
        result = make_robot(body).forward({TileType.BLACK})
        self.assertIs(result.status, MoveStatus.AVOIDED)
        self.assertIs(result.floor, TileType.BLACK)
        self.assertLess(body.furthest, TILE / 2)     # nie mehr als halb auf der schwarzen Kachel
        self.assertAlmostEqual(body.x, 0.0, delta=1.5)

    def test_reports_floor_of_new_tile(self):
        body = Body(wall_cm=None, floors=[TileType.NORMAL, TileType.BLUE, TileType.NORMAL])
        result = make_robot(body).forward({TileType.BLACK})
        self.assertIs(result.floor, TileType.BLUE)


class TurnAndButtonsTest(unittest.TestCase):
    def test_turn_uses_encoder_target(self):
        body = Body(wall_cm=None)
        robot = make_robot(body)
        robot.turn_right()
        left, right = robot.motors.positions()
        self.assertAlmostEqual(left, config.MOTOR_DEG_PER_90, delta=15)
        self.assertAlmostEqual(right, -config.MOTOR_DEG_PER_90, delta=15)

    def test_interrupts(self):
        robot = make_robot(Body(wall_cm=None))
        robot.interrupt("lop")
        with self.assertRaises(LackOfProgress):
            robot.wait(5)
        robot.interrupt("stop")
        with self.assertRaises(RunStopped):
            robot.forward(set())


class HelperTest(unittest.TestCase):
    def test_stop_distance(self):
        c = config.FRONT_CENTER_CM
        self.assertEqual(stop_distance(c + TILE, 0), TILE)              # Wand hinter der nächsten Kachel
        self.assertEqual(stop_distance(c + 2 * TILE + 3, 0), TILE + 3)  # eine Kachel weiter, 3 cm zu weit hinten
        self.assertEqual(stop_distance(c + 2 * TILE - 10, 10), TILE)    # nach 10 cm Fahrt gemessen
        # Echo springt auf eine weiter entfernte Wand: Haltepunkt bleibt bei einer Kachel
        self.assertAlmostEqual(stop_distance(c + 3 * TILE - 13, 13), TILE)
        self.assertIsNone(stop_distance(None, 0))
        self.assertIsNone(stop_distance(config.FRONT_TRUST_CM + 1, 0))

    def test_heading_estimate(self):
        """Schräg (4° nach rechts) durch den Gang, dabei lenkt er sich laut Encoder um 3° zurück."""
        h = HeadingEstimate()
        deg_per_degree = config.MOTOR_DEG_PER_90 / 90 * 2     # Encoderunterschied pro Grad Drehung
        offset = 0.5
        for i in range(13):
            s = 2.0 * i
            angle = 4.0 - 3.0 * s / 24                      # wahrer Schrägstand
            h.track(s, (-deg_per_degree * (4.0 - angle), 0.0))
            h.sample(offset)
            offset += math.sin(math.radians(angle)) * 2.0
        self.assertAlmostEqual(h.degrees(), 1.0, delta=0.1)

    def test_heading_needs_enough_samples(self):
        h = HeadingEstimate()
        for s in range(config.HEADING_MIN_SAMPLES - 1):
            h.track(5.0 * s, (0.0, 0.0))
            h.sample(0.3 * s)
        self.assertIsNone(h.degrees())

    def test_classify_nearest_color(self):
        cal = {"white": (900, 850, 700, 2500), "black": (60, 60, 50, 170),
               "blue": (150, 300, 600, 1000), "silver": (1300, 1250, 1050, 3700),
               "red": (900, 250, 200, 1300)}
        self.assertEqual(classify((880, 830, 690, 2450), cal), "white")
        self.assertEqual(classify((50, 55, 45, 150), cal), "black")
        self.assertEqual(classify((140, 280, 620, 1050), cal), "blue")
        self.assertEqual(classify((950, 260, 190, 1350), cal), "red")


def drive(steering: Steering, left: list, right: list, step_cm: float = 2.0) -> list[float]:
    """Seitenmessungen abwechselnd links/rechts einspeisen, je step_cm Fahrt dazwischen."""
    corrections, travelled = [], 0.0
    for l, r in zip(left, right):
        for side, cm in (("left", l), ("right", r)):
            travelled += step_cm
            corrections.append(steering.side(side, cm, travelled, (0.0, 0.0)))
    return corrections


C = config.SIDE_CENTER_CM          # Abstand Sensor–Wand in der Kachelmitte
FAR = C + TILE                      # Wand eine Kachel weiter


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

if __name__ == "__main__":
    unittest.main()
