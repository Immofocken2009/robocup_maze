"""Tests für hardware/robot.py mit nachgebauter Hardware (nichts bewegt sich).

Der nachgebaute Roboter fährt auf einer Geraden: x = Weg der Robotermitte in cm ab der Mitte
der Startkachel. Die Zeit läuft im Nachbau SPEEDUP-mal so schnell, damit die Tests kurz bleiben.
"""
import time
import unittest

import config
from hardware.color_sensor import classify
from hardware.robot import LackOfProgress, RealRobot, RunStopped, Steering, front_target
from navigation.maze_map import TileType
from navigation.robot_interface import MoveStatus

SPEEDUP = 4.0
TILE = config.TILE_SIZE_CM


class Body:
    """Gerade Strecke mit einer Wand vorne (Abstand von der Mitte der Startkachel)."""

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

    def set_power_limit(self, _percent):
        pass


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
                cm = self.body.wall_cm - self.body.x - (TILE / 2 - config.FRONT_CENTER_CM)
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
        body = Body(wall_cm=2.5 * TILE)            # Wand am Ende der übernächsten Kachel
        result = make_robot(body).forward(set())
        self.assertIs(result.status, MoveStatus.OK)
        self.assertAlmostEqual(body.x, TILE, delta=1.5)

    def test_stops_by_encoder_without_wall(self):
        body = Body(wall_cm=None)
        make_robot(body).forward(set())
        self.assertAlmostEqual(body.x, TILE, delta=1.5)

    def test_front_distance_corrects_position(self):
        """Steht der Roboter 4 cm zu weit hinten, fährt er nach Abstand weiter als 30 cm."""
        body = Body(wall_cm=1.5 * TILE + 4)
        make_robot(body).forward(set())
        self.assertAlmostEqual(body.x, TILE + 4, delta=1.5)

    def test_wall_ahead_blocks(self):
        body = Body(wall_cm=TILE / 2)
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


class AlignTest(unittest.TestCase):
    def test_push_front_then_back_to_center(self):
        body = Body(wall_cm=TILE / 2)
        make_robot(body).align(front_wall=True, back_wall=False)
        self.assertAlmostEqual(body.x, 0.0, delta=1.5)

    def test_no_wall_found_returns(self):
        """Karte sagt Wand vorne, Sensor sieht keine: nicht andrücken."""
        body = Body(wall_cm=None)
        make_robot(body).align(front_wall=True, back_wall=False)
        self.assertEqual(body.x, 0.0)


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
    def test_front_target(self):
        c = config.FRONT_CENTER_CM
        self.assertEqual(front_target(c + TILE), c)
        self.assertEqual(front_target(c + 2 * TILE + 3), c + TILE)
        self.assertIsNone(front_target(None))
        self.assertIsNone(front_target(config.FRONT_TRUST_CM + 1))

    def test_steering_signs(self):
        # näher an der rechten Wand -> nach links lenken (positiv)
        self.assertGreater(Steering().update(12, 6, 0, (0, 0)), 0)
        self.assertLess(Steering().update(6, 12, 0, (0, 0)), 0)
        # ohne Wand: linke Kette ist weiter gefahren -> nach links lenken
        s = Steering()
        s.update(None, None, 0, (0, 0))
        self.assertGreater(s.update(None, None, 5, (110, 100)), 0)

    def test_classify_nearest_color(self):
        cal = {"white": (900, 850, 700, 2500), "black": (60, 60, 50, 170),
               "blue": (150, 300, 600, 1000), "silver": (1300, 1250, 1050, 3700),
               "red": (900, 250, 200, 1300)}
        self.assertEqual(classify((880, 830, 690, 2450), cal), "white")
        self.assertEqual(classify((50, 55, 45, 150), cal), "black")
        self.assertEqual(classify((140, 280, 620, 1050), cal), "blue")
        self.assertEqual(classify((950, 260, 190, 1350), cal), "red")


if __name__ == "__main__":
    unittest.main()
