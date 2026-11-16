"""Nachgebaute Hardware für die Tests des Fahrcodes (Motoren, Ultraschall, Farbsensor, BrickPi3) und
kleine gemeinsame Hilfen. Kein Testmodul; die Tests importieren von hier (`from tests.fakes import ...`)."""
import time

import config
from hardware.robot import RealRobot, Steering
from navigation.maze_map import TileType

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


class BlindUltrasonic(FakeUltrasonic):
    """Alle drei Sensoren ohne Echo (z.B. Versorgung weg)."""
    def measure(self, side):
        self.last[side] = None
        return None


class FakeBrickPi3:
    """BrickPi3 ohne Hardware: Port-Bits wie im Original, die ersten `lose` Schreibbefehle gehen verloren."""
    PORT_A, PORT_B, PORT_C, PORT_D = 1, 2, 4, 8
    MOTOR_FLOAT = -128
    lose = 0

    def __init__(self):
        self.power = {p: 0 for p in (1, 2, 4, 8)}
        self.sent = []

    def set_motor_power(self, ports: int, power: int):
        self.sent.append((ports, power))
        if FakeBrickPi3.lose > 0:
            FakeBrickPi3.lose -= 1
            return
        for p in self.power:
            if ports & p:
                self.power[p] = power

    def get_motor_status(self, port: int):
        return [0, self.power[port], 0, 0]


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

