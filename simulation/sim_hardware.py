"""Fahrphysik für die Simulation: der echte Fahrcode (hardware/robot.py) fährt gegen nachgebildete Hardware.

SimBody          - der Roboter im Labyrinth: Position (cm), Blickrichtung, zwei Ketten mit Wandkontakt,
                   dazu der Schiedsrichter (besuchte Kacheln, Lack of Progress)
SimClock         - Sim-Zeit: sleep() rechnet die Physik weiter und bremst auf das eingestellte Tempo
SimMotors, SimUltrasonic, SimColorSensor - dieselben Methoden wie hardware/motors.py, ultrasonic.py,
                   color_sensor.py
SimHardware      - alles zusammen für ein Labyrinth
SimDrivingRobot  - der echte RealRobot, der vor jeder Aktion auf Start / Pause / Schritt wartet

Koordinaten in cm: x nach Osten, y nach Norden, Kachel (i, j) liegt bei [30·i, 30·i+30] × [30·j, 30·j+30].
Blickrichtung in Grad im Uhrzeigersinn, 0 = Norden (wie Dir: N = 0, E = 1, ...).
"""
import math
import random
from collections import Counter, defaultdict
from collections.abc import Callable

import config
from hardware.color_sensor import COLOR_TO_TILE, ColorSensor
from hardware.robot import RealRobot
from hardware.ultrasonic import SIDES, Ultrasonic
from navigation.maze_map import Dir, Pos, TileType
from simulation.sim_robot import SimController, SimWorld

TILE = config.TILE_SIZE_CM
WALL_HALF = config.WALL_THICKNESS_CM / 2
LENGTH, WIDTH = config.ROBOT_LENGTH_CM, config.ROBOT_WIDTH_CM
PHYSICS_DT = 0.01           # Sim-Schritt in s (so lang wie ein Takt der Regelschleifen)
MOTOR_TAU_S = 0.05          # so schnell folgt die Drehzahl dem Befehl (Regelung im BrickPi3), gemessen 2026-11-10: ca. 0,055 s
MOTOR_TAU_FLOAT_S = 0.012   # stromlos (Befehl 0): Ausrollen ca. 2-4 Encodergrad bei 183-244 dps (gemessen 2026-11-10)
# Kettenabstand so gewählt, dass MOTOR_DEG_PER_90 genau 90° ergibt: die Simulation fährt "wie eingemessen"
TRACK_CM = 4 * config.MOTOR_DEG_PER_90 / config.MOTOR_DEG_PER_CM / math.pi
US_CONE_DEG = 15.0          # halber Öffnungswinkel des Schallkegels
US_NOISE_CM = 0.4           # Messrauschen beim Fahren (Standardabweichung, gemessen 2026-11-10: 0,42), wenn eingeschaltet
US_NOISE_STILL_CM = 0.03    # im Stand (gemessen 2026-11-10: 0,01-0,03)
US_DROPOUT = 0.003          # Anteil Messungen ohne Echo, wenn Rauschen eingeschaltet (gemessen 2026-11-10: 0-0,7 %)
# Schlupf: streut um den eingemessenen Wert (MOTOR_DEG_PER_CM und MOTOR_DEG_PER_90 enthalten den mittleren Schlupf)
SLIP_MAX = 0.03             # die Ketten rutschen unterschiedlich, bis 3 % Unterschied
DRIVE_SLIP = 0.12           # beim Geradeausfahren (normales Tempo) legt er 12 % weniger zurück, als die Encoder zählen
                            # (gemessen 2026-11-10: Abstand vorne sinkt um 0,87 der Encoderstrecke; MOTOR_DEG_PER_CM
                            # wurde mit halbem Tempo eingemessen). Drehen ist davon nicht betroffen (eingemessen).
DRIFT_BIAS = 0.043          # rechte Kette macht immer so viel weniger Weg: 10° nach rechts auf 98,5 cm bei gleichen Encodern
                            # (gemessen 2026-11-04); die Lenkung hält im Mittel ca. 4-6° je Kachel dagegen (Logs 2026-11-10)
CORNER_SOFT_CM = 0.03       # Drehen auf der Stelle: so tief muss eine Ecke in die Wand drücken, bevor die Ketten durchrutschen
                            # (er dreht dann nicht mehr). Keine Messung: so abgestimmt, dass er in der Sackgasse bei 2,7 cm
                            # Versatz und 4 cm Abstand vorne wie am echten Roboter hängen bleibt, bei 1,5 cm nicht.
TURN_SKID = 0.04            # Drehungen streuen bis ±4 % um den eingemessenen Wert
TILE_COLORS = {tile: name for name, tile in COLOR_TO_TILE.items()}
RAW = {"white": (900, 850, 700, 2500), "black": (60, 60, 50, 170), "blue": (150, 300, 600, 1000),
       "silver": (1300, 1250, 1050, 3700), "red": (900, 250, 200, 1300)}   # Rohwerte, zugleich Kalibrierung

Segment = tuple[float, float, float, float]


# --- Geometrie ------------------------------------------------------------------------

def _closest(px: float, py: float, seg: Segment) -> tuple[float, float]:
    ax, ay, bx, by = seg
    vx, vy = bx - ax, by - ay
    t = max(0.0, min(1.0, ((px - ax) * vx + (py - ay) * vy) / (vx * vx + vy * vy)))
    return ax + t * vx, ay + t * vy


def _dist(px: float, py: float, seg: Segment) -> float:
    qx, qy = _closest(px, py, seg)
    return math.hypot(px - qx, py - qy)


def _wall_segments(world: SimWorld) -> list[Segment]:
    """Alle Wände, aneinanderstoßende Stücke zu durchgehenden Wänden zusammengefasst
    (die Fugen sind keine Kanten, an denen ein Echo entsteht)."""
    lines: dict[tuple[str, float], list[tuple[float, float]]] = defaultdict(list)
    for i, j in world.tiles:
        x0, y0 = i * TILE, j * TILE
        edges = {Dir.N: ("h", y0 + TILE, x0), Dir.S: ("h", y0, x0), Dir.W: ("v", x0, y0), Dir.E: ("v", x0 + TILE, y0)}
        for d, (kind, at, start) in edges.items():
            if world.wall((i, j), d):
                lines[(kind, at)].append((start, start + TILE))
    segments = []
    for (kind, at), spans in lines.items():
        spans = sorted(set(spans))
        a, b = spans[0]
        for c, e in spans[1:] + [(math.inf, math.inf)]:
            if c <= b:
                b = max(b, e)
                continue
            segments.append((a, at, b, at) if kind == "h" else (at, a, at, b))
            a, b = c, e
    return sorted(segments)


def _near(world: SimWorld, segments: list[Segment]) -> dict[Pos, list[Segment]]:
    """Wände in der Umgebung jeder Kachel (3 × 3), damit die Kollisionsprüfung schnell bleibt."""
    near = {}
    for i, j in world.tiles:
        x0, x1, y0, y1 = (i - 1) * TILE - 1, (i + 2) * TILE + 1, (j - 1) * TILE - 1, (j + 2) * TILE + 1
        near[(i, j)] = [s for s in segments
                        if max(s[0], s[2]) >= x0 and min(s[0], s[2]) <= x1
                        and max(s[1], s[3]) >= y0 and min(s[1], s[3]) <= y1]
    return near


# Punkte auf dem Umriss des Roboters (quer, längs), für die Kollisionsprüfung
_OUTLINE = [(qx * WIDTH / 2, qy * LENGTH / 2)
            for qx, qy in ((-1, 1), (0, 1), (1, 1), (1, .5), (1, 0), (1, -.5), (1, -1), (0, -1),
                           (-1, -1), (-1, -.5), (-1, 0), (-1, .5))]


# --- Roboter im Labyrinth --------------------------------------------------------------

class SimBody:
    def __init__(self, world: SimWorld):
        self.world = world
        self.x = (world.start[0] + 0.5) * TILE
        self.y = (world.start[1] + 0.5) * TILE
        self.heading = 90.0 * int(world.start_dir)
        self.command = [0.0, 0.0]       # befohlene Drehzahl (dps), links / rechts, positiv = vorwärts
        self.speed = [0.0, 0.0]         # tatsächliche Drehzahl
        self.encoder = [0.0, 0.0]
        self.floating = True
        self.slip_on = False
        self._slip = [0.0, 0.0]
        self._turn_err = 0.0
        self.rng = random.Random()
        self.segments = _wall_segments(world)
        self.near = _near(world, self.segments)
        # Schiedsrichter
        self.time = 0.0
        self.tile: Pos = world.start
        self.entered_at = 0.0
        self.visits: Counter[Pos] = Counter({world.start: 1})
        self.lop = 0
        self.on_event: Callable[[str], None] = lambda text: None

    # Umrechnung Roboter -> Labyrinth
    def _axes(self, heading: float) -> tuple[float, float, float, float]:
        h = math.radians(heading)
        return math.sin(h), math.cos(h), math.cos(h), -math.sin(h)    # vorwärts (x, y), rechts (x, y)

    def point(self, lateral: float, ahead: float, x=None, y=None, heading=None) -> tuple[float, float]:
        fx, fy, rx, ry = self._axes(self.heading if heading is None else heading)
        x = self.x if x is None else x
        y = self.y if y is None else y
        return x + lateral * rx + ahead * fx, y + lateral * ry + ahead * fy

    def tile_at(self, x: float, y: float) -> Pos:
        return int(x // TILE), int(y // TILE)

    def floor_at(self, x: float, y: float) -> TileType:
        return self.world.tiles.get(self.tile_at(x, y), TileType.NORMAL)

    def _segments_near(self, x: float, y: float) -> list[Segment]:
        return self.near.get(self.tile_at(x, y), self.segments)

    # --- Sensoren --------------------------------------------------------------------

    def sensor_pose(self, side: str) -> tuple[float, float, float, float]:
        """Ultraschall: Position und Blickrichtung (Einheitsvektor)."""
        fx, fy, rx, ry = self._axes(self.heading)
        if side == "front":
            px, py = self.point(0, LENGTH / 2 - config.US_FRONT_INSET_CM)
            return px, py, fx, fy
        sign = -1 if side == "left" else 1
        px, py = self.point(sign * (WIDTH / 2 - config.US_SIDE_INSET_CM), config.US_SIDE_AHEAD_CM)
        return px, py, sign * rx, sign * ry

    def distance(self, side: str) -> float:
        """Erstes Echo im Schallkegel. Jede Wand antwortet höchstens mit ihrem nächsten Punkt:
        dem Lotfußpunkt (dort spiegelt sie den Schall zurück) oder dem zugewandten Wandende.
        Flach gestreifte Wände antworten nicht. Am Wandende kriecht der Wert beim Vorbeifahren
        langsam hoch, bis das Ende den Kegel verlässt."""
        px, py, dx, dy = self.sensor_pose(side)
        cos_half = math.cos(math.radians(US_CONE_DEG))
        best = math.inf
        for seg in self.segments:
            qx, qy = _closest(px, py, seg)
            wx, wy = qx - px, qy - py
            d = math.hypot(wx, wy)
            if d > 1e-9 and (wx * dx + wy * dy) / d >= cos_half:
                best = min(best, d)
        return best - WALL_HALF

    def color_point(self) -> tuple[float, float]:
        return self.point(0, LENGTH / 2 - config.COLOR_INSET_CM)

    # --- Physik ----------------------------------------------------------------------

    def step(self, dt: float):
        self.time += dt
        for i in (0, 1):
            tau = MOTOR_TAU_S if self.command[i] else MOTOR_TAU_FLOAT_S
            self.speed[i] += (self.command[i] - self.speed[i]) * min(1.0, dt / tau)
        if abs(self.speed[0]) < 1e-3 and abs(self.speed[1]) < 1e-3:
            self.speed = [0.0, 0.0]
            return
        if self.slip_on:
            self._slip = [min(SLIP_MAX, max(0.0, s + self.rng.gauss(0, 0.002))) for s in self._slip]
            self._turn_err = min(TURN_SKID, max(-TURN_SKID, self._turn_err + self.rng.gauss(0, 0.003)))

        ground = []
        for i in (0, 1):
            lost = (self._slip[i] + (DRIFT_BIAS if i == 1 else 0.0)) if self.slip_on else 0.0
            v = self.speed[i] / config.MOTOR_DEG_PER_CM * (1 - lost)
            if v and self._track_blocked(i, 1 if v > 0 else -1):
                v = 0.0
                self.speed[i] = 0.0         # Kette steht an der Wand, Motor blockiert
            ground.append(v)

        turn = (ground[0] - ground[1]) / TRACK_CM          # rad/s, im Uhrzeigersinn
        if self.slip_on:
            turn *= 1 + self._turn_err
        v = (ground[0] + ground[1]) / 2
        if self.slip_on:
            v *= 1 - DRIVE_SLIP
        heading = self.heading + math.degrees(turn * dt)
        mid = math.radians((self.heading + heading) / 2)
        if self._move(v * dt * math.sin(mid), v * dt * math.cos(mid), heading):
            for i in (0, 1):
                self.encoder[i] += self.speed[i] * dt
        else:
            self.speed = [0.0, 0.0]
        self._referee()

    def _track_blocked(self, track: int, direction: int) -> bool:
        """Stößt die Kette (links = 0, rechts = 1) vorne bzw. hinten an eine Wand?"""
        lateral = (-1 if track == 0 else 1) * WIDTH / 2
        px, py = self.point(lateral, direction * (LENGTH / 2 + 0.3))
        return any(_dist(px, py, seg) < WALL_HALF for seg in self._segments_near(self.x, self.y))

    def _collides(self, x: float, y: float, heading: float, soft: float = 0.0) -> Segment | None:
        """soft: so tief darf eine Ecke in die Wand drücken (Wand gibt nach), ohne dass es als Kollision zählt."""
        fx, fy, rx, ry = self._axes(heading)
        points = [(x + a * rx + b * fx, y + a * ry + b * fy) for a, b in _OUTLINE]
        for seg in self._segments_near(x, y):
            if any(_dist(px, py, seg) < WALL_HALF - soft for px, py in points):
                return seg
            for ex, ey in ((seg[0], seg[1]), (seg[2], seg[3])):     # Wandende ragt in den Roboter
                lat = (ex - x) * rx + (ey - y) * ry
                lon = (ex - x) * fx + (ey - y) * fy
                if abs(lat) < WIDTH / 2 and abs(lon) < LENGTH / 2:
                    return seg
        return None

    def _move(self, dx: float, dy: float, heading: float) -> bool:
        """Bewegen; an einer Wand entlangrutschen oder von ihr weggeschoben werden. False = steckt fest."""
        seg = self._collides(self.x + dx, self.y + dy, heading)
        if seg is None:
            self.x, self.y, self.heading = self.x + dx, self.y + dy, heading
            return True
        if (abs(heading - self.heading) > 0.3 and math.hypot(dx, dy) < 0.05
                and self._collides(self.x + dx, self.y + dy, heading, CORNER_SOFT_CM)):
            return True     # Drehen auf der Stelle, Ecke streift die Wand: Ketten rutschen, Encoder zählen, er dreht nicht
        ux, uy = seg[2] - seg[0], seg[3] - seg[1]
        length = math.hypot(ux, uy)
        ux, uy = ux / length, uy / length
        along = dx * ux + dy * uy
        qx, qy = _closest(self.x, self.y, seg)
        nx, ny = self.x - qx, self.y - qy
        n = math.hypot(nx, ny) or 1.0
        nx, ny = nx / n * 0.3, ny / n * 0.3
        for x, y, h in ((self.x + along * ux, self.y + along * uy, heading),
                        (self.x + dx + nx, self.y + dy + ny, heading),
                        (self.x + along * ux, self.y + along * uy, self.heading)):
            if self._collides(x, y, h) is None:
                self.x, self.y, self.heading = x, y, h
                return True
        return False

    def _referee(self):
        """Kachel besucht, wenn die Robotermitte darauf ist (mehr als der halbe Roboter)."""
        tile = self.tile_at(self.x, self.y)
        if tile == self.tile:
            return
        tiles = self.world.tiles
        if tiles.get(self.tile) is TileType.BLUE and self.time - self.entered_at < config.BLUE_WAIT_S:
            self.lop += 1
            self.on_event(f"blaue Kachel {self.tile} vor {config.BLUE_WAIT_S:g} s verlassen: Lack of Progress")
        self.tile = tile
        self.entered_at = self.time
        self.visits[tile] += 1
        if tiles.get(tile) is TileType.BLACK:
            self.lop += 1
            self.on_event(f"schwarze Kachel {tile} befahren: Lack of Progress")


# --- Nachgebildete Hardware --------------------------------------------------------------

class SimClock:
    """Sim-Zeit für RealRobot. sleep() rechnet die Physik weiter; mit Steuerung auch Pause und Tempo."""

    def __init__(self, body: SimBody, controller: SimController | None = None,
                 on_tick: Callable[[], None] | None = None):
        self.body = body
        self.controller = controller
        self.on_tick = on_tick or (lambda: None)
        self.t = 0.0
        self._unpaced = 0.0         # Sim-Zeit, für die noch nicht in Echtzeit gewartet wurde

    def monotonic(self) -> float:
        return self.t

    def sleep(self, seconds: float):
        end = self.t + seconds
        while self.t < end - 1e-9:
            if self.controller:
                self.controller.wait_running()
            dt = min(PHYSICS_DT, end - self.t)
            self.body.step(dt)
            self.t += dt
            if self.controller:
                self._unpaced += dt
                if self._unpaced / self.controller.speed >= 0.01:
                    self.controller.sleep(self._unpaced)
                    self._unpaced = 0.0
        self.on_tick()


class SimMotors:
    def __init__(self, body: SimBody):
        self.body = body

    def set_dps(self, left: float, right: float):
        self.body.command = [float(left), float(right)]
        self.body.floating = False

    def positions(self) -> tuple[float, float]:
        return tuple(self.body.encoder)

    def power(self) -> tuple[int | None, int | None]:
        if self.body.floating:
            return None, None
        return tuple(round(max(-100.0, min(100.0, c / 10))) for c in self.body.command)

    def stop(self):
        self.set_dps(0, 0)

    def float(self):
        self.body.command = [0.0, 0.0]
        self.body.floating = True

    def battery(self) -> float:
        return 12.0

    def close(self):
        self.float()


class SimUltrasonic:
    median = Ultrasonic.median          # gleiche Auswertung wie beim echten Sensor
    read_all = Ultrasonic.read_all

    def __init__(self, body: SimBody, clock: SimClock):
        self.body = body
        self.clock = clock
        self.noise = False
        self.rng = random.Random()
        self.last: dict[str, float | None] = {side: None for side in SIDES}
        self._last_ping = -math.inf
        self.calibration = None         # die Simulation misst ohne Fehler, keine Kalibrierung
        self.warning = None

    def ready(self) -> bool:
        return self.clock.monotonic() - self._last_ping >= config.US_CYCLE_S

    def measure(self, side: str) -> float | None:
        pause = config.US_CYCLE_S - (self.clock.monotonic() - self._last_ping)
        if pause > 0:
            self.clock.sleep(pause)
        self._last_ping = self.clock.monotonic()
        cm = self.body.distance(side)
        if self.noise:
            still = not any(self.body.speed)
            cm = None if self.rng.random() < US_DROPOUT else cm + self.rng.gauss(
                0, US_NOISE_STILL_CM if still else US_NOISE_CM)
        if cm is not None and cm > config.US_MAX_DISTANCE_CM:
            cm = None
        self.last[side] = cm
        return cm

    def close(self):
        pass


class SimColorSensor:
    read_color = ColorSensor.read_color     # gleiche Auswertung wie beim echten Sensor
    read_tile = ColorSensor.read_tile

    def __init__(self, body: SimBody, clock: SimClock):
        self.body = body
        self.clock = clock
        self.integration_s = max(1, round(config.COLOR_INTEGRATION_MS / 2.4)) * 0.0024
        self.calibration = dict(RAW)
        self.warning = None
        self.last_raw: tuple[int, int, int, int] | None = None
        self.last_color: str | None = None

    def read_raw(self) -> tuple[int, int, int, int]:
        self.last_raw = RAW[TILE_COLORS[self.body.floor_at(*self.body.color_point())]]
        return self.last_raw

    def close(self):
        pass


class SimHardware:
    """Nachgebildete Hardware für ein Labyrinth."""

    def __init__(self, world: SimWorld, controller: SimController | None = None,
                 noise: bool = False, slip: bool = False, on_tick: Callable[[], None] | None = None):
        self.body = SimBody(world)
        self.clock = SimClock(self.body, controller, on_tick)
        self.motors = SimMotors(self.body)
        self.ultrasonic = SimUltrasonic(self.body, self.clock)
        self.color = SimColorSensor(self.body, self.clock)
        self.set_noise(noise)
        self.set_slip(slip)

    def set_noise(self, on: bool):
        self.ultrasonic.noise = on

    def set_slip(self, on: bool):
        self.body.slip_on = on

    def pose(self) -> dict:
        """Für die Anzeige: Mitte in Kacheln (Kachel i = Wert i) und Drehung in Vierteln."""
        b = self.body
        return {"x": b.x / TILE - 0.5, "y": b.y / TILE - 0.5, "angle": b.heading / 90}

    def beams(self) -> list[dict]:
        """Ultraschall-Strahlen für die Anzeige, in Kacheln (Ecke unten links = 0, 0)."""
        result = []
        for side in SIDES:
            px, py, dx, dy = self.body.sensor_pose(side)
            cm = self.ultrasonic.last[side]
            reach = cm if cm is not None else config.US_MAX_DISTANCE_CM
            result.append({"x0": px / TILE, "y0": py / TILE, "x1": (px + dx * reach) / TILE,
                           "y1": (py + dy * reach) / TILE, "echo": cm is not None})
        return result


class SimDrivingRobot(RealRobot):
    """Der echte Fahrcode. Wartet vor jeder Aktion auf Start / Pause / Schritt der Webseite."""

    def __init__(self, hardware: SimHardware, controller: SimController | None = None,
                 on_change: Callable[[], None] | None = None):
        super().__init__(hardware.motors, hardware.ultrasonic, hardware.color,
                         on_change=on_change, clock=hardware.clock)
        self.controller = controller
        self.actions = 0

    def _begin(self, action: str):
        if self.controller:
            self.controller.wait_turn()
        self.actions += 1
        super()._begin(action)
