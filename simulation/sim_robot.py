"""Simulierter Roboter: setzt navigation/robot_interface.py um, ohne Hardware.

SimWorld      - das echte Labyrinth (aus einer JSON-Datei in simulation/mazes/)
SimRobot      - fährt in der SimWorld und liefert exakte Sensorwerte
SimController - Start / Pause / Schritt / Geschwindigkeit (wird von der Webseite gesteuert)
"""
import json
import threading
from collections import deque
from collections.abc import Callable
from pathlib import Path

import config
from navigation.maze_map import Dir, MazeMap, Pos, TileType, step
from navigation.robot_interface import MoveResult, MoveStatus, RobotInterface

MAZES_DIR = Path(__file__).resolve().parent / "mazes"


def list_mazes() -> list[str]:
    return sorted(p.stem for p in MAZES_DIR.glob("*.json"))


class SimWorld:
    """Das echte Labyrinth. Koordinaten wie im JSON: x nach Osten, y nach Norden."""

    def __init__(self, width: int, height: int, tiles: dict[Pos, TileType],
                 listed_walls: set[tuple[Pos, Dir]], start: Pos, start_dir: Dir):
        self.width = width
        self.height = height
        self.tiles = tiles
        self.start = start
        self.start_dir = start_dir
        # Wand, wenn eine der beiden Kacheln sie einträgt; Rand ist immer Wand.
        self._walls = MazeMap()
        for pos in tiles:
            for d in Dir:
                nxt = step(pos, d)
                present = (nxt not in tiles or (pos, d) in listed_walls
                           or (nxt, d.back()) in listed_walls)
                self._walls.set_wall(pos, d, present)

    @classmethod
    def load(cls, name: str) -> "SimWorld":
        path = MAZES_DIR / f"{name}.json"
        data = json.loads(path.read_text())
        width, height = data["width"], data["height"]
        tiles: dict[Pos, TileType] = {}
        listed: set[tuple[Pos, Dir]] = set()
        for t in data["tiles"]:
            pos = (t["x"], t["y"])
            if not (0 <= pos[0] < width and 0 <= pos[1] < height):
                raise ValueError(f"{path.name}: Kachel {pos} liegt außerhalb von {width}x{height}")
            if pos in tiles:
                raise ValueError(f"{path.name}: Kachel {pos} ist doppelt")
            tiles[pos] = TileType(t.get("floor", "normal"))
            for letter in t.get("walls", ""):
                listed.add((pos, Dir[letter]))
        start = tuple(data["start"])
        if start not in tiles:
            raise ValueError(f"{path.name}: Start {start} ist keine Kachel")
        return cls(width, height, tiles, listed, start, Dir[data.get("start_dir", "N")])

    def wall(self, pos: Pos, d: Dir) -> bool:
        return self._walls.wall(pos, d) is not False

    def to_world(self, pos: Pos) -> Pos:
        """Roboterkarte -> Labyrinth: um die Startrichtung drehen, zum Start verschieben."""
        x, y = pos
        for _ in range(self.start_dir):
            x, y = y, -x        # 90° im Uhrzeigersinn
        return (self.start[0] + x, self.start[1] + y)

    def to_world_dir(self, d: Dir) -> Dir:
        return Dir((d + self.start_dir) % 4)

    def reachable_tiles(self) -> set[Pos]:
        """Alle Kacheln, die man vom Start aus erreichen kann, ohne schwarze zu befahren."""
        seen = {self.start}
        todo = deque([self.start])
        while todo:
            pos = todo.popleft()
            for d in Dir:
                nxt = step(pos, d)
                if (not self.wall(pos, d) and nxt not in seen
                        and self.tiles[nxt] is not TileType.BLACK):
                    seen.add(nxt)
                    todo.append(nxt)
        return seen


class SimStopped(Exception):
    """Die Simulation wurde beendet (z.B. anderes Labyrinth geladen)."""


class SimController:
    """Bremst die Simulation auf eine einstellbare Geschwindigkeit und erlaubt Pause/Schritt."""

    def __init__(self, speed: float = 1.0):
        self._cond = threading.Condition()
        self.running = False
        self.stopped = False
        self.speed = speed      # 1.0 = Echtzeit, 10.0 = zehnmal so schnell
        self._steps = 0

    def start(self):
        with self._cond:
            self.running = True
            self._cond.notify_all()

    def pause(self):
        with self._cond:
            self.running = False
            self._cond.notify_all()

    def step(self):
        with self._cond:
            self.running = False
            self._steps += 1
            self._cond.notify_all()

    def stop(self):
        with self._cond:
            self.stopped = True
            self._cond.notify_all()

    def set_speed(self, speed: float):
        with self._cond:
            self.speed = max(0.1, speed)

    def wait_turn(self):
        """Vor jeder Aktion: warten, bis sie ausgeführt werden darf."""
        with self._cond:
            self._cond.wait_for(lambda: self.stopped or self.running or self._steps > 0)
            if self.stopped:
                raise SimStopped
            if not self.running:
                self._steps -= 1

    def sleep(self, sim_seconds: float):
        """Nach jeder Aktion: so lange warten, wie sie bei der eingestellten Geschwindigkeit dauert."""
        with self._cond:
            if self._cond.wait_for(lambda: self.stopped, timeout=sim_seconds / self.speed):
                raise SimStopped


class SimRobot(RobotInterface):
    def __init__(self, world: SimWorld, controller: SimController | None = None,
                 on_action: Callable[[], None] | None = None):
        self.world = world
        self.controller = controller        # None = so schnell wie möglich (Tests)
        self.on_action = on_action or (lambda: None)
        self.pos = world.start
        self.dir = world.start_dir
        self.gyro_deg = 0                   # Drehung seit dem Start, wie ein Gyro sie misst
        self.sim_time = 0.0
        self.actions = 0
        self.last_action = "–"
        self.last_duration = 0.0

    # --- Sensoren -------------------------------------------------------------

    def distances(self) -> dict[str, float | None]:
        """Ultraschall-Abstände in cm (None = nichts in Reichweite)."""
        return {
            "front": self._ray(self.dir, config.ROBOT_LENGTH_CM / 2),
            "left": self._ray(self.dir.left(), config.ROBOT_WIDTH_CM / 2),
            "right": self._ray(self.dir.right(), config.ROBOT_WIDTH_CM / 2),
        }

    def _ray(self, d: Dir, sensor_offset_cm: float) -> float | None:
        pos, tiles = self.pos, 0
        while not self.world.wall(pos, d):
            pos = step(pos, d)
            tiles += 1
        dist = tiles * config.TILE_SIZE_CM + config.TILE_SIZE_CM / 2 - sensor_offset_cm
        return dist if dist <= config.US_MAX_DISTANCE_CM else None

    def read_walls(self) -> dict[str, bool]:
        return {side: dist is not None and dist < config.WALL_THRESHOLD_CM
                for side, dist in self.distances().items()}

    def read_floor(self) -> TileType:
        return self.world.tiles[self.pos]

    def time(self) -> float:
        return self.sim_time

    # --- Aktionen -------------------------------------------------------------

    def forward(self) -> MoveResult:
        self._before()
        if self.world.wall(self.pos, self.dir):
            self._after("Wand vorne", 0.0)
            return MoveResult(MoveStatus.BLOCKED)
        nxt = step(self.pos, self.dir)
        if self.world.tiles[nxt] is TileType.BLACK:
            self._after("schwarz erkannt, zurück", config.BLACK_TIME_S)
            return MoveResult(MoveStatus.BLACK)
        self.pos = nxt
        self._after("vorwärts", config.MOVE_TIME_S)
        return MoveResult(MoveStatus.OK, self.read_floor())

    def turn_left(self):
        self._before()
        self.dir = self.dir.left()
        self.gyro_deg -= 90
        self._after("links drehen", config.TURN_TIME_S)

    def turn_right(self):
        self._before()
        self.dir = self.dir.right()
        self.gyro_deg += 90
        self._after("rechts drehen", config.TURN_TIME_S)

    def wait(self, seconds: float):
        self._before()
        self._after("warten", seconds)

    def _before(self):
        if self.controller:
            self.controller.wait_turn()

    def _after(self, action: str, duration: float):
        self.sim_time += duration
        self.actions += 1
        self.last_action = action
        self.last_duration = duration
        self.on_action()
        if self.controller:
            self.controller.sleep(duration)
