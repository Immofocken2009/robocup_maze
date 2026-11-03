"""Simulierter Roboter: setzt navigation/robot_interface.py um, ohne Hardware.

SimWorld      - das echte Labyrinth (JSON-Datei in simulation/mazes/ oder Zufallslabyrinth)
SimRobot      - fährt in der SimWorld, liefert exakte Sensorwerte und zählt Punkte wie ein Schiedsrichter
SimController - Start / Pause / Schritt / Geschwindigkeit (wird von der Webseite gesteuert)
"""
import json
import threading
from collections import Counter, deque
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
        return cls.from_dict(json.loads(path.read_text()), path.name)

    @classmethod
    def from_dict(cls, data: dict, label: str = "Labyrinth") -> "SimWorld":
        width, height = data["width"], data["height"]
        tiles: dict[Pos, TileType] = {}
        listed: set[tuple[Pos, Dir]] = set()
        for t in data["tiles"]:
            pos = (t["x"], t["y"])
            if not (0 <= pos[0] < width and 0 <= pos[1] < height):
                raise ValueError(f"{label}: Kachel {pos} liegt außerhalb von {width}x{height}")
            if pos in tiles:
                raise ValueError(f"{label}: Kachel {pos} ist doppelt")
            tiles[pos] = TileType(t.get("floor", "normal"))
            for letter in t.get("walls", ""):
                listed.add((pos, Dir[letter]))
        start = tuple(data["start"])
        if start not in tiles:
            raise ValueError(f"{label}: Start {start} ist keine Kachel")
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

    def reachable_tiles(self, avoid: frozenset[TileType] = frozenset({TileType.BLACK})) -> set[Pos]:
        """Alle Kacheln, die man vom Start aus erreichen kann, ohne Kacheln aus `avoid` zu befahren."""
        seen = {self.start}
        todo = deque([self.start])
        while todo:
            pos = todo.popleft()
            for d in Dir:
                nxt = step(pos, d)
                if not self.wall(pos, d) and nxt not in seen and self.tiles[nxt] not in avoid:
                    seen.add(nxt)
                    todo.append(nxt)
        return seen

    def linear_tiles(self) -> set[Pos]:
        """Kacheln, die man vom Start aus mit Links- oder Rechtsfolgen erreicht (Regel 3.3.1).
        Schwarze Kacheln zählen als Wand. Alle anderen erreichbaren Kacheln sind 'floating'."""
        def blocked(p: Pos, d: Dir) -> bool:
            return self.wall(p, d) or self.tiles[step(p, d)] is TileType.BLACK

        result = set()
        for first, second in ((Dir.left, Dir.right), (Dir.right, Dir.left)):
            for start_dir in Dir:
                pos, d = self.start, start_dir
                seen = set()
                while (pos, d) not in seen:
                    seen.add((pos, d))
                    result.add(pos)
                    for nd in (first(d), d, second(d), d.back()):
                        if not blocked(pos, nd):
                            pos, d = step(pos, nd), nd
                            break
                    else:
                        break   # rundum zu
        return result


class SimStopped(Exception):
    """Die Simulation wurde beendet (z.B. anderes Labyrinth geladen)."""


class TimeUp(Exception):
    """Die 8 Minuten sind abgelaufen, der Schiedsrichter beendet das Spiel."""


class SimController:
    """Bremst die Simulation auf eine einstellbare Geschwindigkeit und erlaubt Pause/Schritt."""

    def __init__(self, speed: float = 1.0):
        self._cond = threading.Condition()
        self.running = False
        self.stopped = False
        self.speed = speed      # 1.0 = Echtzeit, 10.0 = zehnmal so schnell
        self._steps = 0
        self.stepping = False   # ein Schritt läuft gerade (bis zur nächsten Aktion)

    def start(self):
        with self._cond:
            self.running = True
            self._cond.notify_all()

    def pause(self):
        with self._cond:
            self.running = False
            self.stepping = False
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
            self.stepping = False
            self._cond.wait_for(lambda: self.stopped or self.running or self._steps > 0)
            if self.stopped:
                raise SimStopped
            if not self.running:
                self._steps -= 1
                self.stepping = True

    def wait_running(self):
        """Während einer Aktion (Fahrphysik): bei Pause anhalten. Ein Schritt läuft bis zur nächsten Aktion."""
        with self._cond:
            self._cond.wait_for(lambda: self.stopped or self.running or self.stepping or self._steps > 0)
            if self.stopped:
                raise SimStopped
            if not self.running and not self.stepping:
                self._steps -= 1
                self.stepping = True

    def sleep(self, sim_seconds: float):
        """Nach jeder Aktion: so lange warten, wie sie bei der eingestellten Geschwindigkeit dauert."""
        with self._cond:
            if self._cond.wait_for(lambda: self.stopped, timeout=sim_seconds / self.speed):
                raise SimStopped


class SimRobot(RobotInterface):
    def __init__(self, world: SimWorld, controller: SimController | None = None,
                 on_action: Callable[[], None] | None = None,
                 start_time_s: float = config.CALIBRATION_TIME_S,
                 time_limit_s: float = config.TIME_LIMIT_S):
        self.world = world
        self.controller = controller        # None = so schnell wie möglich (Tests)
        self.on_action = on_action or (lambda: None)
        self.time_limit_s = time_limit_s
        self.pos = world.start
        self.dir = world.start_dir
        self.gyro_deg = 0                   # Drehung seit dem Start, wie ein Gyro sie misst
        self.sim_time = start_time_s
        self.actions = 0
        self.last_action = "–"
        self.last_duration = 0.0
        # Schiedsrichter
        self.visits: Counter[Pos] = Counter({world.start: 1})
        self.lop = 0                        # Lack of Progress
        self.exit_done = False
        self._blue_wait_left = 0.0          # so lange muss noch gewartet werden

    # --- Sensoren -------------------------------------------------------------

    def distances(self) -> dict[str, float | None]:
        """Ultraschall-Abstände in cm (None = nichts in Reichweite)."""
        return {
            "front": self._ray(self.dir, config.TILE_SIZE_CM / 2 - config.FRONT_CENTER_CM),
            "left": self._ray(self.dir.left(), config.TILE_SIZE_CM / 2 - config.SIDE_CENTER_CM),
            "right": self._ray(self.dir.right(), config.TILE_SIZE_CM / 2 - config.SIDE_CENTER_CM),
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

    def forward(self, avoid: set[TileType]) -> MoveResult:
        self._before()
        if self.world.wall(self.pos, self.dir):
            self._after("Wand vorne", 0.0)
            return MoveResult(MoveStatus.BLOCKED)
        nxt = step(self.pos, self.dir)
        floor = self.world.tiles[nxt]
        if floor in avoid:
            self._after(f"{_FLOOR_NAMES[floor]} erkannt, zurück", config.BACKOFF_TIME_S)
            return MoveResult(MoveStatus.AVOIDED, floor)
        if self._blue_wait_left > 0 or floor is TileType.BLACK:
            self.lop += 1   # Regel 5.5: blau nicht abgewartet oder schwarze Kachel befahren
        self.pos = nxt
        self.visits[nxt] += 1
        self._blue_wait_left = config.BLUE_WAIT_S if floor is TileType.BLUE else 0.0
        self._after("vorwärts", config.MOVE_TIME_S)
        return MoveResult(MoveStatus.OK, floor)

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

    def align(self, front_wall: bool, back_wall: bool):
        if front_wall or back_wall:
            self._before()
            self._after("ausrichten", config.ALIGN_TIME_S)

    def wait(self, seconds: float):
        self._before()
        self._blue_wait_left = max(0.0, self._blue_wait_left - seconds)
        self._after("warten", seconds)

    def signal_exit(self):
        self._before()
        self._after("blinken (Exit)", config.EXIT_SIGNAL_S)
        self.exit_done = self.pos == self.world.start and self.sim_time <= self.time_limit_s

    def _before(self):
        if self.sim_time >= self.time_limit_s:
            raise TimeUp
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

    # --- Punkte nach Regelwerk 2026 ---------------------------------------------

    def score(self) -> dict[str, int]:
        return score_2026(self.world.tiles, self.visits, self.lop, self.exit_done)


def score_2026(tiles: dict[Pos, TileType], visits: Counter, lop: int, exit_done: bool) -> dict[str, int]:
    """Punkte nach Regelwerk 2026 (ohne Opfer, Rampen, Treppen, Bodenwellen)."""
    checkpoints = sum(1 for p in visits if tiles.get(p) is TileType.SILVER)
    blue_visits = [n for p, n in visits.items() if tiles.get(p) is TileType.BLUE]
    blue = sum(max(0, config.POINTS_BLUE - config.POINTS_BLUE_REVISIT * (n - 1)) for n in blue_visits)
    sbv = sum(1 for n in blue_visits if n == 1)
    result = {
        "checkpoints": checkpoints * config.POINTS_CHECKPOINT,
        "blue": blue,
        "reliability": max(0, sbv * config.BONUS_PER_SBV - lop * config.PENALTY_PER_LOP),
        "exit": sbv * config.BONUS_PER_SBV if exit_done else 0,
    }
    result["total"] = sum(result.values())
    result["lop"] = lop
    return result


_FLOOR_NAMES = {TileType.NORMAL: "normal", TileType.BLACK: "schwarz", TileType.BLUE: "blau",
                TileType.SILVER: "silber", TileType.RED: "rot"}
