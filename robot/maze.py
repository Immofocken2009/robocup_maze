"""Die Karte des Labyrinths, wie der Roboter sie kennt.

Koordinaten: (x, y), Start ist (0, 0). Der Roboter schaut am Start nach Norden.
Norden = y + 1, Osten = x + 1.
"""
from enum import Enum, IntEnum

Pos = tuple[int, int]


class Dir(IntEnum):
    N = 0
    E = 1
    S = 2
    W = 3

    def left(self) -> "Dir":
        return Dir((self - 1) % 4)

    def right(self) -> "Dir":
        return Dir((self + 1) % 4)

    def back(self) -> "Dir":
        return Dir((self + 2) % 4)


_DELTAS = {Dir.N: (0, 1), Dir.E: (1, 0), Dir.S: (0, -1), Dir.W: (-1, 0)}


def step(pos: Pos, d: Dir) -> Pos:
    dx, dy = _DELTAS[d]
    return (pos[0] + dx, pos[1] + dy)


class TileType(Enum):
    NORMAL = "normal"
    BLACK = "black"     # Loch: nie befahren
    BLUE = "blue"       # Sumpf: warten
    SILVER = "silver"   # Checkpoint


class Maze:
    def __init__(self):
        self.tiles: dict[Pos, TileType] = {}
        self.visited: set[Pos] = set()
        # Wand zwischen zwei Kacheln: True = Wand, False = offen, fehlt = unbekannt
        self._walls: dict[tuple[Pos, Pos], bool] = {}

    @staticmethod
    def _edge(pos: Pos, d: Dir) -> tuple[Pos, Pos]:
        other = step(pos, d)
        return (pos, other) if pos <= other else (other, pos)

    def set_wall(self, pos: Pos, d: Dir, present: bool):
        self._walls[self._edge(pos, d)] = present

    def wall(self, pos: Pos, d: Dir) -> bool | None:
        return self._walls.get(self._edge(pos, d))

    def mark_visited(self, pos: Pos, kind: TileType):
        self.visited.add(pos)
        self.tiles[pos] = kind

    def mark_black(self, pos: Pos):
        self.tiles[pos] = TileType.BLACK

    def kind(self, pos: Pos) -> TileType | None:
        return self.tiles.get(pos)

    def is_passable(self, pos: Pos, d: Dir) -> bool:
        """Bekannt offen und das Ziel ist keine schwarze Kachel."""
        return self.wall(pos, d) is False and self.kind(step(pos, d)) is not TileType.BLACK

    def frontier(self) -> set[Pos]:
        """Unbesuchte Kacheln, die man von einer besuchten Kachel aus erreichen kann."""
        result = set()
        for pos in self.visited:
            for d in Dir:
                nxt = step(pos, d)
                if nxt not in self.visited and self.is_passable(pos, d):
                    result.add(nxt)
        return result

    def render(self, robot_pos: Pos | None = None, robot_dir: Dir | None = None) -> str:
        """ASCII-Karte. '..' / ':' = Wand unbekannt."""
        known = set(self.tiles) | {(0, 0)}
        xs = [p[0] for p in known]
        ys = [p[1] for p in known]
        cell = {TileType.NORMAL: "  ", TileType.BLACK: "##",
                TileType.BLUE: "~~", TileType.SILVER: "$$"}

        def hwall(pos):
            w = self.wall(pos, Dir.N)
            return "--" if w else ("  " if w is False else "..")

        def vwall(pos, d):
            w = self.wall(pos, d)
            return "|" if w else (" " if w is False else ":")

        lines = []
        for y in range(max(ys), min(ys) - 1, -1):
            lines.append("+" + "+".join(hwall((x, y)) for x in range(min(xs), max(xs) + 1)) + "+")
            row = ""
            for x in range(min(xs), max(xs) + 1):
                pos = (x, y)
                if pos == robot_pos and robot_dir is not None:
                    c = "^>v<"[robot_dir] + " "
                elif pos == (0, 0):
                    c = "S "
                elif pos in self.tiles:
                    c = cell[self.tiles[pos]]
                else:
                    c = "??"
                row += vwall(pos, Dir.W) + c
            row += vwall((max(xs), y), Dir.E)
            lines.append(row)
        lines.append("+" + "+".join(
            ("--" if self.wall((x, min(ys)), Dir.S) else
             "  " if self.wall((x, min(ys)), Dir.S) is False else "..")
            for x in range(min(xs), max(xs) + 1)) + "+")
        return "\n".join(lines)
