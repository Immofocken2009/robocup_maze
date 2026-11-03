"""Die Karte des Labyrinths.

Koordinaten (x, y): x nach Osten, y nach Norden. In der Roboterkarte ist der
Start (0, 0) und der Roboter schaut am Start nach "Norden".
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
    RED = "red"         # Eingang zur Gefahrenzone


class MazeMap:
    def __init__(self):
        self.tiles: dict[Pos, TileType] = {}   # bekannte Kacheln (besucht oder von außen erkannt)
        self.visited: set[Pos] = set()
        # Wand zwischen zwei Kacheln: True = Wand, False = offen, fehlt = unbekannt.
        # Schlüssel ist das Kachelpaar (kleinere zuerst), so ist jede Wand nur einmal gespeichert.
        self._walls: dict[tuple[Pos, Pos], bool] = {}

    @staticmethod
    def _edge(pos: Pos, d: Dir) -> tuple[Pos, Pos]:
        other = step(pos, d)
        return (pos, other) if pos <= other else (other, pos)

    def set_wall(self, pos: Pos, d: Dir, present: bool):
        self._walls[self._edge(pos, d)] = present

    def wall(self, pos: Pos, d: Dir) -> bool | None:
        return self._walls.get(self._edge(pos, d))

    def walls(self) -> list[tuple[Pos, Dir, bool]]:
        """Alle bekannten Wände als (Kachel, Richtung N oder E, Wand ja/nein)."""
        result = []
        for (a, b), present in self._walls.items():
            d = Dir.E if b[0] > a[0] else Dir.N
            result.append((a, d, present))
        return result

    def mark_visited(self, pos: Pos, kind: TileType):
        self.visited.add(pos)
        self.tiles[pos] = kind

    def mark_detected(self, pos: Pos, kind: TileType):
        """Boden erkannt, ohne die Kachel zu befahren (z.B. schwarz oder rot)."""
        self.tiles[pos] = kind

    def kind(self, pos: Pos) -> TileType | None:
        return self.tiles.get(pos)

    def is_passable(self, pos: Pos, d: Dir) -> bool:
        """Verbindung ist bekannt offen und das Ziel ist keine schwarze Kachel."""
        return self.wall(pos, d) is False and self.kind(step(pos, d)) is not TileType.BLACK

    def frontier(self) -> set[Pos]:
        """Grenzkacheln: unbesucht, aber über eine offene Wand von einer besuchten Kachel erreichbar."""
        result = set()
        for pos in self.visited:
            for d in Dir:
                nxt = step(pos, d)
                if nxt not in self.visited and self.is_passable(pos, d):
                    result.add(nxt)
        return result
