"""Die Karte des Labyrinths.

Koordinaten (x, y): x nach Osten, y nach Norden. In der Roboterkarte ist der
Start (0, 0) und der Roboter schaut am Start nach "Norden".
"""
from enum import Enum, IntEnum

import config

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
        # Wand zwischen zwei Kacheln als Stimmenzähler: > 0 = Wand, < 0 = offen, 0 = Gleichstand (bisheriger Wert bleibt), fehlt = unbekannt. Jede Messung gibt eine
        # Stimme (Wand +1, offen -1), begrenzt auf +-WALL_VOTE_HARD. Schlüssel ist das Kachelpaar (kleinere zuerst),
        # so ist jede Wand nur einmal gespeichert.
        self._votes: dict[tuple[Pos, Pos], int] = {}
        self._last_known: dict[tuple[Pos, Pos], bool] = {}   # zuletzt bekannter Wert (für das Erkennen von Vorzeichenwechseln)
        # Wie oft der Roboter auf dieser Kante „blockiert“ meldete, obwohl sie als offen galt (Zielkachel besucht).
        self._blocked: dict[tuple[Pos, Pos], int] = {}

    @staticmethod
    def _edge(pos: Pos, d: Dir) -> tuple[Pos, Pos]:
        other = step(pos, d)
        return (pos, other) if pos <= other else (other, pos)

    def set_wall(self, pos: Pos, d: Dir, present: bool):
        """Harte Setzung (blockiert, Laden, Tests): Zähler auf +-WALL_VOTE_HARD."""
        edge = self._edge(pos, d)
        self._votes[edge] = config.WALL_VOTE_HARD if present else -config.WALL_VOTE_HARD
        self._last_known[edge] = present

    def vote_wall(self, pos: Pos, d: Dir, present: bool) -> bool:
        """Eine Messung als Stimme eintragen (Wand +1, offen -1). True = die Kante kippte auf das Gegenteil des
        zuletzt bekannten Werts (Vorzeichenwechsel; bei Gleichstand bleibt der alte Wert)."""
        edge = self._edge(pos, d)
        hard = config.WALL_VOTE_HARD
        self._votes[edge] = max(-hard, min(hard, self._votes.get(edge, 0) + (1 if present else -1)))
        now = self.wall(pos, d)
        flipped = self._last_known.get(edge, now) != now
        self._last_known[edge] = now
        return flipped

    def wall(self, pos: Pos, d: Dir) -> bool | None:
        edge = self._edge(pos, d)
        votes = self._votes.get(edge, 0)
        if votes == 0:
            return self._last_known.get(edge)   # Gleichstand: der bisherige Wert bleibt (nie wieder „unbekannt“ nach einer Messung)
        return votes > 0

    def walls(self) -> list[tuple[Pos, Dir, bool]]:
        """Alle bekannten Wände als (Kachel, Richtung N oder E, Wand ja/nein)."""
        result = []
        for (a, b) in self._votes:
            d = Dir.E if b[0] > a[0] else Dir.N
            present = self.wall(a, d)
            if present is not None:
                result.append((a, d, present))
        return result

    def mark_blocked(self, pos: Pos, d: Dir) -> bool:
        """„Blockiert“ auf dieser Kante melden. True = jetzt als Wand eintragen (Zähler hat WALL_AFTER_BLOCKED erreicht)."""
        edge = self._edge(pos, d)
        self._blocked[edge] = self._blocked.get(edge, 0) + 1
        return self._blocked[edge] >= config.WALL_AFTER_BLOCKED

    def blocked(self, pos: Pos, d: Dir) -> int:
        return self._blocked.get(self._edge(pos, d), 0)

    def clear_blocked(self, pos: Pos, d: Dir):
        """Die Kante wurde sicher befahren: der Verdacht entfällt."""
        self._blocked.pop(self._edge(pos, d), None)

    def mark_visited(self, pos: Pos, kind: TileType):
        self.visited.add(pos)
        self.tiles[pos] = kind

    def mark_detected(self, pos: Pos, kind: TileType) -> bool:
        """Boden erkannt, ohne die Kachel zu befahren (z.B. schwarz oder rot).
        Eine besuchte Kachel oder eine mit schon anderem bekanntem Typ wird nicht überschrieben
        (False = Widerspruch, Karte bleibt)."""
        old = self.tiles.get(pos)
        if pos in self.visited or (old is not None and old != kind):
            return False
        self.tiles[pos] = kind
        return True

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
