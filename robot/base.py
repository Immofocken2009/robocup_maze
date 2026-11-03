"""Die Schnittstelle zwischen Erkundungslogik und Roboter.

Der Explorer benutzt nur diese Methoden. Dadurch funktioniert derselbe Code
mit dem echten Roboter (real_robot.py) und mit dem Simulator (sim.py).
"""
from dataclasses import dataclass
from enum import Enum

from robot.maze import TileType


class MoveStatus(Enum):
    OK = "ok"            # eine Kachel vorwärts gefahren
    BLOCKED = "blocked"  # Wand vorne, nicht gefahren
    BLACK = "black"      # schwarze Kachel erkannt, zurück auf die alte Kachel gefahren


@dataclass
class MoveResult:
    status: MoveStatus
    floor: TileType | None = None   # Boden der neuen Kachel (nur bei OK)


class RobotBase:
    def read_walls(self) -> dict[str, bool]:
        """Wände um die aktuelle Kachel: {'front': bool, 'left': bool, 'right': bool}."""
        raise NotImplementedError

    def read_floor(self) -> TileType:
        raise NotImplementedError

    def forward(self) -> MoveResult:
        """Eine Kachel vorwärts fahren."""
        raise NotImplementedError

    def turn_left(self):
        raise NotImplementedError

    def turn_right(self):
        raise NotImplementedError

    def wait(self, seconds: float):
        raise NotImplementedError

    def close(self):
        pass
