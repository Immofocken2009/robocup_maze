"""Schnittstelle zwischen Explorer und Roboter.

Der Explorer benutzt nur diese Methoden. Die Simulation (simulation/sim_robot.py)
und später der echte Roboter setzen sie um.
"""
from abc import ABC, abstractmethod
from dataclasses import dataclass
from enum import Enum

from navigation.maze_map import TileType


class MoveStatus(Enum):
    OK = "ok"            # eine Kachel vorwärts gefahren
    BLOCKED = "blocked"  # Wand vorne, nicht gefahren
    BLACK = "black"      # schwarze Kachel erkannt, zurück auf die alte Kachel gefahren


@dataclass
class MoveResult:
    status: MoveStatus
    floor: TileType | None = None   # Boden der neuen Kachel (nur bei OK)


class RobotInterface(ABC):
    @abstractmethod
    def read_walls(self) -> dict[str, bool]:
        """Wände an der aktuellen Kachel: {'front': bool, 'left': bool, 'right': bool}."""

    @abstractmethod
    def read_floor(self) -> TileType:
        """Boden unter dem Farbsensor."""

    @abstractmethod
    def forward(self) -> MoveResult:
        """Eine Kachel vorwärts fahren."""

    @abstractmethod
    def turn_left(self):
        """90° nach links drehen."""

    @abstractmethod
    def turn_right(self):
        """90° nach rechts drehen."""

    @abstractmethod
    def wait(self, seconds: float):
        """Stehen bleiben (z.B. auf blauen Kacheln)."""

    @abstractmethod
    def time(self) -> float:
        """Sekunden seit dem Start des Laufs (in der Simulation: Sim-Zeit)."""
