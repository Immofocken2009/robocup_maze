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
    AVOIDED = "avoided"  # verbotene Kachel erkannt (z.B. schwarz), zurück auf die alte Kachel gefahren


@dataclass
class MoveResult:
    status: MoveStatus
    floor: TileType | None = None   # Boden der neuen Kachel (OK) bzw. der erkannten Kachel (AVOIDED)


class RobotInterface(ABC):
    @abstractmethod
    def read_walls(self) -> dict[str, bool]:
        """Wände an der aktuellen Kachel: {'front': bool, 'left': bool, 'right': bool}."""

    @abstractmethod
    def read_floor(self) -> TileType:
        """Boden unter dem Farbsensor."""

    @abstractmethod
    def forward(self, avoid: set[TileType]) -> MoveResult:
        """Eine Kachel vorwärts fahren. Erkennt der Farbsensor vorne einen Boden aus
        `avoid`, hält der Roboter an und fährt zurück (MoveStatus.AVOIDED)."""

    @abstractmethod
    def turn_left(self):
        """90° nach links drehen."""

    @abstractmethod
    def turn_right(self):
        """90° nach rechts drehen."""

    @abstractmethod
    def align(self, front_wall: bool, back_wall: bool):
        """Nach einer Drehung an einer Wand gerade stellen (laut Karte Wand vorne/hinten)."""

    @abstractmethod
    def wait(self, seconds: float):
        """Stehen bleiben (z.B. auf blauen Kacheln)."""

    @abstractmethod
    def signal_exit(self):
        """Am Start blinken für den Exit-Bonus (config.EXIT_SIGNAL_S)."""

    @abstractmethod
    def time(self) -> float:
        """Spielzeit in Sekunden, inkl. Kalibrierzeit (in der Simulation: Sim-Zeit)."""
