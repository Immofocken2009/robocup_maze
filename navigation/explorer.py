"""Erkundungslogik: Karte aufbauen, Grenzkacheln anfahren, rechtzeitig heimfahren."""
from collections import deque
from collections.abc import Callable

import config
from navigation.maze_map import Dir, MazeMap, Pos, TileType, step
from navigation.pathfinding import estimate_time, follow, plan_path
from navigation.robot_interface import MoveStatus, RobotInterface

START: Pos = (0, 0)


class Explorer:
    def __init__(self, robot: RobotInterface, time_limit_s: float = config.TIME_LIMIT_S,
                 on_update: Callable[[], None] | None = None):
        self.robot = robot
        self.time_limit_s = time_limit_s
        self.on_update = on_update or (lambda: None)   # wird nach jeder Änderung aufgerufen (Anzeige)

        self.maze = MazeMap()
        self.pos: Pos = START
        self.heading = Dir.N
        self.mode = "bereit"            # bereit, erkunden, heimfahren, fertig
        self.planned: list[Pos] = []    # noch abzufahrende Kacheln des aktuellen Plans
        self.last_checkpoint: Pos | None = None
        self.events: deque[str] = deque(maxlen=50)

    # --- Hauptablauf ----------------------------------------------------------

    def run(self):
        self.mode = "erkunden"
        self._log("Start")
        self._sense_start()

        while True:
            frontier = self.maze.frontier()
            if not frontier:
                self._log("Alles erkundet")
                break
            actions = plan_path(self.maze, self.pos, self.heading, frontier)
            if actions is None:
                self._log("Keine Grenzkachel erreichbar")
                break
            if not self._enough_time_for(actions):
                self._log("Zeit wird knapp, fahre heim")
                break
            self._execute(actions)

        self.mode = "heimfahren"
        self._return_home()
        self.mode = "fertig"
        self._log("Am Start angekommen" if self.pos == START else "Start nicht erreicht")

    def _sense_start(self):
        """Boden und Wände messen, 90° drehen, nochmal messen (kein Sensor hinten)."""
        self.maze.mark_visited(self.pos, self.robot.read_floor())
        self._sense_walls()
        self.on_update()
        self.robot.turn_right()
        self.heading = self.heading.right()
        self._sense_walls()
        self.on_update()

    def _return_home(self):
        while self.pos != START:
            actions = plan_path(self.maze, self.pos, self.heading, {START})
            if actions is None:
                self._log("Kein Weg zum Start bekannt")
                return
            self._execute(actions)

    # --- Fahren ---------------------------------------------------------------

    def _execute(self, actions: list[str]):
        """Aktionen abfahren. Bricht ab, wenn eine neue Kachel erreicht wurde
        oder etwas Unerwartetes passiert (dann wird neu geplant)."""
        states = follow(self.pos, self.heading, actions)
        for i, action in enumerate(actions):
            self.planned = _tiles_ahead(states[i:])
            self.on_update()
            if action == "L":
                self.robot.turn_left()
                self.heading = self.heading.left()
            elif action == "R":
                self.robot.turn_right()
                self.heading = self.heading.right()
            elif not self._forward():
                break
        self.planned = []
        self.on_update()

    def _forward(self) -> bool:
        """Eine Kachel vorwärts. True = Plan weiter abfahren."""
        target = step(self.pos, self.heading)
        result = self.robot.forward()

        if result.status is MoveStatus.BLOCKED:
            self.maze.set_wall(self.pos, self.heading, True)
            self._log(f"Unerwartete Wand bei {self.pos}, plane neu")
            return False
        if result.status is MoveStatus.BLACK:
            self.maze.mark_black(target)
            self._log(f"Schwarze Kachel bei {target}, zurück")
            return False

        self.pos = target
        is_new = target not in self.maze.visited
        if is_new:
            floor = result.floor or self.robot.read_floor()
            self.maze.mark_visited(target, floor)
            self._sense_walls()
            if floor is TileType.SILVER:
                self.last_checkpoint = target
                self._log(f"Checkpoint bei {target}")
        if self.maze.kind(target) is TileType.BLUE:
            self._log(f"Blaue Kachel bei {target}, warte {config.BLUE_WAIT_S:g} s")
            self.on_update()
            self.robot.wait(config.BLUE_WAIT_S)
        return not is_new

    # --- Hilfsfunktionen ------------------------------------------------------

    def _sense_walls(self):
        """Wände nur eintragen, wenn sie noch unbekannt sind (gegen Messrauschen)."""
        sides = {"front": self.heading, "left": self.heading.left(), "right": self.heading.right()}
        for side, present in self.robot.read_walls().items():
            d = sides[side]
            if self.maze.wall(self.pos, d) is None:
                self.maze.set_wall(self.pos, d, present)

    def _enough_time_for(self, actions: list[str]) -> bool:
        """Reicht die Zeit, um zur Grenzkachel und von dort heim zu fahren?"""
        end_pos, end_heading = follow(self.pos, self.heading, actions)[-1]
        home = plan_path(self.maze, end_pos, end_heading, {START})
        if home is None:
            return True
        needed = (estimate_time(self.maze, self.pos, self.heading, actions)
                  + estimate_time(self.maze, end_pos, end_heading, home))
        return self.robot.time() + needed + config.TIME_MARGIN_S <= self.time_limit_s

    def _log(self, text: str):
        self.events.append(f"{self.robot.time():6.1f} s  {text}")
        self.on_update()


def _tiles_ahead(states: list[tuple[Pos, Dir]]) -> list[Pos]:
    tiles = []
    for pos, _ in states:
        if not tiles or tiles[-1] != pos:
            tiles.append(pos)
    return tiles
