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
                 red_mode: str = config.RED_MODE, on_update: Callable[[], None] | None = None):
        if red_mode not in config.RED_MODES:
            raise ValueError(f"Unbekannter Modus für die Gefahrenzone: {red_mode}")
        self.robot = robot
        self.time_limit_s = time_limit_s
        self.red_mode = red_mode                # zuletzt, meiden, normal
        self.enter_red = red_mode == "normal"   # darf die Gefahrenzone gerade betreten werden?
        self.on_update = on_update or (lambda: None)   # wird nach jeder Änderung aufgerufen (Anzeige)

        self.maze = MazeMap()
        self.pos: Pos = START
        self.heading = Dir.N
        self.mode = "bereit"            # bereit, erkunden, heimfahren, Exit-Signal, fertig
        self.planned: list[Pos] = []    # noch abzufahrende Kacheln des aktuellen Plans
        self.last_checkpoint: Pos | None = None
        self.events: deque[str] = deque(maxlen=50)

    # --- Hauptablauf ----------------------------------------------------------

    def run(self):
        self.mode = "erkunden"
        self._log("Start")
        self._sense_start()

        while True:
            targets = self._targets()
            if not targets:
                if self._red_waiting():
                    self.enter_red = True
                    self._log("Rest erkundet, jetzt die Gefahrenzone")
                    continue
                self._log("Alles erkundet")
                break
            actions = plan_path(self.maze, self.pos, self.heading, targets)
            if actions is None:
                self._log("Keine Grenzkachel erreichbar")
                break
            if not self._enough_time_for(actions):
                self._log("Zeit wird knapp, fahre heim")
                break
            self._execute(actions)

        self.mode = "heimfahren"
        self._return_home()
        if self.pos == START:
            self.mode = "Exit-Signal"
            self._log(f"Am Start, blinke {config.EXIT_SIGNAL_S:g} s")
            self.robot.signal_exit()
        else:
            self._log("Start nicht erreicht")
        self.mode = "fertig"
        self.on_update()

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

    def _targets(self) -> set[Pos]:
        """Grenzkacheln, die gerade angefahren werden dürfen."""
        frontier = self.maze.frontier()
        if not self.enter_red:
            frontier = {p for p in frontier if self.maze.kind(p) is not TileType.RED}
        return frontier

    def _red_waiting(self) -> bool:
        """Modus 'zuletzt': Gibt es eine erkannte, noch nicht betretene Gefahrenzone?"""
        return (self.red_mode == "zuletzt" and not self.enter_red
                and any(self.maze.kind(p) is TileType.RED for p in self.maze.frontier()))

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
        avoid = {TileType.BLACK} if self.enter_red else {TileType.BLACK, TileType.RED}
        result = self.robot.forward(avoid)

        if result.status is MoveStatus.BLOCKED:
            self.maze.set_wall(self.pos, self.heading, True)
            self._log(f"Unerwartete Wand bei {self.pos}, plane neu")
            return False
        if result.status is MoveStatus.AVOIDED:
            self.maze.mark_detected(target, result.floor)
            what = "Schwarze Kachel" if result.floor is TileType.BLACK else "Gefahrenzone"
            self._log(f"{what} bei {target}, zurück")
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
        """Reicht die Zeit, um zur Grenzkachel, von dort heim und für das Exit-Signal?"""
        end_pos, end_heading = follow(self.pos, self.heading, actions)[-1]
        home = plan_path(self.maze, end_pos, end_heading, {START})
        if home is None:
            return True
        needed = (estimate_time(self.maze, self.pos, self.heading, actions)
                  + estimate_time(self.maze, end_pos, end_heading, home)
                  + config.EXIT_SIGNAL_S)
        return self.robot.time() + needed + config.TIME_MARGIN_S <= self.time_limit_s

    def _log(self, text: str):
        self.events.append(f"{_clock(self.robot.time())}  {text}")
        self.on_update()


def _clock(seconds: float) -> str:
    return f"{int(seconds) // 60}:{int(seconds) % 60:02d}"


def _tiles_ahead(states: list[tuple[Pos, Dir]]) -> list[Pos]:
    tiles = []
    for pos, _ in states:
        if not tiles or tiles[-1] != pos:
            tiles.append(pos)
    return tiles
