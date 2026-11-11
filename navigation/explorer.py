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
                 red_mode: str = config.RED_MODE, on_update: Callable[[], None] | None = None,
                 trace: Callable[..., None] | None = None):
        if red_mode not in config.RED_MODES:
            raise ValueError(f"Unbekannter Modus für die Gefahrenzone: {red_mode}")
        self.robot = robot
        self.time_limit_s = time_limit_s
        self.red_mode = red_mode                # zuletzt, meiden, normal
        self.enter_red = red_mode == "normal"   # darf die Gefahrenzone gerade betreten werden?
        self.on_update = on_update or (lambda: None)   # wird nach jeder Änderung aufgerufen (Anzeige)
        self.trace = trace or (lambda what, **values: None)   # jede Entscheidung (Aufzeichnung)

        self.maze = MazeMap()
        self.pos: Pos = START
        self.heading = Dir.N
        self.mode = "bereit"            # bereit, erkunden, heimfahren, Exit-Signal, fertig, abgebrochen: ...
        self.planned: list[Pos] = []    # noch abzufahrende Kacheln des aktuellen Plans
        self.last_checkpoint: Pos | None = None
        self.checkpoint_heading = Dir.N  # Blickrichtung bei der letzten Ankunft am Checkpoint
        self.events: deque[str] = deque(maxlen=50)
        self.stop_reason: str | None = None   # warum die Erkundung vorzeitig endete (None = alles erkundet)
        self.contradictions = 0         # Widersprüche zwischen Karte und Messung (Folge: die betroffenen Kacheln werden beim Wiederbesuch neu gemessen)
        self.durations: dict[str, tuple[float, int]] = {}   # gemessene Dauer je Art ("F", "B", "T"): (Summe in s, Anzahl)
        self.resense: set[Pos] = set()  # Kacheln, deren Wände beim nächsten Besuch neu gemessen werden (Widerspruch, Lack of Progress)

    # --- Hauptablauf ----------------------------------------------------------

    def run(self):
        self.mode = "erkunden"
        self._log("Start")
        self._sense_start()
        self._explore()

    def lack_of_progress(self):
        """Lack of Progress: Der Roboter steht jetzt auf dem letzten besuchten Checkpoint, mit der
        Blickrichtung seiner letzten Ankunft dort. Ohne Checkpoint: am Start wie zu Beginn des Laufs.
        Die Karte bleibt erhalten."""
        if self.last_checkpoint is not None:
            self.pos, self.heading = self.last_checkpoint, self.checkpoint_heading
        else:
            self.pos, self.heading = START, Dir.N
        self.planned = []
        self.resense.add(self.pos)      # von Hand hingestellt: Wände hier neu messen
        self.mode = "Lack of Progress"
        self._log(f"Lack of Progress, weiter bei {self.pos}")

    def resume(self):
        """Nach Lack of Progress weiterfahren."""
        self.mode = "erkunden"
        if START not in self.maze.visited:
            self._sense_start()
        elif self.pos in self.resense and self.pos in self.maze.visited:
            self.resense.discard(self.pos)
            self._sense_walls()     # von Hand hingestellt: Wände hier neu messen (Stimmen)
        self._explore()

    def _explore(self):
        self.stop_reason = None
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
            self.trace("plan", position=self.pos, blick=self.heading.name, ziele=sorted(targets),
                       aktionen="".join(actions) if actions is not None else None)
            if actions is None:
                self._log("Keine Grenzkachel erreichbar")
                self.stop_reason = f"keine Grenzkachel erreichbar ({len(targets)} Ziele)"
                break
            if not self._enough_time_for(actions):
                self._log("Zeit wird knapp, fahre heim")
                self.stop_reason = "Zeit knapp"
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
        if self.pos != START:
            self.mode = "abgebrochen: Start nicht erreicht" + (f" ({self.stop_reason})" if self.stop_reason else "")
        elif self.stop_reason and self.stop_reason != "Zeit knapp":
            self.mode = f"abgebrochen: {self.stop_reason}"
        else:
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
                t0 = self.robot.time()
                self.robot.turn_left()
                self._measured("T", t0)
                self.heading = self.heading.left()
            elif action == "R":
                t0 = self.robot.time()
                self.robot.turn_right()
                self._measured("T", t0)
                self.heading = self.heading.right()
            elif not self._forward(backward=action == "B"):
                break
        self.planned = []
        self.on_update()

    def _forward(self, backward: bool = False) -> bool:
        """Eine Kachel vorwärts, bei backward rückwärts (Blickrichtung bleibt, nur auf besuchte Kacheln geplant).
        True = Plan weiter abfahren."""
        way = self.heading.back() if backward else self.heading
        target = step(self.pos, way)
        avoid = {TileType.BLACK} if self.enter_red else {TileType.BLACK, TileType.RED}
        t0 = self.robot.time()
        result = self.robot.backward() if backward else self.robot.forward(avoid)
        self.trace("ergebnis", ziel=target, status=result.status.name,
                   boden=result.floor.value if result.floor else None, **({"rückwärts": True} if backward else {}))

        if result.status is MoveStatus.BLOCKED:
            known_open = self.maze.wall(self.pos, way) is False
            if known_open:
                self._contradiction(self.pos, target)   # Karte sagte „offen“
            where = f"{'hinten ' if backward else ''}bei {self.pos}"
            # Bekannt offene Kante, Zielkachel besucht (er ist dort schon durchgefahren): erst beim zweiten Mal Wand,
            # sonst könnte ein einmaliges Verkeilen den einzigen Heimweg abschneiden.
            if known_open and target in self.maze.visited and not self.maze.mark_blocked(self.pos, way):
                self._log(f"Blockiert {where} nach {way.name}, Kante gemieden "
                          f"({self.maze.blocked(self.pos, way)}/{config.WALL_AFTER_BLOCKED})")
                return False
            self.maze.set_wall(self.pos, way, True)
            self._log(f"Unerwartete Wand {where}, plane neu")
            return False
        if result.status is MoveStatus.AVOIDED:
            if not self.maze.mark_detected(target, result.floor):
                self._contradiction(self.pos, target)
                self._log(f"Widerspruch: {result.floor.value} erkannt bei {target}, "
                          f"Karte sagt {self.maze.kind(target).value} (besucht: {target in self.maze.visited})")
            what = "Schwarze Kachel" if result.floor is TileType.BLACK else "Gefahrenzone"
            self._log(f"{what} bei {target}, zurück")
            return False

        self.maze.set_wall(self.pos, way, False)    # befahren = sicher offen (harte Setzung)
        self.maze.clear_blocked(self.pos, way)
        self.pos = target
        is_new = target not in self.maze.visited
        if is_new:
            floor = result.floor or self.robot.read_floor()
            self.maze.mark_visited(target, floor)
            self._sense_walls()
            if floor is TileType.SILVER:
                self._log(f"Checkpoint bei {target}")
        elif target in self.resense:
            self._sense_walls()                 # Karte hat hier widersprochen: beim Wiederbesuch neu messen
        self._measured("B" if backward else "F", t0)    # ohne das Blau-Warten (estimate_time rechnet es getrennt)
        if self.maze.kind(target) is TileType.SILVER:
            self.last_checkpoint = target
            self.checkpoint_heading = self.heading
        if self.maze.kind(target) is TileType.BLUE:
            self._log(f"Blaue Kachel bei {target}, warte {config.BLUE_WAIT_S:g} s")
            self.robot.wait(config.BLUE_WAIT_S)
        return not is_new

    # --- Hilfsfunktionen ------------------------------------------------------

    def _sense_walls(self):
        """Jede Messung ist eine Stimme für die Kante (Wand +1, offen -1, auch bei schon bekannten Kanten): ein einzelner
        Messfehler kippt nichts, wiederholte Gegenstimmen korrigieren die Karte."""
        sides = {"front": self.heading, "left": self.heading.left(), "right": self.heading.right()}
        for side, present in self.robot.read_walls().items():
            d = sides[side]
            if self.maze.wall(self.pos, d) not in (None, present):
                self._contradiction(self.pos, step(self.pos, d))
            self._vote(self.pos, d, present)
        self.resense.discard(self.pos)          # hier gerade gemessen (die Gegenkachel eines Widerspruchs bleibt vorgemerkt)

    def _contradiction(self, a: Pos, b: Pos):
        """Karte und Messung widersprechen sich: zählen, beide Kacheln beim nächsten Besuch neu messen."""
        self.contradictions += 1
        self.resense.update((a, b))

    def _vote(self, pos: Pos, d: Dir, present: bool):
        if self.maze.vote_wall(pos, d, present):
            self._log(f"Wand bei {pos} nach {d.name} korrigiert: jetzt {'Wand' if present else 'offen'}")

    def _measured(self, kind: str, t0: float):
        """Dauer einer Aktion merken (laufender Mittelwert je Art)."""
        total, count = self.durations.get(kind, (0.0, 0))
        self.durations[kind] = (total + self.robot.time() - t0, count + 1)

    def _times(self) -> dict[str, float]:
        """Dauer je Art für die Schätzung: das Größere aus config und dem gemessenen Mittel (ab TIME_LEARN_MIN Messungen)."""
        defaults = {"F": config.MOVE_TIME_S, "B": config.MOVE_BACK_TIME_S, "T": config.TURN_TIME_S}
        times = {}
        for kind, default in defaults.items():
            total, count = self.durations.get(kind, (0.0, 0))
            times[kind] = max(default, total / count) if count >= config.TIME_LEARN_MIN else default
        return times

    def _measured_means(self) -> dict[str, float | None]:
        return {k: (self.durations[k][0] / self.durations[k][1] if k in self.durations else None) for k in ("F", "B", "T")}

    def _enough_time_for(self, actions: list[str]) -> bool:
        """Reicht die Zeit, um zur Grenzkachel, von dort heim und für das Exit-Signal?"""
        end_pos, end_heading = follow(self.pos, self.heading, actions)[-1]
        home = plan_path(self.maze, end_pos, end_heading, {START})
        if home is None:
            self.trace("zeit", heimweg=None, reicht=True)
            return True
        times = self._times()
        there = estimate_time(self.maze, self.pos, self.heading, actions, times)
        back = estimate_time(self.maze, end_pos, end_heading, home, times)
        now = self.robot.time()
        enough = now + there + back + config.EXIT_SIGNAL_S + config.TIME_MARGIN_S <= self.time_limit_s
        self.trace("zeit", jetzt=now, hin=there, heimweg="".join(home), heim=back, exit=config.EXIT_SIGNAL_S,
                   reserve=config.TIME_MARGIN_S, grenze=self.time_limit_s, reicht=enough,
                   gemessen=self._measured_means())
        return enough

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
