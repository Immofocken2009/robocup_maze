"""Echter Roboter: setzt navigation/robot_interface.py mit der Hardware um.

- Geradeaus: PD-Regler an den Seitenwänden, ohne Wand Encoder-Gleichlauf.
- Kachelende nach dem Abstand vorne (zwischen den Messungen mit Encoder weitergerechnet),
  ohne Wand in Reichweite nach Encoder-Strecke.
- Drehen über Encoder, kein Ausrichten an Wänden (Ketten drehen dort durch). Stattdessen am
  Kachelende um den Schrägstand geradedrehen, der beim Fahren aus den Seitenwänden geschätzt wurde.
- Schwarz/Rot erkennt der Farbsensor vor der Kachelmitte: anhalten, zurück.

Rechenhilfen: Lenkung/Schrägstand in hardware/steering.py, Haltepunkt/Stillstand/Zentrieren in hardware/drive_math.py.

Taster (main.py) unterbrechen jede Aktion über interrupt(): K2 -> RunStopped, K3 -> LackOfProgress.
"""
import math
import time
from collections import Counter, deque
from collections.abc import Callable
from itertools import cycle

# Die verschobenen Namen bleiben aus hardware.robot importierbar (bestehender Code, Tests).
import config
from hardware.color_sensor import COLOR_TO_TILE, ColorSensor
from hardware.drive_math import (StallWatch, arrival_fix, arrival_target, centering_plan, stop_distance,  # noqa: F401
                                 stop_distance_back, stop_median)
from hardware.motors import Motors
from hardware.steering import STEER_TEXT, HeadingEstimate, Steering, _clamp, fit_slope  # noqa: F401
from hardware.ultrasonic import Ultrasonic
from navigation.maze_map import TileType
from navigation.robot_interface import MoveResult, MoveStatus, RobotInterface

LOOP_S = 0.01   # Takt der Regelschleifen
FLOOR_READINGS = 3  # Boden im Stand: so viele frische Messungen, die häufigste Farbe gewinnt

FLOOR_NAMES = {TileType.NORMAL: "Weiß", TileType.BLACK: "Schwarz", TileType.BLUE: "Blau",
               TileType.SILVER: "Silber", TileType.RED: "Rot"}


class RunStopped(Exception):
    """K2: Lauf abbrechen."""


class LackOfProgress(Exception):
    """K3: Lack of Progress, der Roboter wird umgesetzt."""


class TimeUp(Exception):
    """Die 8 Minuten sind abgelaufen."""


class SensorsBlind(Exception):
    """Alle drei Ultraschallsensoren melden mehrmals hintereinander gar kein Echo: Hardwarefehler
    (Versorgung, Kabel), der Roboter würde sonst blind weiterfahren und lauter offene Wände eintragen."""


class RealRobot(RobotInterface):
    def __init__(self, motors: Motors, ultrasonic: Ultrasonic, color: ColorSensor,
                 on_change: Callable[[], None] | None = None, clock=time, settings_file=None):
        self.motors = motors
        self.us = ultrasonic
        self.color = color
        self.on_change = on_change or (lambda: None)   # nach jeder Änderung (Anzeige, wird gedrosselt)
        self.trace = lambda what, **values: None        # jeder Rechenschritt (Aufzeichnung, hardware/run_log.py)
        self.clock = clock      # monotonic() und sleep(); die Simulation gibt ihre Sim-Uhr hinein
        self.settings_file = settings_file      # Einstellungen der Webseite; None = nur die Werte aus config.py
        self._defaults = {name: getattr(config, name) for name in config.STEER_SETTINGS}

        self._interrupt: str | None = None
        self.started_at: float | None = None
        self.calibration_s = config.CALIBRATION_TIME_S
        # für die Anzeige
        self.action = "bereit"
        self.detail = ""
        self.motion: dict | None = None     # {"forward_cm": x} oder {"turn_deg": x}, seit Aktionsbeginn
        self.walls: dict[str, bool] | None = None   # zuletzt erkannte Wände
        self.floor: TileType | None = None          # zuletzt erkannter Boden
        self._wall_values = None    # letzte Wandmessung, solange der Roboter seither nicht gefahren oder gedreht ist
        self.blind_streak = 0       # Wandmessungen hintereinander, bei denen alle drei Sensoren kein Echo hatten
        self.encoders = (0.0, 0.0)
        self.corridor_cm = config.CORRIDOR_CM   # gemessene Gangbreite, bleibt über die Fahrten erhalten
        # Freier Raum hinter dem Roboter (cm) fürs Zurücksetzen; kein Sensor hinten: nach einer Fahrt = gefahrene Strecke
        # (er kam von dort), nach einer Drehung None = unbekannt, nach Rückwärtsfahren um x cm entsprechend weniger
        self.rear_free_cm: float | None = None
        # Freier Raum vor dem Roboter (cm), nur nach Rückwärtsfahren bekannt (er kam von dort, `backward`): damit kann er vor
        # einer Drehung auch nach VORNE seitlich zentrieren, wenn hinten nichts bekannt ist. Sonst None = unbekannt.
        self.front_free_cm: float | None = None
        # Abstand vorne (cm) vom Kachelende (`_settle`), gültig nur, solange er sich seither nicht bewegt hat: spart `read_walls`
        # und dem nächsten `forward`/`backward` die Messung vorne. None = nicht vorhanden.
        self._front_fresh: float | None = None

    # --- Steuerung durch main.py ------------------------------------------------

    def start_clock(self, calibration_s: float = config.CALIBRATION_TIME_S):
        """Uhr starten; calibration_s: so viel der Spielzeit ist schon verbraucht (Kalibrieren)."""
        self.calibration_s = calibration_s
        self.started_at = self.clock.monotonic()

    def reset(self):
        """Zustand wie nach dem Einschalten (Motoren aus, Uhr steht)."""
        self.motors.float()
        self.started_at = None
        self._interrupt = None
        self.motion = None
        self.walls = None
        self.floor = None
        self.blind_streak = 0
        self.rear_free_cm = None
        self.front_free_cm = None
        self._front_fresh = None
        self.set_action("bereit")

    def relocated(self):
        """Nach Lack of Progress: der Roboter wurde von Hand umgesetzt, alle Werte von der alten Stelle gelten nicht mehr
        (Wandmessung, Raum hinten/vorne, Abstand vorne vom Kachelende). Sonst nutzt das Zentrieren vor der Drehung Messwerte der
        alten Kachel und einen alten Rückraum. `blind_streak` bleibt (der Sensorausfall hängt nicht vom Ort ab)."""
        self._wall_values = None
        self.rear_free_cm = None
        self.front_free_cm = None
        self._front_fresh = None

    def interrupt(self, reason: str):
        """Von einem anderen Thread: laufende Aktion abbrechen ('stop' oder 'lop')."""
        self._interrupt = reason

    def clear_interrupt(self):
        self._interrupt = None

    def poll_sensors(self):
        """Im Stand alle Sensoren einmal lesen (für die Anzeige)."""
        self.us.read_all()
        self.color.read_color()
        self.encoders = self.motors.positions()

    def sensors(self) -> dict:
        color = self.color.last_color
        return {**self.us.last,
                "rgbc": list(self.color.last_raw) if self.color.last_raw else None,
                "color": COLOR_TO_TILE[color].value if color else None,
                "encoders": [round(e) for e in self.encoders]}

    # --- RobotInterface ---------------------------------------------------------

    def time(self) -> float:
        if self.started_at is None:
            return self.calibration_s
        return self.calibration_s + self.clock.monotonic() - self.started_at

    def load_settings(self) -> dict:
        """Einstellungen der Webseite lesen (eigene Methode, damit die Aufzeichnung sie mitschreibt)."""
        return config.read_settings(self.settings_file)

    def apply_settings(self):
        """Vor jeder Fahrt: Lenk-Einstellungen der Webseite übernehmen (nicht gesetzte = Standardwerte)."""
        if not self.settings_file:
            return
        values = self.load_settings()
        for name, default in self._defaults.items():
            setattr(config, name, values.get(name, default))
        self.trace("einstellungen", werte={name: getattr(config, name) for name in self._defaults})

    def read_walls(self) -> dict[str, bool]:
        self._begin("misst Wände")
        if self._front_fresh is not None:       # vorne schon am Kachelende gemessen (`_settle`): nur die Seiten messen
            values = {"front": self._front_fresh,
                      **{side: self.us.median_fast(side, config.US_READINGS) for side in ("left", "right")}}
        else:
            values = self.us.read_all(config.US_READINGS)
        self.walls = {side: v is not None and v < config.WALL_THRESHOLD_CM for side, v in values.items()}
        self.trace("wände", werte=values, wände=self.walls)
        self._wall_values = values
        self.blind_streak = self.blind_streak + 1 if all(v is None for v in values.values()) else 0
        self.on_change()
        if self.blind_streak >= config.US_BLIND_STOP:
            self.motors.float()
            self.trace("ultraschall_blind", streak=self.blind_streak)
            raise SensorsBlind(config.US_BLIND_WARN)
        return self.walls

    def _center_before_turn(self, values: dict):
        """Vor dem Drehen mittig setzen, damit die Ecken (Drehkreis 12,8 cm) nicht an die Wände stoßen: vorne auf den Kachelmitten-Abstand zurück (Auslaufen ohne Bremse schiebt ihn zu nah an die Wand)
        und seitlich mittig, indem er um einen kleinen Winkel von der näheren Wand weg dreht, schräg zurücksetzt und
        wieder gerade dreht. Die Entscheidung steht in `centering_plan`."""
        front, left, right = values["front"], values["left"], values["right"]
        back, side = centering_plan(front, left, right)
        wants_back = back > 0
        rear = self._rear_room(front)
        back = min(back, rear)
        angle = dist = 0.0
        if side:
            angle = config.DEAD_END_ANGLE_DEG
            dist = min(abs(side) / math.sin(math.radians(angle)), config.DEAD_END_MAX_BACK_CM, rear)
            if dist * math.sin(math.radians(angle)) < config.DEAD_END_SIDE_MIN_CM - 1e-9:
                angle = dist = 0.0                              # hinten zu wenig Platz: Seitenkorrektur würde nichts bringen
        ahead = False
        if side and not dist and not wants_back and self.front_free_cm is not None:
            # hinten nichts bekannt (z.B. nach Rückwärtsfahren), vorne frei: schräg nach VORNE statt zurück
            room = self.front_free_cm if front is None else min(self.front_free_cm, front - config.TURN_FRONT_MIN_CM)
            dist = min(abs(side) / math.sin(math.radians(config.DEAD_END_ANGLE_DEG)), config.DEAD_END_MAX_BACK_CM,
                       max(0.0, room - config.REAR_MARGIN_CM))
            if dist * math.sin(math.radians(config.DEAD_END_ANGLE_DEG)) >= config.DEAD_END_SIDE_MIN_CM - 1e-9:
                angle, ahead = config.DEAD_END_ANGLE_DEG, True
            else:
                dist = 0.0
        extra = max(0.0, back - dist * math.cos(math.radians(angle))) if not ahead else 0.0   # reicht der schräge Weg nicht nach hinten
        if dist or extra >= 0.5:
            self.trace("zentriert_vor_drehung", vorne=front, links=left, rechts=right, seite=side, zurück=back,
                       winkel=angle, strecke=dist, gerade=extra, rueckraum=rear, **({"vorwärts": True} if ahead else {}))
        if dist == 0.0 and extra < 0.5:
            return
        self.set_action("Zentriert vor Drehung", f"{max(dist, extra):.0f} cm")
        self._wall_values = None
        if angle and ahead:
            turn = -angle if side > 0 else angle                # nach links schieben, vorwärts = erst nach links drehen
            self._rotate(turn)
            self.drive_cm(dist)
            self._rotate(-turn)
            self.drive_cm(-dist * math.cos(math.radians(angle)))    # wieder gerade zurück auf die Kachelmitte (der Raum ist frei)
        elif angle:
            turn = angle if side > 0 else -angle                # nach links schieben = erst nach rechts drehen
            self._rotate(turn)
            self.drive_cm(-dist)
            self._rotate(-turn)
        if extra >= 0.5:
            self.drive_cm(-extra)
        elif not ahead and (forward := dist * math.cos(math.radians(angle)) - back) >= 0.5:
            self.drive_cm(forward)          # nur seitlich nötig: wieder vor, sonst steht er nach dem Drehen versetzt

    def _rear_room(self, front: float | None) -> float:
        """Wie weit er hinten Platz hat (cm), ohne Sensor hinten. Bekannt aus der letzten Fahrt (`rear_free_cm`), sonst
        aus dem Abstand vorne: Innenmaß der Kachel − Roboterlänge − vorne (hinten steht dann die Kachelwand), minus
        REAR_MARGIN_CM. Vorne unbekannt: 0 (nie blind zurücksetzen: die Ketten rutschen an der Wand, die Encoder zählen
        trotzdem, Läufe 2026-11-11)."""
        if self.rear_free_cm is not None:
            return max(0.0, self.rear_free_cm)
        if front is None:
            return 0.0
        return max(0.0, config.TILE_SIZE_CM - config.WALL_THICKNESS_CM - config.ROBOT_LENGTH_CM - front
                   - config.REAR_MARGIN_CM)

    def us_blind(self) -> bool:
        """Für die Anzeige: alle drei Sensoren ohne Echo (im Stand sofort, im Lauf ab der ersten Wandmessung)."""
        none = all(v is None for v in self.us.last.values())
        return none and (self.started_at is None or self.blind_streak > 0)

    def read_floor(self) -> TileType:
        self._begin("misst Boden")
        return self._measure_floor()

    def _start_values(self) -> tuple[dict | None, float | None, str]:
        """Beginn von `forward`/`backward`: Messwerte, die noch gelten (Roboter hat sich seit `read_walls` bzw. dem Nachsetzen nicht
        bewegt), nicht noch einmal messen. Ergebnis: (Wandwerte oder None, Abstand vorne, Quelle 'wände'/'frisch'/'gemessen').
        Löscht beides: ab jetzt bewegt er sich."""
        values, fresh = self._wall_values, self._front_fresh
        self._wall_values = None
        self._front_fresh = None
        self.apply_settings()
        self._begin("prüft vorne")
        if values is not None:
            return values, values["front"], "wände"
        if fresh is not None:
            return None, fresh, "frisch"
        return None, self.us.median("front"), "gemessen"

    def _settle(self):
        """Am Kachelende (nach Anhalten und Geradedrehen), wenn vorne eine Wand in Reichweite war: Abstand vorne messen und, wenn er
        um ARRIVE_FIX_MIN_CM bis ARRIVE_FIX_MAX_CM vom Kachelmitten-Abstand abweicht, nachsetzen (`arrival_fix`), dann noch einmal
        messen. Der Wert bleibt in `_front_fresh` (für `read_walls` und den nächsten Start), im Normalfall kostet das also nichts extra."""
        front = self.us.median_fast("front")
        delta = arrival_fix(front)
        if delta:
            self.set_action("Setzt nach", f"{delta:+.1f} cm")
            self.drive_cm(delta)
            after = self.us.median_fast("front")
            self.trace("nachgesetzt", vorne=front, soll=arrival_target(front), strecke=delta, danach=after)
            front = after
        self._front_fresh = front

    def forward(self, avoid: set[TileType]) -> MoveResult:
        values, front, source = self._start_values()
        if front is not None and front < config.WALL_THRESHOLD_CM:
            self.set_action("Wand vorne", f"{front:.1f} cm")
            return MoveResult(MoveStatus.BLOCKED)
        stop_at = stop_distance(front, 0.0)
        last_grid = stop_at                     # Raster der letzten Messung vorne (None = keine Wand in Reichweite)
        grid_hist: deque[float] = deque([] if stop_at is None else [stop_at], maxlen=config.FRONT_STOP_MEDIAN)
        stop_text = "Halt nach Abstand vorne" if stop_at is not None else "Halt nach Encoder"
        stop_at = stop_at or config.TILE_SIZE_CM
        self.trace("vorwärts", vorne=front, halt_bei=stop_at, halt_nach=stop_text, gang=self.corridor_cm, vorne_quelle=source)

        l0, r0 = self.motors.positions()
        base = 0.0                              # schon gefahrene cm vor l0/r0 (nach dem Zurücksetzen bei Wandkontakt neu gesetzt)
        steering = Steering(self.corridor_cm)
        heading = HeadingEstimate()
        for side in ("left", "right"):          # erste Seitenmessung, zählt nach der zweiten
            cm = values[side] if values is not None else self.us.measure(side)
            steering.side(side, cm, base, (0.0, 0.0))
            self.trace("seite", seite=side, cm=cm, gefahren=base, **steering.values())
        tries = 0                               # Zurücksetzen wegen Schleifen an der Wand oder Rutschen vorne
        front_hist: deque[tuple[float, float]] = deque()    # (gefahrene cm, Abstand vorne) für das Festfahren vorne
        slipping = False
        self.set_action("fährt vorwärts", f"{stop_text}, {STEER_TEXT[steering.mode]}")
        order = cycle(("front", "left", "front", "right"))
        corr, mode = 0.0, steering.mode
        bad, last_color_t = 0, 0.0
        stall = StallWatch(self.clock)

        while True:
            self._check()
            travelled = base + self._travelled(l0, r0)
            l, r = self.encoders
            heading.track(travelled, self._eff(l0, r0))

            # Boden: schwarz/rot vor der Kachelmitte erkennen
            now = self.clock.monotonic()
            if now - last_color_t >= self.color.integration_s:
                last_color_t = now
                tile = self.color.read_tile()
                bad = bad + 1 if tile in avoid else 0
                self.trace("farbe", boden=tile, meiden=tile in avoid, hintereinander=bad)
                if bad >= config.COLOR_STOP_COUNT:
                    self.motors.float()
                    self.floor = tile
                    self.set_action(f"{FLOOR_NAMES[tile]} erkannt", "fährt zurück")
                    self.drive_cm(-(base + self._travelled(l0, r0)))
                    return MoveResult(MoveStatus.AVOIDED, tile)

            # Ultraschall: abwechselnd vorne und seitlich
            if self.us.ready():
                side = next(order)
                cm = self.us.measure(side)
                travelled = base + self._travelled(l0, r0)
                if side == "front":
                    grid = stop_distance(cm, travelled)
                    last_grid = grid
                    if grid is not None:            # Median der letzten Rasterwerte: ein Störimpuls verschiebt den Halt nicht
                        grid_hist.append(grid)
                        stop_at = stop_median(grid_hist)
                    self.trace("vorne", cm=cm, gefahren=travelled, raster=grid, halt_bei=stop_at, median=stop_at)
                    if grid is not None and not slipping:
                        slipping = self._front_slips(front_hist, travelled, cm)
                else:
                    l, r = self.encoders
                    heading.track(travelled, self._eff(l0, r0))
                    corr = steering.side(side, cm, travelled, self._eff(l0, r0))
                    if steering.mode == "both":
                        heading.sample(steering.offset)
                    self.trace("seite", seite=side, cm=cm, gefahren=travelled, schräg_proben=len(heading.samples),
                               **steering.values())

            # Schleift an einer Seitenwand (Encoder zeigen es nicht): zurücksetzen, von der Wand weg drehen, weiterfahren.
            # Rutscht er vorne (Abstand sinkt viel weniger als die Encoderstrecke): zurücksetzen, ohne Wegdrehen.
            touching = steering.sticking(travelled)
            if (touching or slipping) and tries < config.CONTACT_MAX_TRIES:
                tries += 1
                self.motors.float()
                if touching:
                    name = "links" if touching == "left" else "rechts"
                    self.trace("schleift", seite=touching, gefahren=travelled, versuch=tries)
                    self.set_action(f"Schleift {name}", "setzt zurück")
                else:
                    self.set_action("Rutscht vorne", "setzt zurück")
                self.drive_cm(-config.CONTACT_BACK_CM)
                if touching:
                    self._rotate(config.CONTACT_ANGLE_DEG if touching == "left" else -config.CONTACT_ANGLE_DEG)
                # Fenster neu: Encoder, Lenkung und Schrägstandschätzung beginnen nach dem Wegdrehen von vorn
                # (die Drehung läge sonst als falscher Schrägstand in den alten Messungen)
                base += self._travelled(l0, r0)
                l0, r0 = self.encoders
                corridor = steering.corridor
                steering = Steering(corridor)
                steering.corr_at = base
                heading = HeadingEstimate()
                heading.at = base
                front_hist.clear()
                grid_hist.clear()
                slipping = False
                for side in ("left", "right"):
                    cm = self.us.measure(side)
                    steering.side(side, cm, base, (0.0, 0.0))
                    self.trace("seite", seite=side, cm=cm, gefahren=base, **steering.values())
                corr, mode = 0.0, steering.mode
                stall = StallWatch(self.clock)
                self.set_action("fährt vorwärts", f"{stop_text}, {STEER_TEXT[steering.mode]}")
                continue

            # Ketten blockiert (oder schleift/rutscht trotz Zurücksetzen): nicht ewig weiterdrücken, zurück und als Wand melden
            if stall.stalled(self.encoders) or touching or slipping:
                self.motors.float()
                self.trace("blockiert", gefahren=travelled, encoder=self.encoders)
                self.set_action("Ketten blockiert", "fährt zurück")
                self.drive_cm(-travelled)
                return MoveResult(MoveStatus.BLOCKED)

            # Wie weit noch? Haltepunkt aus der letzten Messung vorne (Encoder begrenzt ihn)
            remaining = stop_at - travelled
            if remaining <= 0:
                self.trace("angekommen", gefahren=travelled, halt_bei=stop_at)
                break
            speed = config.DRIVE_SPEED_DPS
            l, r = self.encoders
            corr = steering.hold(travelled, self._eff(l0, r0))
            self.trace("fahrt", gefahren=travelled, halt_bei=stop_at, rest=remaining, tempo=speed,
                       korrektur=corr, modus=steering.mode, links_dps=speed - corr,
                       rechts_dps=(speed + corr) * config.MOTOR_TRIM_RIGHT)
            if steering.mode != mode:
                mode = steering.mode
                self.detail = f"{stop_text}, {STEER_TEXT[mode]}"
            self.motors.set_dps(*self._straight(speed - corr, speed + corr))
            self.motion = {"forward_cm": round(travelled, 1)}
            self.on_change()
            self.clock.sleep(LOOP_S)

        self.motors.float()
        self.corridor_cm = steering.corridor
        l, r = self.encoders
        heading.track(base + self._travelled(l0, r0), self._eff(l0, r0))
        self.rear_free_cm = max(0.0, min(base + self._travelled(l0, r0), config.TILE_SIZE_CM))   # er kam von dort
        self.front_free_cm = None
        skew = heading.degrees()
        fix = 0.0
        if skew is not None and abs(skew) >= config.HEADING_FIX_MIN_DEG:
            fix = -_clamp(skew, config.HEADING_FIX_MAX_DEG)
        self.trace("schräglage", proben=len(heading.samples), geschätzt=skew,
                   encoder_drehung=math.degrees(heading.angle), dreht=fix)
        if fix:
            self.detail = f"dreht sich {abs(fix):.1f}° gerade"
            self._rotate(fix)
        if last_grid is not None:
            self._settle()
        floor = self._measure_floor()
        return MoveResult(MoveStatus.OK, floor)

    def backward(self) -> MoveResult:
        """Eine Kachel rückwärts (Blickrichtung bleibt), nur auf eine besuchte Kachel (Farbsensor sitzt vorne).

        Vorzeichen (alles gegen die Fahrtrichtung `direction` = −1 geprüft, Rechnung im Kopf der Entscheidung 2026-11-11):
        - Die Ketten bekommen das Vorwärts-Kommando mal −1: (−(Tempo − k), −(Tempo + k)). k > 0 (Abweichung nach rechts)
          dreht die Blickrichtung im Uhrzeigersinn, rückwärts wandert der Roboter dabei nach links: das Eindrehen wirkt
          wie vorwärts gegen die Abweichung. P und D der `Steering` bleiben unverändert: die Abweichung (links − rechts) / 2
          und ihre Änderung pro gefahrenem cm (positiv = driftet nach rechts, rückwärts = −sin Schrägwinkel) zeigen
          beide in die richtige Richtung (D dämpft den Winkel: Schräg nach rechts rückwärts = Abweichung sinkt → k < 0).
        - `Steering` bekommt die Encoder mal −1 (Strecke zählt positiv, Modus „encoder“ regelt dann wie vorwärts),
          `HeadingEstimate` die echten Encoder (Drehwinkel im Uhrzeigersinn) und direction = −1 (Abweichung wandert
          rückwärts um −sin Winkel).
        - Hebel der Seitensensoren (sitzen US_SIDE_AHEAD_CM vor der Drehmitte): vorwärts dämpft er die Lenkung (Eindrehen
          nach links schiebt den Sensor sofort nach links: wirkt wie ein D-Anteil), rückwärts wäre es eine Mitkopplung
          (Eindrehen im Uhrzeigersinn schiebt den Sensor nach rechts, der Regler sähe noch mehr Abweichung: P-Regler ohne
          Dämpfung, Simulation: bis 16° Schrägstand am Ende). Darum die Seitenwerte für `Steering` so umrechnen, als säßen
          die Sensoren um dasselbe Stück HINTER der Drehmitte (`_back_side`: −2 · Hebel statt +Hebel).
        - Halt: Fahrstrecke TILE_SIZE_CM, bei Wand vorne in Reichweite nach dem wachsenden Abstand vorne ins Raster
          (`stop_distance_back`). Kein Farbsensor-Stopp, kein Zentrieren vorher.
        - Ketten blockiert / Festfahren vorne: zurück nach vorn (zur Startlage), BLOCKED = Wand hinten."""
        values, front, source = self._start_values()
        stop_at = stop_distance_back(front, 0.0)
        last_grid = stop_at
        grid_hist: deque[float] = deque([] if stop_at is None else [stop_at], maxlen=config.FRONT_STOP_MEDIAN)
        stop_text = "Halt nach Abstand vorne" if stop_at is not None else "Halt nach Encoder"
        stop_at = stop_at or config.TILE_SIZE_CM
        self.trace("rückwärts", vorne=front, halt_bei=stop_at, halt_nach=stop_text, gang=self.corridor_cm, vorne_quelle=source)

        l0, r0 = self.motors.positions()
        base = 0.0                              # schon gefahrene cm (positiv) vor l0/r0, nach dem Vorsetzen neu gesetzt
        steering = Steering(self.corridor_cm)
        heading = HeadingEstimate(-1)
        for side in ("left", "right"):
            cm = values[side] if values is not None else self.us.measure(side)
            steering.side(side, cm, base, (0.0, 0.0))
            self.trace("seite", seite=side, cm=cm, gefahren=base, richtung=-1, **steering.values())
        tries = 0
        front_hist: deque[tuple[float, float]] = deque()
        slipping = False
        self.set_action("fährt rückwärts", f"{stop_text}, {STEER_TEXT[steering.mode]}")
        order = cycle(("front", "left", "front", "right"))
        corr, mode = 0.0, steering.mode
        stall = StallWatch(self.clock)

        def distance() -> float:
            return base - self._travelled(l0, r0)       # gefahrene Strecke, positiv

        def enc() -> tuple[float, float]:
            el, er = self._eff(l0, r0)
            return -el, -er                             # für `Steering`: wie vorwärts gezählt

        while True:
            self._check()
            travelled = distance()
            el, er = self._eff(l0, r0)
            heading.track(travelled, (el, er))

            if self.us.ready():
                side = next(order)
                cm = self.us.measure(side)
                travelled = distance()
                if side == "front":
                    grid = stop_distance_back(cm, travelled)
                    last_grid = grid
                    if grid is not None:
                        grid_hist.append(grid)
                        stop_at = stop_median(grid_hist)
                    self.trace("vorne", cm=cm, gefahren=travelled, raster=grid, halt_bei=stop_at, median=stop_at, richtung=-1)
                    if grid is not None and not slipping:
                        slipping = self._front_slips(front_hist, travelled, cm, -1)
                else:
                    el, er = self._eff(l0, r0)
                    heading.track(travelled, (el, er))
                    corr = steering.side(side, self._back_side(side, cm, heading.angle), travelled, enc())
                    if steering.mode == "both":
                        heading.sample(steering.offset)
                    self.trace("seite", seite=side, cm=cm, gefahren=travelled, schräg_proben=len(heading.samples),
                               richtung=-1, **steering.values())

            # Rutscht er (Abstand vorne wächst viel weniger als die Encoderstrecke): vorsetzen, weiter; sonst wie blockiert
            if slipping and tries < config.CONTACT_MAX_TRIES:
                tries += 1
                self.motors.float()
                self.trace("rutscht_zurück", gefahren=travelled, versuch=tries)
                self.set_action("Rutscht hinten", "setzt vor")
                self.drive_cm(config.CONTACT_BACK_CM)
                base = distance()
                l0, r0 = self.encoders
                corridor = steering.corridor
                steering = Steering(corridor)
                steering.corr_at = base
                heading = HeadingEstimate(-1)
                heading.at = base
                front_hist.clear()
                grid_hist.clear()
                slipping = False
                for side in ("left", "right"):
                    cm = self.us.measure(side)
                    steering.side(side, cm, base, (0.0, 0.0))
                    self.trace("seite", seite=side, cm=cm, gefahren=base, richtung=-1, **steering.values())
                corr, mode = 0.0, steering.mode
                stall = StallWatch(self.clock)
                self.set_action("fährt rückwärts", f"{stop_text}, {STEER_TEXT[steering.mode]}")
                continue

            if stall.stalled(self.encoders) or slipping:
                self.motors.float()
                self.trace("blockiert", gefahren=travelled, encoder=self.encoders, richtung=-1)
                self.set_action("Ketten blockiert", "fährt wieder vor")
                self.drive_cm(distance())
                self.rear_free_cm = 0.0         # Wand hinten
                return MoveResult(MoveStatus.BLOCKED)

            remaining = stop_at - travelled
            if remaining <= 0:
                self.trace("angekommen", gefahren=travelled, halt_bei=stop_at, richtung=-1)
                break
            speed = config.DRIVE_SPEED_DPS
            corr = steering.hold(travelled, enc())
            self.trace("fahrt", gefahren=travelled, halt_bei=stop_at, rest=remaining, tempo=speed,
                       korrektur=corr, modus=steering.mode, links_dps=-(speed - corr),
                       rechts_dps=-(speed + corr) * config.MOTOR_TRIM_RIGHT, richtung=-1)
            if steering.mode != mode:
                mode = steering.mode
                self.detail = f"{stop_text}, {STEER_TEXT[mode]}"
            self.motors.set_dps(*self._straight(-(speed - corr), -(speed + corr)))
            self.motion = {"forward_cm": -round(travelled, 1)}
            self.on_change()
            self.clock.sleep(LOOP_S)

        self.motors.float()
        self.corridor_cm = steering.corridor
        heading.track(distance(), self._eff(l0, r0))
        self.rear_free_cm = None        # kein Sensor hinten: unbekannt (Rückraum wird aus dem Abstand vorne geschätzt)
        self.front_free_cm = max(0.0, min(distance(), config.TILE_SIZE_CM))   # er kam von dort
        skew = heading.degrees()
        if skew is None:
            # Kein Schrägstand aus zwei Wänden schätzbar (Raum, eine Wand): wenigstens die eigene Lenkdrehung zurücknehmen
            # (rückwärts ohne Dämpfung, Simulation: Lenkung dreht ihn in der offenen Fläche bis ca. 4° weiter)
            skew = math.degrees(heading.angle)
        fix = 0.0
        if abs(skew) >= config.HEADING_FIX_MIN_DEG:
            fix = -_clamp(skew, config.HEADING_FIX_MAX_DEG)
        self.trace("schräglage", proben=len(heading.samples), geschätzt=skew,
                   encoder_drehung=math.degrees(heading.angle), dreht=fix, richtung=-1)
        if fix:
            self.detail = f"dreht sich {abs(fix):.1f}° gerade"
            self._rotate(fix)
        if last_grid is not None:
            self._settle()
        floor = self._measure_floor()
        return MoveResult(MoveStatus.OK, floor)

    @staticmethod
    def _back_side(side: str, cm: float | None, angle: float) -> float | None:
        """Seitenwert für die Lenkung beim Rückwärtsfahren: der Hebel der Sensoren (cm, positiv wenn der Sensor durch die
        Drehung seit dem Start nach rechts rückt: links +, rechts −) wird doppelt abgezogen, so dass er wie hinter der
        Drehmitte sitzend wirkt (siehe `backward`). angle in rad, positiv = im Uhrzeigersinn gedreht."""
        if cm is None:
            return None
        shift = 2 * config.US_SIDE_AHEAD_CM * math.sin(angle)
        return max(0.0, cm - shift if side == "left" else cm + shift)

    def _front_slips(self, hist: deque, travelled: float, cm: float | None, direction: int = 1) -> bool:
        """Festfahren vorne: sinkt der Abstand vorne über die letzten FRONT_STUCK_DIST_CM cm Encoderstrecke um weniger
        als FRONT_STUCK_RATIO · Strecke, drehen die Ketten durch (Läufe 2026-11-11: 22 cm Encoder, 9,5 cm weniger Abstand).
        hist = (gefahrene cm, Abstand vorne) der Messungen mit Wand in Reichweite. Steigt der Wert (anderes Echo,
        Wandkante), beginnt das Fenster neu. direction -1 = rückwärts: der Abstand wächst statt zu sinken, Vorzeichen gedreht."""
        if cm is None:
            return False
        if hist and direction * (cm - hist[-1][1]) > config.STEER_STEADY_CM:
            hist.clear()
        hist.append((travelled, cm))
        while len(hist) > 1 and travelled - hist[1][0] >= config.FRONT_STUCK_DIST_CM:
            hist.popleft()
        span = travelled - hist[0][0]
        if span < config.FRONT_STUCK_DIST_CM:
            return False
        drop = direction * (hist[0][1] - cm)
        if drop >= config.FRONT_STUCK_RATIO * span:
            return False
        self.trace("rutscht", erwartet=span, gemessen=drop, strecke=span, gefahren=travelled, vorne=cm)
        return True

    def turn_left(self):
        self._turn(-1)

    def turn_right(self):
        self._turn(1)

    def wait(self, seconds: float):
        self._begin("wartet")
        self.motors.float()
        end = self.clock.monotonic() + seconds
        while (left := end - self.clock.monotonic()) > 0:
            self._check()
            self.detail = f"noch {left:.1f} s"
            self.on_change()
            self.clock.sleep(0.05)

    def signal_exit(self):
        # Kein Blinken (Entscheidung Projektinhaber): am Start einfach stehen bleiben.
        self._begin("steht am Start")
        self.motors.float()

    # --- Bausteine --------------------------------------------------------------

    def _begin(self, action: str):
        """Neue Aktion: Der Explorer hat die letzte schon in seine Position übernommen."""
        self._check()
        self.motion = None
        self.set_action(action)

    def set_action(self, action: str, detail: str = ""):
        self.action = action
        self.detail = detail
        self.on_change()

    def _check(self):
        """Taster oder Zeitlimit: anhalten und die Aktion abbrechen."""
        reason = self._read_interrupt()
        if reason:
            self.motors.float()
            raise RunStopped if reason == "stop" else LackOfProgress
        if self.started_at is not None and self.time() >= config.TIME_LIMIT_S:
            self.motors.float()
            raise TimeUp

    def _read_interrupt(self) -> str | None:
        """Unterbrechung durch einen Taster (eigene Methode, damit die Aufzeichnung sie mitschreibt)."""
        return self._interrupt

    def _travelled(self, l0: float, r0: float) -> float:
        self.encoders = self.motors.positions()
        l, r = self.encoders
        return ((l - l0) + (r - r0) / config.MOTOR_TRIM_RIGHT) / 2 / config.MOTOR_DEG_PER_CM

    def _eff(self, l0: float, r0: float) -> tuple[float, float]:
        """Wirksame Encoder seit dem Start beim Geradeausfahren: die rechte Kette ist um den Kettenfaktor
        schneller befohlen, rechts zählt daher (r − r0) / MOTOR_TRIM_RIGHT (gleicher Weg wie links)."""
        l, r = self.encoders
        return l - l0, (r - r0) / config.MOTOR_TRIM_RIGHT

    @staticmethod
    def _straight(left: float, right: float) -> tuple[float, float]:
        """Drehzahlen beim Geradeausfahren: rechts mal Kettenfaktor (Drift-Ausgleich, nicht beim Drehen)."""
        return left, right * config.MOTOR_TRIM_RIGHT

    def _measure_floor(self) -> TileType:
        """Im Stand: frische Messungen (über die Roboter-Uhr gewartet), die häufigste Farbe gewinnt."""
        tiles = []
        for _ in range(FLOOR_READINGS):
            self.clock.sleep(self.color.integration_s)
            tiles.append(self.color.read_tile())
        self.floor = Counter(tiles).most_common(1)[0][0]
        self.trace("boden", messungen=tiles, ergebnis=self.floor)
        self.on_change()
        return self.floor

    def _turn(self, direction: int):
        """90° drehen, direction +1 = rechts, -1 = links."""
        self._begin("dreht rechts" if direction > 0 else "dreht links")
        values = self._wall_values              # frisch nur direkt nach read_walls, sonst (z.B. 2. Drehung) neu messen
        if values is None:
            values = self.us.read_all(config.US_READINGS)
        self._wall_values = None
        self._center_before_turn(values)
        self.set_action("dreht rechts" if direction > 0 else "dreht links")
        self._rotate(direction * 90)

    def _rotate(self, deg: float):
        """Auf der Stelle über die Encoder drehen, positiv = rechts."""
        self._wall_values = None
        self._front_fresh = None
        if abs(deg) >= 45:
            self.rear_free_cm = None            # nach einer Drehung steht hinten etwas anderes
            self.front_free_cm = None
        direction = 1 if deg > 0 else -1
        target = abs(deg) / 90 * config.MOTOR_DEG_PER_90
        l0, r0 = self.motors.positions()
        stall = StallWatch(self.clock)
        while True:
            self._check()
            self.encoders = self.motors.positions()
            l, r = self.encoders
            done = direction * ((l - l0) - (r - r0)) / 2
            remaining = target - done
            if remaining <= 0 or stall.stalled(self.encoders):
                break
            speed = config.TURN_SPEED_DPS
            self.trace("drehen", winkel=deg, ziel=target, gedreht=done, rest=remaining, tempo=speed)
            self.motors.set_dps(direction * speed, -direction * speed)
            self.motion = {**(self.motion or {}),
                           "turn_deg": round(direction * 90 * done / config.MOTOR_DEG_PER_90)}
            self.on_change()
            self.clock.sleep(LOOP_S)
        self.motors.float()

    def drive_cm(self, cm: float):
        """Gerade nach Encoder fahren (negativ = rückwärts), beide Ketten im Gleichlauf."""
        self._wall_values = None
        self._front_fresh = None
        if cm < 0 and self.rear_free_cm is not None:
            self.rear_free_cm = max(0.0, self.rear_free_cm + cm)
        if self.front_free_cm is not None:
            self.front_free_cm = max(0.0, self.front_free_cm - cm)    # vorwärts verkleinert den freien Raum vorne
        direction = 1 if cm >= 0 else -1
        l0, r0 = self.motors.positions()
        stall = StallWatch(self.clock)
        while True:
            self._check()
            done = direction * self._travelled(l0, r0)
            remaining = abs(cm) - done
            if remaining <= 0 or stall.stalled(self.encoders):
                break
            speed = config.DRIVE_SPEED_DPS
            el, er = self._eff(l0, r0)
            corr = _clamp(config.STEER_ENC_K * (el - er), config.STEER_MAX_DPS)
            self.trace("strecke", ziel=cm, gefahren=done, rest=remaining, tempo=speed, korrektur=corr)
            self.motors.set_dps(*self._straight(direction * speed - corr, direction * speed + corr))
            self.on_change()
            self.clock.sleep(LOOP_S)
        self.motors.float()
