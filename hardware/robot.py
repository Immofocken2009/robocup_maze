"""Echter Roboter: setzt navigation/robot_interface.py mit der Hardware um.

- Geradeaus: PD-Regler an den Seitenwänden, ohne Wand Encoder-Gleichlauf.
- Kachelende nach dem Abstand vorne (zwischen den Messungen mit Encoder weitergerechnet),
  ohne Wand in Reichweite nach Encoder-Strecke.
- Drehen über Encoder, kein Ausrichten an Wänden (Ketten drehen dort durch). Stattdessen am
  Kachelende um den Schrägstand geradedrehen, der beim Fahren aus den Seitenwänden geschätzt wurde.
- Schwarz/Rot erkennt der Farbsensor vor der Kachelmitte: anhalten, zurück.

Taster (main.py) unterbrechen jede Aktion über interrupt(): K2 -> RunStopped, K3 -> LackOfProgress.
"""
import math
import time
from collections import Counter, deque
from collections.abc import Callable
from itertools import cycle

import config
from hardware.color_sensor import COLOR_TO_TILE, ColorSensor
from hardware.motors import Motors
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


def stop_distance(front_cm: float | None, travelled: float) -> float | None:
    """Fahrstrecke (ab Start der Fahrt), bei der der Roboter mittig auf der nächsten Kachel steht,
    aus einer Messung vorne nach `travelled` cm. Die Wand darf mehrere Kacheln entfernt sein:
    gewählt wird die Kachelmitte, die am nächsten an einer Kachellänge liegt. So passt der
    Haltepunkt auch dann, wenn das Echo von einer anderen Wand oder Wandkante kommt als vorher.
    None = keine Wand in Reichweite."""
    if front_cm is None or front_cm > config.FRONT_TRUST_CM:
        return None
    ahead = travelled + front_cm - config.FRONT_CENTER_CM     # bis zur Mitte der Kachel vor der Wand
    tiles = round((ahead - config.TILE_SIZE_CM) / config.TILE_SIZE_CM)
    stop = ahead - tiles * config.TILE_SIZE_CM
    tolerance = config.FORWARD_TOLERANCE_CM
    return max(config.TILE_SIZE_CM - tolerance, min(config.TILE_SIZE_CM + tolerance, stop))


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


STEER_TEXT = {"both": "lenkt auf die Mitte zwischen beiden Wänden", "left": "lenkt an der linken Wand",
              "right": "lenkt an der rechten Wand", "encoder": "geradeaus nach Encoder"}


class Steering:
    """PD-Regler für das Geradeausfahren. Ergebnis: Lenkkorrektur in dps, positiv = nach links
    (rechte Kette schneller).

    Grundregel: links und rechts gleich. Die Gangbreite B = links + rechts ändert sich beim
    seitlichen Wandern nicht, nur wenn eine Wand wegbricht oder auftaucht. Darauf beruht der
    Breiten-Wächter, der eine Seite ignoriert, sobald sie nicht mehr zu B passt. Fehlt eine Seite,
    hält er zur anderen B / 2. Eine Seite zählt erst nach zwei zusammenpassenden Messungen.
    Den D-Anteil gibt es nur, wenn beide Seiten zählen: Bei einer einzelnen Wand hält er sonst das
    Hochkriechen einer wegbrechenden Wand für Schrägstand.
    """

    def __init__(self, corridor_cm: float = config.CORRIDOR_CM):
        self.corridor = corridor_cm     # Gangbreite einer Kachel (links + rechts), wird nachgeführt
        self.width = corridor_cm        # Gangbreite B, solange beide Seiten zählen
        self.last: dict[str, float | None] = {"left": None, "right": None}
        self.trusted = {"left": False, "right": False}
        self.pending: dict[str, float | None] = {"left": None, "right": None}   # wartet auf Bestätigung
        self.mode: str | None = None    # both, left, right, encoder
        self.offset = 0.0               # letzte Abweichung von der Sollposition (cm, positiv = zu weit rechts)
        self.slope = 0.0                # letzter Schrägstand (Abweichung je cm Fahrt) für den D-Anteil
        self.raw = 0.0                  # Lenkkorrektur vor Begrenzung (dps)
        self.samples: deque[tuple[float, float]] = deque()   # (gefahrene cm, Abweichung cm)
        self.enc_ref = 0.0
        self.corr = 0.0
        self.corr_at = 0.0

    def side(self, side: str, cm: float | None, travelled: float, enc: tuple[float, float]) -> float:
        """Neue Messung einer Seite. travelled: gefahrene cm, enc: Encodergrad seit dem Start."""
        other = "right" if side == "left" else "left"
        prev = self.last[side]
        self.last[side] = cm
        if cm is None or cm > config.STEER_SIDE_MAX_CM:
            self.trusted[side] = False
            self.pending[side] = None
        elif self.trusted[side]:
            if self.trusted[other]:
                diff = cm + self.last[other] - self.width
                if abs(diff) > config.STEER_WIDTH_JUMP_CM:
                    self._distrust(side, cm)            # Wand bricht weg oder taucht auf
                elif abs(diff) < config.STEER_WIDTH_JUMP_CM / 2:
                    self.width += 0.2 * diff            # langsam nachführen (Wände nicht ganz parallel)
            elif prev is not None and abs(cm - prev) > config.STEER_WIDTH_JUMP_CM:
                self._distrust(side, cm)
        elif self.pending[side] is not None and abs(cm - self.pending[side]) <= config.STEER_STEADY_CM:
            self.trusted[side] = True                   # zweite passende Messung: zählt wieder
            self.pending[side] = None
            if self.trusted[other]:
                self.width = cm + self.last[other]
        else:
            self.pending[side] = cm
        if self.trusted["left"] and self.trusted["right"] and self.width <= config.CORRIDOR_MAX_CM:
            self.corridor = self.width
        return self._control(travelled, enc, new_sample=True)

    def hold(self, travelled: float, enc: tuple[float, float]) -> float:
        """Zwischen zwei Seitenmessungen (ohne Seitenwand wird über die Encoder weiter geregelt)."""
        return self._control(travelled, enc, new_sample=False)

    def _distrust(self, side: str, cm: float):
        self.trusted[side] = False
        self.pending[side] = cm

    def _offset(self) -> tuple[str, float]:
        """Abweichung von der Sollposition in cm, positiv = zu weit rechts."""
        left = self.last["left"] if self.trusted["left"] else None
        right = self.last["right"] if self.trusted["right"] else None
        half = self.corridor / 2
        if left is not None and right is not None and left + right <= config.CORRIDOR_MAX_CM:
            return "both", (left - right) / 2           # Gang: links und rechts gleich
        if left is not None and (right is None or left <= right):
            return "left", left - _grid(left, half)     # sonst die nähere bzw. einzige Wand
        if right is not None:
            return "right", _grid(right, half) - right
        return "encoder", 0.0

    def _control(self, travelled: float, enc: tuple[float, float], new_sample: bool) -> float:
        mode, offset = self._offset()
        self.offset = offset
        if mode != self.mode:                           # Wechsel: D-Anteil und Encoder neu beginnen
            self.samples.clear()
            self.enc_ref = enc[0] - enc[1]
            self.mode = mode
        if mode == "encoder":
            raw = config.STEER_ENC_K * (enc[0] - enc[1] - self.enc_ref)
            self.slope = 0.0
        elif not new_sample:
            return self.corr
        else:
            self.samples.append((travelled, offset))
            while len(self.samples) > 2 and travelled - self.samples[1][0] >= config.STEER_D_WINDOW_CM:
                self.samples.popleft()
            slope = 0.0
            t0, o0 = self.samples[0]
            if travelled - t0 >= config.STEER_D_WINDOW_CM / 2:
                slope = (offset - o0) / (travelled - t0)   # positiv = driftet nach rechts
            if not (self.trusted["left"] and self.trusted["right"]):
                slope = 0.0                                 # nur eine Wand: nur P
            raw = config.STEER_KP * offset + config.STEER_KD * slope
            self.slope = slope
        self.raw = raw
        # Lenkänderung pro gefahrenem cm begrenzen: kein Ruck, auch nicht bei einer Fehlmessung
        allowed = config.STEER_SLEW_DPS_PER_CM * max(0.0, travelled - self.corr_at)
        self.corr += _clamp(_clamp(raw, config.STEER_MAX_DPS) - self.corr, allowed)
        self.corr_at = travelled
        return self.corr


    def values(self) -> dict:
        """Alle Werte der Lenkung (für die Aufzeichnung)."""
        pd = self.mode not in (None, "encoder")
        return {"links": self.last["left"], "rechts": self.last["right"],
                "zählt_links": self.trusted["left"], "zählt_rechts": self.trusted["right"],
                "wartet_links": self.pending["left"], "wartet_rechts": self.pending["right"],
                "breite": self.width, "gang": self.corridor, "modus": self.mode,
                "abweichung": self.offset, "schräg": self.slope,
                "p": config.STEER_KP * self.offset if pd else None,
                "d": config.STEER_KD * self.slope if pd else None,
                "encoder_ref": self.enc_ref, "soll": self.raw, "korrektur": self.corr}


class HeadingEstimate:
    """Schrägstand am Ende einer Fahrt, geschätzt aus den Seitenwänden (nur Messungen mit zwei Wänden).

    Die Abweichung (links − rechts) / 2 ändert sich pro gefahrenem cm um sin(Schrägwinkel). Was die
    Lenkung selbst dreht, zeigen die Encoder und wird abgezogen; übrig bleibt der Winkel beim Start,
    geschätzt per Ausgleichsgerade. Schrägstand am Ende = Winkel beim Start + Drehung laut Encoder.
    Die Seitensensoren sitzen US_SIDE_AHEAD_CM vor der Drehmitte: dreht die Lenkung, verschiebt das die
    Messung sofort um US_SIDE_AHEAD_CM · sin(Drehung); das wird ebenfalls abgezogen.
    """

    def __init__(self):
        self.at = 0.0           # gefahrene cm beim letzten track()
        self.angle = 0.0        # Drehung seit dem Start laut Encoder (rad, positiv = nach rechts)
        self.drift = 0.0        # dadurch seitlich gewandert (cm)
        self.samples: list[tuple[float, float]] = []   # (gefahrene cm, Abweichung ohne Encoder-Anteil und Hebel)

    def track(self, travelled: float, enc: tuple[float, float]):
        """Bei jedem Schleifendurchlauf: enc = Encodergrad seit dem Start."""
        self.drift += math.sin(self.angle) * (travelled - self.at)
        self.at = travelled
        self.angle = math.radians(90 * (enc[0] - enc[1]) / 2 / config.MOTOR_DEG_PER_90)

    def sample(self, offset: float):
        """Neue Abweichung, während beide Wände zählen."""
        lever = config.US_SIDE_AHEAD_CM * math.sin(self.angle)
        self.samples.append((self.at, offset - self.drift - lever))

    def degrees(self) -> float | None:
        """Schrägstand jetzt in Grad (positiv = nach rechts gedreht), None = zu wenig Messungen."""
        if len(self.samples) < config.HEADING_MIN_SAMPLES:
            return None
        s = [p[0] for p in self.samples]
        y = [p[1] for p in self.samples]
        if max(s) - min(s) < config.HEADING_MIN_SPAN_CM:
            return None
        ms, my = sum(s) / len(s), sum(y) / len(y)
        slope = sum((a - ms) * (b - my) for a, b in zip(s, y)) / sum((a - ms) ** 2 for a in s)
        return math.degrees(math.asin(_clamp(slope, 1.0)) + self.angle)


def _grid(cm: float, half: float) -> float:
    """Sollabstand zu einer Wand, die (ganze Kacheln weiter) neben der eigenen Kachel steht."""
    return half + config.TILE_SIZE_CM * round((cm - half) / config.TILE_SIZE_CM)


class StallWatch:
    """Merkt, wenn sich die Ketten trotz Fahrbefehl länger nicht drehen (z.B. schräg an einer Wand)."""

    def __init__(self, clock):
        self.clock = clock
        self.ref: tuple[float, float] | None = None
        self.since = clock.monotonic()

    def stalled(self, enc: tuple[float, float]) -> bool:
        now = self.clock.monotonic()
        if self.ref is None or abs(enc[0] - self.ref[0]) + abs(enc[1] - self.ref[1]) > config.STALL_DEG:
            self.ref, self.since = enc, now
            return False
        return now - self.since >= config.STALL_S


class RealRobot(RobotInterface):
    def __init__(self, motors: Motors, ultrasonic: Ultrasonic, color: ColorSensor,
                 on_change: Callable[[], None] | None = None, clock=time):
        self.motors = motors
        self.us = ultrasonic
        self.color = color
        self.on_change = on_change or (lambda: None)   # nach jeder Änderung (Anzeige, wird gedrosselt)
        self.trace = lambda what, **values: None        # jeder Rechenschritt (Aufzeichnung, hardware/run_log.py)
        self.clock = clock      # monotonic() und sleep(); die Simulation gibt ihre Sim-Uhr hinein

        self._interrupt: str | None = None
        self.started_at: float | None = None
        # für die Anzeige
        self.action = "bereit"
        self.detail = ""
        self.motion: dict | None = None     # {"forward_cm": x} oder {"turn_deg": x}, seit Aktionsbeginn
        self.walls: dict[str, bool] | None = None   # zuletzt erkannte Wände
        self.floor: TileType | None = None          # zuletzt erkannter Boden
        self.encoders = (0.0, 0.0)
        self.corridor_cm = config.CORRIDOR_CM   # gemessene Gangbreite, bleibt über die Fahrten erhalten

    # --- Steuerung durch main.py ------------------------------------------------

    def start_clock(self):
        self.started_at = self.clock.monotonic()

    def reset(self):
        """Zustand wie nach dem Einschalten (Motoren aus, Uhr steht)."""
        self.motors.stop()
        self.motors.float()
        self.started_at = None
        self._interrupt = None
        self.motion = None
        self.walls = None
        self.floor = None
        self.set_action("bereit")

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
            return config.CALIBRATION_TIME_S
        return config.CALIBRATION_TIME_S + self.clock.monotonic() - self.started_at

    def read_walls(self) -> dict[str, bool]:
        self._begin("misst Wände")
        values = self.us.read_all(config.US_READINGS)
        self.walls = {side: v is not None and v < config.WALL_THRESHOLD_CM for side, v in values.items()}
        self.trace("wände", werte=values, wände=self.walls)
        self.on_change()
        return self.walls

    def read_floor(self) -> TileType:
        self._begin("misst Boden")
        return self._measure_floor()

    def forward(self, avoid: set[TileType]) -> MoveResult:
        self._begin("prüft vorne")
        front = self.us.median("front")
        if front is not None and front < config.WALL_THRESHOLD_CM:
            self.set_action("Wand vorne", f"{front:.1f} cm")
            return MoveResult(MoveStatus.BLOCKED)
        stop_at = stop_distance(front, 0.0)
        stop_text = "Halt nach Abstand vorne" if stop_at is not None else "Halt nach Encoder"
        stop_at = stop_at or config.TILE_SIZE_CM
        self.trace("vorwärts", vorne=front, halt_bei=stop_at, halt_nach=stop_text, gang=self.corridor_cm)

        l0, r0 = self.motors.positions()
        steering = Steering(self.corridor_cm)
        heading = HeadingEstimate()
        for side in ("left", "right"):          # erste Seitenmessung, zählt nach der zweiten
            cm = self.us.measure(side)
            steering.side(side, cm, 0.0, (0.0, 0.0))
            self.trace("seite", seite=side, cm=cm, gefahren=0.0, **steering.values())
        self.set_action("fährt vorwärts", f"{stop_text}, {STEER_TEXT[steering.mode]}")
        order = cycle(("front", "left", "front", "right"))
        corr, mode = 0.0, steering.mode
        bad, last_color_t = 0, 0.0
        stall = StallWatch(self.clock)

        while True:
            self._check()
            travelled = self._travelled(l0, r0)
            l, r = self.encoders
            heading.track(travelled, (l - l0, r - r0))

            # Boden: schwarz/rot vor der Kachelmitte erkennen
            now = self.clock.monotonic()
            if now - last_color_t >= self.color.integration_s:
                last_color_t = now
                tile = self.color.read_tile()
                bad = bad + 1 if tile in avoid else 0
                self.trace("farbe", boden=tile, meiden=tile in avoid, hintereinander=bad)
                if bad >= config.COLOR_STOP_COUNT:
                    self.motors.stop()
                    self.floor = tile
                    self.set_action(f"{FLOOR_NAMES[tile]} erkannt", "fährt zurück")
                    self.drive_cm(-self._travelled(l0, r0))
                    return MoveResult(MoveStatus.AVOIDED, tile)

            # Ultraschall: abwechselnd vorne und seitlich
            if self.us.ready():
                side = next(order)
                cm = self.us.measure(side)
                travelled = self._travelled(l0, r0)
                if side == "front":
                    grid = stop_distance(cm, travelled)
                    stop_at = grid or stop_at       # jede Messung neu ins Raster
                    self.trace("vorne", cm=cm, gefahren=travelled, raster=grid, halt_bei=stop_at)
                else:
                    l, r = self.encoders
                    heading.track(travelled, (l - l0, r - r0))
                    corr = steering.side(side, cm, travelled, (l - l0, r - r0))
                    if steering.mode == "both":
                        heading.sample(steering.offset)
                    self.trace("seite", seite=side, cm=cm, gefahren=travelled, schräg_proben=len(heading.samples),
                               **steering.values())

            # Ketten blockiert: nicht ewig weiterdrücken, zurück und als Wand melden
            if stall.stalled(self.encoders):
                self.motors.stop()
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
            corr = steering.hold(travelled, (l - l0, r - r0))
            self.trace("fahrt", gefahren=travelled, halt_bei=stop_at, rest=remaining, tempo=speed,
                       korrektur=corr, modus=steering.mode, links_dps=speed - corr, rechts_dps=speed + corr)
            if steering.mode != mode:
                mode = steering.mode
                self.detail = f"{stop_text}, {STEER_TEXT[mode]}"
            self.motors.set_dps(speed - corr, speed + corr)
            self.motion = {"forward_cm": round(travelled, 1)}
            self.on_change()
            self.clock.sleep(LOOP_S)

        self.motors.stop()
        self.corridor_cm = steering.corridor
        l, r = self.encoders
        heading.track(self._travelled(l0, r0), (l - l0, r - r0))
        skew = heading.degrees()
        fix = 0.0
        if skew is not None and abs(skew) >= config.HEADING_FIX_MIN_DEG:
            fix = -_clamp(skew, config.HEADING_FIX_MAX_DEG)
        self.trace("schräglage", proben=len(heading.samples), geschätzt=skew,
                   encoder_drehung=math.degrees(heading.angle), dreht=fix)
        if fix:
            self.detail = f"dreht sich {abs(fix):.1f}° gerade"
            self._rotate(fix)
        floor = self._measure_floor()
        return MoveResult(MoveStatus.OK, floor)

    def turn_left(self):
        self._turn(-1)

    def turn_right(self):
        self._turn(1)

    def wait(self, seconds: float):
        self._begin("wartet")
        self.motors.stop()
        end = self.clock.monotonic() + seconds
        while (left := end - self.clock.monotonic()) > 0:
            self._check()
            self.detail = f"noch {left:.1f} s"
            self.on_change()
            self.clock.sleep(0.05)

    def signal_exit(self):
        # Kein Blinken (Entscheidung Projektinhaber): am Start einfach stehen bleiben.
        self._begin("steht am Start")
        self.motors.stop()

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
            self.motors.stop()
            raise RunStopped if reason == "stop" else LackOfProgress
        if self.started_at is not None and self.time() >= config.TIME_LIMIT_S:
            self.motors.stop()
            raise TimeUp

    def _read_interrupt(self) -> str | None:
        """Unterbrechung durch einen Taster (eigene Methode, damit die Aufzeichnung sie mitschreibt)."""
        return self._interrupt

    def _travelled(self, l0: float, r0: float) -> float:
        self.encoders = self.motors.positions()
        l, r = self.encoders
        return ((l - l0) + (r - r0)) / 2 / config.MOTOR_DEG_PER_CM

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
        self._rotate(direction * 90)

    def _rotate(self, deg: float):
        """Auf der Stelle über die Encoder drehen, positiv = rechts."""
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
        self.motors.stop()

    def drive_cm(self, cm: float):
        """Gerade nach Encoder fahren (negativ = rückwärts), beide Ketten im Gleichlauf."""
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
            l, r = self.encoders
            corr = _clamp(config.STEER_ENC_K * ((l - l0) - (r - r0)), config.STEER_MAX_DPS)
            self.trace("strecke", ziel=cm, gefahren=done, rest=remaining, tempo=speed, korrektur=corr)
            self.motors.set_dps(direction * speed - corr, direction * speed + corr)
            self.on_change()
            self.clock.sleep(LOOP_S)
        self.motors.stop()
