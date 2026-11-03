"""Echter Roboter: setzt navigation/robot_interface.py mit der Hardware um.

- Geradeaus: PD-Regler an den Seitenwänden, ohne Wand Encoder-Gleichlauf.
- Kachelende nach dem Abstand vorne (zwischen den Messungen mit Encoder weitergerechnet),
  ohne Wand in Reichweite nach Encoder-Strecke.
- Drehen über Encoder, danach an einer Wand vorne oder hinten ausrichten (align).
- Schwarz/Rot erkennt der Farbsensor vor der Kachelmitte: anhalten, zurück.

Taster (main.py) unterbrechen jede Aktion über interrupt(): K2 -> RunStopped, K3 -> LackOfProgress.
"""
import time
from collections import deque
from collections.abc import Callable
from itertools import cycle

import config
from hardware.color_sensor import COLOR_TO_TILE, ColorSensor
from hardware.motors import Motors
from hardware.ultrasonic import Ultrasonic
from navigation.maze_map import TileType
from navigation.robot_interface import MoveResult, MoveStatus, RobotInterface

LOOP_S = 0.01   # Takt der Regelschleifen

FLOOR_NAMES = {TileType.NORMAL: "Weiß", TileType.BLACK: "Schwarz", TileType.BLUE: "Blau",
               TileType.SILVER: "Silber", TileType.RED: "Rot"}


class RunStopped(Exception):
    """K2: Lauf abbrechen."""


class LackOfProgress(Exception):
    """K3: Lack of Progress, der Roboter wird umgesetzt."""


class TimeUp(Exception):
    """Die 8 Minuten sind abgelaufen."""


def front_target(front_cm: float | None) -> float | None:
    """Abstand vorne, bei dem der Roboter mittig auf der nächsten Kachel steht.
    None = keine Wand in Reichweite, nach Encoder fahren."""
    if front_cm is None or front_cm > config.FRONT_TRUST_CM:
        return None
    tiles = max(0, round((front_cm - config.FRONT_CENTER_CM) / config.TILE_SIZE_CM) - 1)
    return config.FRONT_CENTER_CM + tiles * config.TILE_SIZE_CM


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


class Steering:
    """PD-Regler für das Geradeausfahren. Ergebnis: Lenkkorrektur in dps, positiv = nach links
    (rechte Kette schneller)."""

    def __init__(self):
        self.mode: str | None = None    # both, left, right, encoder
        self.samples: deque[tuple[float, float]] = deque()   # (gefahrene cm, Abweichung cm)
        self.enc_ref = 0.0

    def update(self, left: float | None, right: float | None, travelled: float,
               enc: tuple[float, float]) -> float:
        """left/right: Seitenabstände (cm), travelled: gefahrene cm, enc: Encodergrad seit dem Start."""
        has_left = left is not None and left < config.WALL_THRESHOLD_CM
        has_right = right is not None and right < config.WALL_THRESHOLD_CM
        if has_left and has_right:
            mode, offset = "both", (left - right) / 2      # positiv = rechts von der Mitte
        elif has_left:
            mode, offset = "left", left - config.SIDE_CENTER_CM
        elif has_right:
            mode, offset = "right", config.SIDE_CENTER_CM - right
        else:
            mode, offset = "encoder", 0.0

        if mode != self.mode:
            self.samples.clear()
            self.enc_ref = enc[0] - enc[1]
            self.mode = mode
        if mode == "encoder":
            return _clamp(config.STEER_ENC_K * (enc[0] - enc[1] - self.enc_ref), config.STEER_MAX_DPS)

        self.samples.append((travelled, offset))
        while len(self.samples) > 2 and travelled - self.samples[1][0] >= config.STEER_D_WINDOW_CM:
            self.samples.popleft()
        slope = 0.0
        t0, o0 = self.samples[0]
        if travelled - t0 >= config.STEER_D_WINDOW_CM / 2:
            slope = (offset - o0) / (travelled - t0)       # positiv = driftet nach rechts
        return _clamp(config.STEER_KP * offset + config.STEER_KD * slope, config.STEER_MAX_DPS)


class RealRobot(RobotInterface):
    def __init__(self, motors: Motors, ultrasonic: Ultrasonic, color: ColorSensor,
                 on_change: Callable[[], None] | None = None):
        self.motors = motors
        self.us = ultrasonic
        self.color = color
        self.on_change = on_change or (lambda: None)   # nach jeder Änderung (Anzeige, wird gedrosselt)

        self._interrupt: str | None = None
        self.started_at: float | None = None
        # für die Anzeige
        self.action = "bereit"
        self.detail = ""
        self.motion: dict | None = None     # {"forward_cm": x} oder {"turn_deg": x}, seit Aktionsbeginn
        self.walls: dict[str, bool] | None = None   # zuletzt erkannte Wände
        self.floor: TileType | None = None          # zuletzt erkannter Boden
        self.encoders = (0.0, 0.0)

    # --- Steuerung durch main.py ------------------------------------------------

    def start_clock(self):
        self.started_at = time.monotonic()

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
        return config.CALIBRATION_TIME_S + time.monotonic() - self.started_at

    def read_walls(self) -> dict[str, bool]:
        self._begin("misst Wände")
        values = self.us.read_all(config.US_READINGS)
        self.walls = {side: v is not None and v < config.WALL_THRESHOLD_CM for side, v in values.items()}
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
        target = front_target(front)
        self.set_action("fährt vorwärts", "nach Abstand vorne" if target is not None else "nach Encoder")

        l0, r0 = self.motors.positions()
        steering = Steering()
        sides = {"left": self.us.last["left"], "right": self.us.last["right"]}
        front_at, front_travel = front, 0.0
        order = cycle(("front", "left", "front", "right"))
        corr = steering.update(sides["left"], sides["right"], 0.0, (0.0, 0.0))
        bad, last_color_t = 0, 0.0

        while True:
            self._check()
            travelled = self._travelled(l0, r0)

            # Boden: schwarz/rot vor der Kachelmitte erkennen
            now = time.monotonic()
            if now - last_color_t >= self.color.integration_s:
                last_color_t = now
                tile = self.color.read_tile()
                bad = bad + 1 if tile in avoid else 0
                if bad >= config.COLOR_STOP_COUNT:
                    self.motors.stop()
                    self.floor = tile
                    self.set_action(f"{FLOOR_NAMES[tile]} erkannt", "fährt zurück")
                    self._drive_cm(-self._travelled(l0, r0))
                    return MoveResult(MoveStatus.AVOIDED, tile)

            # Ultraschall: abwechselnd vorne und seitlich
            if self.us.ready():
                side = next(order)
                cm = self.us.measure(side)
                travelled = self._travelled(l0, r0)
                if side == "front":
                    if cm is not None:
                        front_at, front_travel = cm, travelled
                else:
                    sides[side] = cm
                    l, r = self.encoders
                    corr = steering.update(sides["left"], sides["right"], travelled, (l - l0, r - r0))

            # Wie weit noch? Abstand vorne, begrenzt durch die Encoder-Strecke
            remaining = config.TILE_SIZE_CM - travelled
            if target is not None and front_at is not None:
                remaining = front_at - (travelled - front_travel) - target
            remaining = max(remaining, config.TILE_SIZE_CM - config.FORWARD_TOLERANCE_CM - travelled)
            remaining = min(remaining, config.TILE_SIZE_CM + config.FORWARD_TOLERANCE_CM - travelled)
            if remaining <= 0:
                break
            speed = config.DRIVE_SPEED_DPS if remaining > config.DRIVE_SLOW_CM else config.DRIVE_SLOW_DPS
            if steering.mode == "encoder":
                l, r = self.encoders
                corr = steering.update(None, None, travelled, (l - l0, r - r0))
            self.motors.set_dps(speed - corr, speed + corr)
            self.motion = {"forward_cm": round(travelled, 1)}
            self.on_change()
            time.sleep(LOOP_S)

        self.motors.stop()
        floor = self._measure_floor()
        return MoveResult(MoveStatus.OK, floor)

    def turn_left(self):
        self._turn(-1)

    def turn_right(self):
        self._turn(1)

    def align(self, front_wall: bool, back_wall: bool):
        self._begin("richtet sich aus")
        if front_wall:
            front = self.us.median("front")
            if front is not None and front < config.WALL_THRESHOLD_CM:
                self.set_action("richtet sich aus", "an der Wand vorne")
                if self._push(1):
                    self._drive_cm(-config.ALIGN_BACK_CM)
                return
        if back_wall:
            self.set_action("richtet sich aus", "an der Wand hinten")
            if self._push(-1):
                self._drive_cm(config.ALIGN_BACK_CM)

    def wait(self, seconds: float):
        self._begin("wartet")
        self.motors.stop()
        end = time.monotonic() + seconds
        while (left := end - time.monotonic()) > 0:
            self._check()
            self.detail = f"noch {left:.1f} s"
            self.on_change()
            time.sleep(0.05)

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
        reason = self._interrupt
        if reason:
            self.motors.stop()
            raise RunStopped if reason == "stop" else LackOfProgress
        if self.started_at is not None and self.time() >= config.TIME_LIMIT_S:
            self.motors.stop()
            raise TimeUp

    def _travelled(self, l0: float, r0: float) -> float:
        self.encoders = self.motors.positions()
        l, r = self.encoders
        return ((l - l0) + (r - r0)) / 2 / config.MOTOR_DEG_PER_CM

    def _measure_floor(self) -> TileType:
        self.floor = self.color.read_floor()
        self.on_change()
        return self.floor

    def _turn(self, direction: int):
        """90° drehen, direction +1 = rechts, -1 = links."""
        self._begin("dreht rechts" if direction > 0 else "dreht links")
        l0, r0 = self.motors.positions()
        while True:
            self._check()
            self.encoders = self.motors.positions()
            l, r = self.encoders
            done = direction * ((l - l0) - (r - r0)) / 2
            remaining = config.MOTOR_DEG_PER_90 - done
            if remaining <= 0:
                break
            speed = config.TURN_SPEED_DPS if remaining > config.TURN_SLOW_DEG else config.TURN_SLOW_DPS
            self.motors.set_dps(direction * speed, -direction * speed)
            self.motion = {"turn_deg": round(direction * 90 * done / config.MOTOR_DEG_PER_90)}
            self.on_change()
            time.sleep(LOOP_S)
        self.motors.stop()

    def _drive_cm(self, cm: float):
        """Gerade nach Encoder fahren (negativ = rückwärts), beide Ketten im Gleichlauf."""
        direction = 1 if cm >= 0 else -1
        l0, r0 = self.motors.positions()
        while True:
            self._check()
            done = direction * self._travelled(l0, r0)
            remaining = abs(cm) - done
            if remaining <= 0:
                break
            speed = config.DRIVE_SPEED_DPS if remaining > config.DRIVE_SLOW_CM else config.DRIVE_SLOW_DPS
            l, r = self.encoders
            corr = _clamp(config.STEER_ENC_K * ((l - l0) - (r - r0)), config.STEER_MAX_DPS)
            self.motors.set_dps(direction * speed - corr, direction * speed + corr)
            self.on_change()
            time.sleep(LOOP_S)
        self.motors.stop()

    def _push(self, direction: int) -> bool:
        """Langsam gegen eine Wand fahren, bis beide Ketten stehen (stellt den Roboter gerade).
        False = keine Wand gefunden, dann zurück an den Ausgangspunkt."""
        l0, r0 = self.motors.positions()
        self.motors.set_power_limit(config.ALIGN_POWER_LIMIT)
        try:
            start = last_t = time.monotonic()
            last = (l0, r0)
            self.motors.set_dps(direction * config.ALIGN_SPEED_DPS, direction * config.ALIGN_SPEED_DPS)
            while True:
                self._check()
                time.sleep(LOOP_S)
                moved = direction * self._travelled(l0, r0)
                if moved > config.ALIGN_MAX_CM:
                    self.motors.stop()
                    self._drive_cm(-direction * moved)
                    return False
                now = time.monotonic()
                if now - last_t >= 0.1:
                    l, r = self.encoders
                    stalled = (abs(l - last[0]) < config.ALIGN_STALL_DEG
                               and abs(r - last[1]) < config.ALIGN_STALL_DEG)
                    last, last_t = (l, r), now
                    if stalled and now - start > 0.3:
                        break
            end = time.monotonic() + config.ALIGN_PUSH_S
            while time.monotonic() < end:
                self._check()
                time.sleep(LOOP_S)
            self.motors.stop()
            return True
        finally:
            self.motors.set_power_limit(0)
