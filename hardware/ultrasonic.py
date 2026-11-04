"""Drei Ultraschallsensoren (RCWL-9610A im GPIO-Modus, Trig/Echo wie HC-SR04).

Die Sensoren werden nacheinander ausgelöst (stören sich so nicht gegenseitig), zwischen zwei
Messungen liegen mindestens config.US_CYCLE_S. Die Echo-Flanken erfasst der Kernel mit
Zeitstempel (lgpio). Kommt kein Echo, liefert die Messung nach US_ECHO_TIMEOUT_S None.
Ist eine Kalibrierung gespeichert (data/ultrasonic.json, tools/calibrate_ultrasonic.py), wird jede Messung je Sensor
korrigiert: richtig = Faktor × gemessen + Versatz (Abstand von der Vorderseite des Sensors).
"""
import json
import statistics
import threading
import time
from pathlib import Path

import lgpio

import config

SIDES = ("front", "left", "right")
NAMES = {"front": "vorne", "left": "links", "right": "rechts"}


class Echo:
    """Misst die Länge des Echo-Pulses an einem Pin."""

    def __init__(self, chip: int, pin: int):
        self.width_ns: int | None = None
        self._up: int | None = None
        self._done = threading.Event()
        lgpio.gpio_claim_alert(chip, pin, lgpio.BOTH_EDGES)
        self._callback = lgpio.callback(chip, pin, lgpio.BOTH_EDGES, self._edge)

    def _edge(self, _chip, _gpio, level, tick):
        if level == 1:
            self._up = tick
        elif level == 0 and self._up is not None:
            self.width_ns = tick - self._up
            self._done.set()

    def arm(self):
        self._up = None
        self.width_ns = None
        self._done.clear()

    def wait(self, timeout: float) -> bool:
        return self._done.wait(timeout)

    def close(self):
        self._callback.cancel()


class Ultrasonic:
    def __init__(self, calibration_file: Path | None = config.US_CALIBRATION_FILE):
        """calibration_file=None: ohne Kalibrierung messen (Rohwerte, zum Kalibrieren selbst)."""
        self.calibration, self.warning = load_calibration(calibration_file)
        self.chip = lgpio.gpiochip_open(config.GPIO_CHIP)
        self.echos: dict[str, Echo] = {}
        for side in SIDES:
            pins = config.US_PINS[side]
            lgpio.gpio_claim_output(self.chip, pins["trigger"], 0)
            self.echos[side] = Echo(self.chip, pins["echo"])
        self._last_ping = 0.0
        self.last: dict[str, float | None] = {side: None for side in SIDES}   # letzte Messung je Sensor
        self.last_echo_ns: int | None = None    # Pulsbreite der letzten Messung (für die Aufzeichnung)

    def ready(self) -> bool:
        """Darf jetzt ohne Warten gemessen werden?"""
        return time.monotonic() - self._last_ping >= config.US_CYCLE_S

    def measure(self, side: str) -> float | None:
        """Eine Messung in cm, None = kein Echo / außer Reichweite."""
        pause = config.US_CYCLE_S - (time.monotonic() - self._last_ping)
        if pause > 0:
            time.sleep(pause)
        trigger, echo = config.US_PINS[side]["trigger"], self.echos[side]
        echo.arm()
        self._last_ping = time.monotonic()
        lgpio.gpio_write(self.chip, trigger, 1)      # HIGH-Puls länger als 10 µs
        t = time.perf_counter()
        while time.perf_counter() - t < 15e-6:
            pass
        lgpio.gpio_write(self.chip, trigger, 0)
        cm = None
        self.last_echo_ns = None
        if echo.wait(config.US_ECHO_TIMEOUT_S):
            self.last_echo_ns = echo.width_ns
            cm = echo.width_ns / 1e9 * config.SPEED_OF_SOUND_M_S / 2 * 100
            if self.calibration and side in self.calibration:
                factor, offset = self.calibration[side]
                cm = factor * cm + offset
            if cm > config.US_MAX_DISTANCE_CM:
                cm = None
        self.last[side] = cm
        return cm

    def median(self, side: str, n: int = config.US_READINGS) -> float | None:
        """Median aus n Messungen. None, wenn die Mehrheit kein Echo hatte."""
        values = [self.measure(side) for _ in range(n)]
        hits = [v for v in values if v is not None]
        if len(hits) * 2 <= n:
            self.last[side] = None
            return None
        self.last[side] = statistics.median(hits)
        return self.last[side]

    def read_all(self, n: int = 1) -> dict[str, float | None]:
        if n == 1:
            return {side: self.measure(side) for side in SIDES}
        return {side: self.median(side, n) for side in SIDES}

    def close(self):
        for echo in self.echos.values():
            echo.close()
        lgpio.gpiochip_close(self.chip)


def load_calibration(path: Path | None = config.US_CALIBRATION_FILE) -> tuple[dict[str, tuple[float, float]] | None,
                                                                              str | None]:
    """Kalibrierung laden: je Sensor (Faktor, Versatz). Ergebnis: (Werte, Warnung). Fehlt ein Sensor, misst er dort
    ohne Korrektur."""
    if path is None:
        return None, None
    if not path.exists():
        return None, "Ultraschall nicht kalibriert (python3 -m tools.calibrate_ultrasonic)"
    try:
        data = json.loads(path.read_text())
        values = {side: (float(data[side]["factor"]), float(data[side]["offset"])) for side in SIDES if side in data}
    except (ValueError, KeyError, TypeError, AttributeError):
        return None, f"Ultraschall-Kalibrierung unlesbar ({path.name}), misst ohne Korrektur"
    missing = [NAMES[side] for side in SIDES if side not in values]
    if missing:
        return values, f"Ultraschall nicht kalibriert: {', '.join(missing)} (python3 -m tools.calibrate_ultrasonic)"
    return values, None
