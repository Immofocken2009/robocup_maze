"""Lüfter am Fan-Anschluss des Pi 5 (Kernel-Gerät pwmfan) und CPU-Temperatur.

Eigene Regelung in Python: Stufen nach config.FAN_STEPS mit Hysterese. Damit der Kernel nicht
dazwischenregelt, schaltet robocup-robot.service beim Start die Kernel-Regelung ab und gibt
Schreibrechte auf den Lüfter; beim Beenden oder Absturz schaltet systemd sie wieder ein.
Ohne diese Rechte (z.B. main.py von Hand gestartet) wird nur gelesen, dann regelt der Kernel.
"""
import os
from pathlib import Path

import config

THERMAL = Path("/sys/class/thermal/thermal_zone0")
HWMON = Path("/sys/class/hwmon")
COOLING = Path("/sys/class/thermal")
FULL_PWM = 255


def _read(path: Path) -> str | None:
    try:
        return path.read_text().strip()
    except OSError:
        return None


def fan_level(temp_c: float | None, level: int) -> int:
    """Neue Stufe (0 = aus) aus Temperatur und bisheriger Stufe. Hoch sofort, runter erst
    FAN_HYST_C unter der Schwelle (sonst schaltet er an der Grenze ständig hin und her)."""
    if temp_c is None:
        return len(config.FAN_STEPS)
    target = sum(1 for t, _ in config.FAN_STEPS if temp_c >= t)
    while level > target and temp_c < config.FAN_STEPS[level - 1][0] - config.FAN_HYST_C:
        level -= 1
    return max(level, target)


def fan_pwm(temp_c: float | None, level: int) -> int:
    """Leistung 0–255 für die Stufe; Lesefehler oder ab FAN_FULL_C volle Leistung."""
    if temp_c is None or temp_c >= config.FAN_FULL_C:
        return FULL_PWM
    return 0 if level == 0 else config.FAN_STEPS[level - 1][1]


class Fan:
    def __init__(self):
        self.dir = next((p for p in HWMON.glob("hwmon*") if _read(p / "name") == "pwmfan"), None)
        self.cooling = next((p for p in COOLING.glob("cooling_device*") if _read(p / "type") == "pwm-fan"), None)
        self.level = 0
        self.temp: float | None = None      # CPU-Temperatur in °C
        self.rpm: int | None = None
        self.pwm: int | None = None         # aktuelle Leistung 0–255
        self.python_controls = False        # regelt gerade Python (sonst der Kernel)
        self.warning: str | None = None

    def _may_control(self) -> bool:
        """Hat der Dienst die Kernel-Regelung abgeschaltet und uns Schreibrechte gegeben?"""
        return (self.dir is not None and os.access(self.dir / "pwm1", os.W_OK)
                and _read(THERMAL / "mode") == "disabled")

    def update(self):
        """Temperatur und Drehzahl lesen, Stufe bestimmen und, wenn erlaubt, setzen."""
        raw = _read(THERMAL / "temp")
        self.temp = int(raw) / 1000 if raw else None
        if self.dir is None:
            self.warning = "Kein Lüfter am Fan-Anschluss gefunden"
            return
        self.python_controls = self._may_control()
        if self.python_controls:
            self.level = fan_level(self.temp, self.level)
            try:
                (self.dir / "pwm1").write_text(str(fan_pwm(self.temp, self.level)))
                self.warning = None
            except OSError as exc:
                self.python_controls = False
                self.warning = f"Lüfter lässt sich nicht setzen ({exc.strerror}), der Kernel regelt"
        else:
            state = _read(self.cooling / "cur_state") if self.cooling else None
            self.level = int(state) if state else 0
            self.warning = "Lüfter: Python darf nicht regeln, der Kernel regelt (main.py über den Dienst starten)"
        rpm, pwm = _read(self.dir / "fan1_input"), _read(self.dir / "pwm1")
        self.rpm = int(rpm) if rpm else None
        self.pwm = int(pwm) if pwm else None
