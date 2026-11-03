"""Testet die drei Ultraschallsensoren (RCWL-9610A, GPIO-Modus).

Starten (im Projektordner):  python3 -m tools.test_ultrasonic
Zeigt die Abstände live an. Hand vor die Sensoren halten, beenden mit Strg+C.

Die Sensoren werden nacheinander abgefragt (stören sich so nicht gegenseitig).
Die Echo-Flanken werden vom Kernel mit Zeitstempel erfasst (lgpio), ein fehlender
Sensor meldet nach US_ECHO_TIMEOUT_S einfach "kein Echo".
"""
import threading
import time

import lgpio

import config

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


def measure_cm(chip: int, trigger: int, echo: Echo) -> float | None:
    echo.arm()
    lgpio.gpio_write(chip, trigger, 1)      # HIGH-Puls länger als 10 µs
    t = time.perf_counter()
    while time.perf_counter() - t < 15e-6:
        pass
    lgpio.gpio_write(chip, trigger, 0)
    if not echo.wait(config.US_ECHO_TIMEOUT_S):
        return None
    cm = echo.width_ns / 1e9 * config.SPEED_OF_SOUND_M_S / 2 * 100
    return cm if cm <= config.US_MAX_DISTANCE_CM else None


def main(duration_s: float | None = None):
    chip = lgpio.gpiochip_open(config.GPIO_CHIP)
    echos = {}
    for side, pins in config.US_PINS.items():
        lgpio.gpio_claim_output(chip, pins["trigger"], 0)
        echos[side] = Echo(chip, pins["echo"])
    seen = {side: False for side in echos}

    print("Abstände in cm (– = kein Echo / außer Reichweite). Beenden mit Strg+C.\n")
    print("".join(f"{NAMES[s]:>12}" for s in echos))
    start = time.monotonic()
    try:
        while duration_s is None or time.monotonic() - start < duration_s:
            row = ""
            for side, echo in echos.items():
                cm = measure_cm(chip, config.US_PINS[side]["trigger"], echo)
                if cm is None:
                    row += f"{'–':>12}"
                else:
                    seen[side] = True
                    row += f"{cm:12.1f}"
                time.sleep(config.US_CYCLE_S)
            print(row)
    except KeyboardInterrupt:
        pass
    finally:
        for echo in echos.values():
            echo.close()
        lgpio.gpiochip_close(chip)

    print("\nErgebnis:")
    for side, ok in seen.items():
        pins = config.US_PINS[side]
        state = "misst" if ok else "KEIN Echo, Verkabelung prüfen"
        print(f"  {NAMES[side]:<7} (Trig {pins['trigger']:>2}, Echo {pins['echo']:>2})  {state}")


if __name__ == "__main__":
    main()
