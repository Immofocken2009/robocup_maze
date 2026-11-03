"""Testet die drei Ultraschallsensoren (RCWL-9610A, GPIO-Modus).

Starten (im Projektordner):  python3 -m tools.test_ultrasonic
Zeigt die Abstände live an. Hand vor die Sensoren halten, beenden mit Strg+C.
"""
import time
import warnings

from gpiozero import DistanceSensor
from gpiozero.exc import DistanceSensorNoEcho

import config

NAMES = {"front": "vorne", "left": "links", "right": "rechts"}


def main(duration_s: float | None = None):
    warnings.simplefilter("ignore", DistanceSensorNoEcho)
    sensors = {
        side: DistanceSensor(echo=pins["echo"], trigger=pins["trigger"],
                             max_distance=config.US_MAX_DISTANCE_CM / 100, queue_len=3)
        for side, pins in config.US_PINS.items()
    }
    seen = {side: False for side in sensors}
    print("Abstände in cm (– = kein Echo / außer Reichweite). Beenden mit Strg+C.\n")
    print("   " + "".join(f"{NAMES[s]:>14}" for s in sensors))
    start = time.monotonic()
    try:
        while duration_s is None or time.monotonic() - start < duration_s:
            row = ""
            for side, sensor in sensors.items():
                cm = sensor.distance * 100
                if cm < config.US_MAX_DISTANCE_CM - 1:
                    seen[side] = True
                    row += f"{cm:14.1f}"
                else:
                    row += f"{'–':>14}"
            print("   " + row)
            time.sleep(0.3)
    except KeyboardInterrupt:
        pass

    print("\nErgebnis:")
    for side, ok in seen.items():
        pins = config.US_PINS[side]
        state = "misst" if ok else "KEIN Echo, Verkabelung prüfen"
        print(f"  {NAMES[side]:<7} (Trig {pins['trigger']:>2}, Echo {pins['echo']:>2})  {state}")
    for sensor in sensors.values():
        sensor.close()


if __name__ == "__main__":
    main()
