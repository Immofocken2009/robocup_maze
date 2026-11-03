"""Testet die drei Ultraschallsensoren (RCWL-9610A, GPIO-Modus).

Starten (im Projektordner):  python3 -m tools.test_ultrasonic
Zeigt die Abstände live an. Hand vor die Sensoren halten, beenden mit Strg+C.

Die Messung selbst steckt in hardware/ultrasonic.py (Sensoren nacheinander,
Echo-Flanken mit Kernel-Zeitstempel, ein fehlender Sensor meldet "kein Echo").
"""
import time

import config
from hardware.ultrasonic import SIDES, Ultrasonic

NAMES = {"front": "vorne", "left": "links", "right": "rechts"}


def main(duration_s: float | None = None):
    us = Ultrasonic()
    seen = {side: False for side in SIDES}

    print("Abstände in cm (– = kein Echo / außer Reichweite). Beenden mit Strg+C.\n")
    print("".join(f"{NAMES[s]:>12}" for s in SIDES))
    start = time.monotonic()
    try:
        while duration_s is None or time.monotonic() - start < duration_s:
            row = ""
            for side, cm in us.read_all().items():
                if cm is None:
                    row += f"{'–':>12}"
                else:
                    seen[side] = True
                    row += f"{cm:12.1f}"
            print(row)
    except KeyboardInterrupt:
        pass
    finally:
        us.close()

    print("\nErgebnis:")
    for side, ok in seen.items():
        pins = config.US_PINS[side]
        state = "misst" if ok else "KEIN Echo, Verkabelung prüfen"
        print(f"  {NAMES[side]:<7} (Trig {pins['trigger']:>2}, Echo {pins['echo']:>2})  {state}")


if __name__ == "__main__":
    main()
