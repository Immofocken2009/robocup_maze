"""Farbsensor auf dem echten Boden kalibrieren.

Starten (im Projektordner):  python3 -m tools.calibrate_color
Den Roboter nacheinander mit dem Farbsensor auf Weiß, Schwarz, Blau, Silber und Rot stellen,
jeweils Enter drücken. Die Werte landen in data/colors.json (config.COLOR_CALIBRATION_FILE).
Danach zeigt das Programm live, welche Farbe es erkennt (beenden mit Strg+C).

Nur die Live-Anzeige mit vorhandener Kalibrierung:  python3 -m tools.calibrate_color --test
"""
import json
import sys
import time

import config
from hardware.color_sensor import ColorSensor, classify, features

NAMES = {"white": "Weiß", "black": "Schwarz", "blue": "Blau", "silver": "Silber", "red": "Rot"}
SAMPLES = 20


def average(sensor: ColorSensor) -> tuple[int, int, int, int]:
    readings = []
    for _ in range(SAMPLES):
        time.sleep(sensor.integration_s)
        readings.append(sensor.read_raw())
    return tuple(round(sum(r[i] for r in readings) / SAMPLES) for i in range(4))


def calibrate(sensor: ColorSensor) -> dict:
    colors = {}
    print("Roboter so hinstellen, dass der Farbsensor mitten über der Farbe ist.")
    print("Enter = messen, s + Enter = Farbe überspringen (Weiß und Schwarz sind Pflicht).\n")
    for name in config.COLOR_NAMES:
        while True:
            answer = input(f"{NAMES[name]}: bereit? ").strip().lower()
            if answer == "s" and name not in ("white", "black"):
                print("  übersprungen")
                break
            if answer == "":
                colors[name] = average(sensor)
                r, g, b, c = colors[name]
                print(f"  R {r}  G {g}  B {b}  C {c}")
                if c >= 0.95 * 1024 * round(config.COLOR_INTEGRATION_MS / 2.4):
                    print("  Achtung: Sensor fast übersteuert, COLOR_GAIN in config.py kleiner wählen")
                break
    return colors


def warn_close(colors: dict):
    """Farben, die der Sensor kaum unterscheiden kann."""
    white_c = colors["white"][3]
    names = list(colors)
    for i, a in enumerate(names):
        for b in names[i + 1:]:
            fa, fb = features(colors[a], white_c), features(colors[b], white_c)
            if sum((x - y) ** 2 for x, y in zip(fa, fb)) < 0.01:
                print(f"Achtung: {NAMES[a]} und {NAMES[b]} sehen fast gleich aus, eventuell neu messen.")


def live(sensor: ColorSensor):
    print("\nLive-Erkennung, Roboter über verschiedene Farben schieben. Beenden mit Strg+C.\n")
    try:
        while True:
            time.sleep(max(sensor.integration_s, 0.1))
            raw = sensor.read_raw()
            name = classify(raw, sensor.calibration)
            print(f"\r{NAMES[name]:<8}  R {raw[0]:>5}  G {raw[1]:>5}  B {raw[2]:>5}  C {raw[3]:>5}   ",
                  end="", flush=True)
    except KeyboardInterrupt:
        print()


def main():
    sensor = ColorSensor()
    try:
        if "--test" not in sys.argv:
            colors = calibrate(sensor)
            data = {"settings": {"integration_ms": config.COLOR_INTEGRATION_MS, "gain": config.COLOR_GAIN},
                    "colors": colors}
            path = config.COLOR_CALIBRATION_FILE
            path.parent.mkdir(exist_ok=True)
            path.write_text(json.dumps(data, indent=1) + "\n")
            print(f"\nGespeichert in {path.relative_to(config.PROJECT_DIR)}")
            warn_close(colors)
            sensor.calibration = {name: tuple(v) for name, v in colors.items()}
        elif not sensor.calibration:
            print(sensor.warning)
            return
        live(sensor)
    finally:
        sensor.close()


if __name__ == "__main__":
    main()
