"""Farbsensor auf dem echten Boden kalibrieren.

Roboterdienst vorher stoppen (er liest den Farbsensor sonst mit):  sudo systemctl stop robocup-robot
Starten (im Projektordner):
    python3 -m tools.calibrate_color                # alle Farben
    python3 -m tools.calibrate_color silber         # nur diese neu (weiß, schwarz, blau, silber, rot), die anderen bleiben
    python3 call.py --rgb [farbe ...]               # dasselbe über call.py
Den Roboter nacheinander mit dem Farbsensor auf Weiß, Schwarz, Blau, Silber und Rot stellen,
jeweils Enter drücken. Die Werte landen in data/colors.json (config.COLOR_CALIBRATION_FILE).
Danach zeigt das Programm live, welche Farbe es erkennt (beenden mit Strg+C).

Nur die Live-Anzeige mit vorhandener Kalibrierung:  python3 -m tools.calibrate_color --test
"""
import json
import sys
import time
from pathlib import Path

import config
from hardware.color_sensor import ColorSensor, classify, features

NAMES = {"white": "Weiß", "black": "Schwarz", "blue": "Blau", "silver": "Silber", "red": "Rot"}
ARGS = {"weiß": "white", "weiss": "white", "schwarz": "black", "blau": "blue", "silber": "silver", "rot": "red"}
SAMPLES = 20


def average(sensor: ColorSensor) -> tuple[int, int, int, int]:
    readings = []
    for _ in range(SAMPLES):
        time.sleep(sensor.integration_s)
        readings.append(sensor.read_raw())
    return tuple(round(sum(r[i] for r in readings) / SAMPLES) for i in range(4))


def calibrate(sensor: ColorSensor, names: list[str]) -> dict:
    colors = {}
    print("Roboter so hinstellen, dass der Farbsensor mitten über der Farbe ist.")
    print("Enter = messen, s + Enter = Farbe überspringen (Weiß und Schwarz sind Pflicht).\n")
    for name in names:
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


def saved_colors(path: Path = config.COLOR_CALIBRATION_FILE) -> dict | None:
    """Bisher gespeicherte Farben, wenn sie mit den jetzigen Einstellungen gemessen wurden, sonst None."""
    try:
        data = json.loads(path.read_text())
    except (OSError, ValueError):
        return None
    settings = data.get("settings", {})
    if settings.get("integration_ms") != config.COLOR_INTEGRATION_MS or settings.get("gain") != config.COLOR_GAIN:
        return None
    return {name: tuple(values) for name, values in data.get("colors", {}).items()}


def warn_close(colors: dict):
    """Farben, die der Sensor kaum unterscheiden kann."""
    if "white" not in colors:
        return
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


def run(names: list[str] | None = None, test_only: bool = False):
    """Farben einmessen und speichern, danach Live-Erkennung (auch von call.py benutzt).
    names: nur diese Farben neu ("white", ...), die anderen bleiben; None = alle."""
    path = config.COLOR_CALIBRATION_FILE
    old = {}
    if names and not test_only:
        old = saved_colors(path)
        if old is None:
            print("Noch keine passende Kalibrierung (fehlt oder mit anderen Einstellungen gemessen), "
                  "bitte alle Farben einmessen: python3 call.py --rgb")
            return
    sensor = ColorSensor()
    try:
        if not test_only:
            try:
                new = calibrate(sensor, [n for n in config.COLOR_NAMES if not names or n in names])
            except KeyboardInterrupt:
                print("\nAbgebrochen, nichts gespeichert")
                return
            colors = {n: new.get(n, old.get(n)) for n in config.COLOR_NAMES if n in new or n in old}
            data = {"settings": {"integration_ms": config.COLOR_INTEGRATION_MS, "gain": config.COLOR_GAIN},
                    "colors": colors}
            path.parent.mkdir(exist_ok=True)
            path.write_text(json.dumps(data, indent=1) + "\n")
            print(f"\nGespeichert in {path.relative_to(config.PROJECT_DIR)}"
                  + (f" (neu: {', '.join(NAMES[n] for n in new)})" if names else ""))
            warn_close(colors)
            sensor.calibration = {name: tuple(v) for name, v in colors.items()}
        elif not sensor.calibration:
            print(sensor.warning)
            return
        live(sensor)
    finally:
        sensor.close()


def main():
    args = [a for a in sys.argv[1:] if a != "--test"]
    if any(a.lower() not in ARGS for a in args):
        print(__doc__)
        return
    run([ARGS[a.lower()] for a in args] or None, test_only="--test" in sys.argv)


if __name__ == "__main__":
    main()
