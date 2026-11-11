"""Farbsensor auf dem echten Boden kalibrieren.

Roboterdienst vorher stoppen (er liest den Farbsensor sonst mit):  sudo systemctl stop robocup-robot
Starten (im Projektordner):
    python3 -m tools.calibrate_color                # alle Farben
    python3 -m tools.calibrate_color silber         # nur diese neu (weiß, schwarz, blau, silber, rot), die anderen bleiben
    python3 call.py --rgb [farbe ...]               # dasselbe über call.py
Den Roboter nacheinander mit dem Farbsensor auf Weiß, Schwarz, Blau, Silber und Rot stellen,
jeweils Enter drücken. Je Farbe config.COLOR_CAL_SAMPLES Messungen: gespeichert werden das Rohmittel ("colors")
und Mittel und Streuung der Merkmale ("stats") in data/colors.json (config.COLOR_CALIBRATION_FILE).
Danach zeigt das Programm live, welche Farbe es erkennt und wie weit sie in Streuungen entfernt ist
(beenden mit Strg+C).

Nur die Live-Anzeige mit vorhandener Kalibrierung:  python3 -m tools.calibrate_color --test
"""
import json
import statistics
import sys
import time
from pathlib import Path

import config
from hardware.color_sensor import ColorSensor, build_calibration, classify, features, sigma_distance

NAMES = {"white": "Weiß", "black": "Schwarz", "blue": "Blau", "silver": "Silber", "red": "Rot"}
ARGS = {"weiß": "white", "weiss": "white", "schwarz": "black", "blau": "blue", "silber": "silver", "rot": "red"}


def measure(sensor: ColorSensor) -> list[tuple[int, int, int, int]]:
    """config.COLOR_CAL_SAMPLES Messungen im Abstand der Integrationszeit."""
    readings = []
    for _ in range(config.COLOR_CAL_SAMPLES):
        time.sleep(sensor.integration_s)
        readings.append(sensor.read_raw())
    return readings


def average(readings: list) -> tuple[int, int, int, int]:
    """Rohmittel (R, G, B, C)."""
    return tuple(round(sum(r[i] for r in readings) / len(readings)) for i in range(4))


def feature_stats(readings: list, white_c: float) -> dict:
    """Mittel und Standardabweichung der vier Merkmale (`features`) über alle Messungen."""
    fs = [features(r, white_c) for r in readings]
    return {"mean": [round(statistics.fmean(f[i] for f in fs), 5) for i in range(4)],
            "std": [round(statistics.pstdev(f[i] for f in fs), 5) for i in range(4)]}


def calibrate(sensor: ColorSensor, names: list[str]) -> dict:
    """Ergebnis: {Farbe: Liste der Messungen}."""
    readings = {}
    print("Roboter so hinstellen, dass der Farbsensor mitten über der Farbe ist.")
    print("Enter = messen, s + Enter = Farbe überspringen (Weiß und Schwarz sind Pflicht).\n")
    for name in names:
        while True:
            answer = input(f"{NAMES[name]}: bereit? ").strip().lower()
            if answer == "s" and name not in ("white", "black"):
                print("  übersprungen")
                break
            if answer == "":
                readings[name] = measure(sensor)
                r, g, b, c = average(readings[name])
                print(f"  R {r}  G {g}  B {b}  C {c}")
                if c >= 0.95 * 1024 * round(config.COLOR_INTEGRATION_MS / 2.4):
                    print("  Achtung: Sensor fast übersteuert, COLOR_GAIN in config.py kleiner wählen")
                break
    return readings


def merge(old_colors: dict, old_stats: dict, readings: dict) -> tuple[dict, dict]:
    """Neue Messungen zu den alten Werten dazunehmen. Ergebnis: (Rohmittel, Merkmalsstatistik) je Farbe.
    Wird Weiß neu gemessen, ändert sich der Bezug der Helligkeit: die alte Helligkeit der anderen Farben wird umgerechnet."""
    colors = {n: tuple(v) for n, v in old_colors.items()}
    stats = {n: v for n, v in old_stats.items() if n in colors}
    old_white_c = colors["white"][3] if "white" in colors else None
    for name, rs in readings.items():
        colors[name] = average(rs)
    white_c = colors["white"][3]
    if "white" in readings and old_white_c and old_white_c != white_c:
        for name, st in stats.items():
            if name not in readings:
                k = old_white_c / white_c
                st["mean"][3] = round(min(3.0, st["mean"][3] * k), 5)   # (Kappung bei 3 wie in features, hier ungenau)
                st["std"][3] = round(st["std"][3] * k, 5)
    for name, rs in readings.items():
        stats[name] = feature_stats(rs, white_c)
    order = [n for n in config.COLOR_NAMES if n in colors]
    return {n: colors[n] for n in order}, {n: stats[n] for n in order if n in stats}


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


def saved_stats(path: Path = config.COLOR_CALIBRATION_FILE) -> dict:
    """Bisher gespeicherte Merkmalsstatistik ({} wenn keine oder unlesbar; Einstellungen prüft saved_colors)."""
    try:
        stats = json.loads(path.read_text()).get("stats", {})
    except (OSError, ValueError):
        return {}
    return {n: {"mean": list(v["mean"]), "std": list(v["std"])} for n, v in stats.items()
            if isinstance(v, dict) and "mean" in v and "std" in v}


def warn_close(stats: dict):
    """Farben, die weniger als COLOR_MIN_SEPARATION_SIGMA Streuungen auseinanderliegen
    (Streuung je Merkmal: die größere der beiden Farben, nicht unter COLOR_STD_MIN)."""
    names = list(stats)
    for i, a in enumerate(names):
        for b in names[i + 1:]:
            std = [max(x, y) for x, y in zip(stats[a]["std"], stats[b]["std"])]
            d = sigma_distance(tuple(stats[a]["mean"]), stats[b]["mean"], std)
            if d < config.COLOR_MIN_SEPARATION_SIGMA:
                print(f"Achtung: {NAMES[a]} und {NAMES[b]} liegen nur {d:.1f} Sigma auseinander "
                      f"(mindestens {config.COLOR_MIN_SEPARATION_SIGMA:g}), eventuell neu messen.")


def live(sensor: ColorSensor):
    print("\nLive-Erkennung, Roboter über verschiedene Farben schieben. Beenden mit Strg+C.\n")
    try:
        while True:
            time.sleep(max(sensor.integration_s, 0.1))
            raw = sensor.read_raw()
            name = classify(raw, sensor.calibration)
            print(f"\r{NAMES.get(name, 'unsicher'):<8}  R {raw[0]:>5}  G {raw[1]:>5}  B {raw[2]:>5}  C {raw[3]:>5}   ",
                  end="", flush=True)
    except KeyboardInterrupt:
        print()


def run(names: list[str] | None = None, test_only: bool = False):
    """Farben einmessen und speichern, danach Live-Erkennung (auch von call.py benutzt).
    names: nur diese Farben neu ("white", ...), die anderen bleiben; None = alle."""
    path = config.COLOR_CALIBRATION_FILE
    old, old_stats = {}, {}
    if names and not test_only:
        old = saved_colors(path)
        if old is None:
            print("Noch keine passende Kalibrierung (fehlt oder mit anderen Einstellungen gemessen), "
                  "bitte alle Farben einmessen: python3 call.py --rgb")
            return
        old_stats = saved_stats(path)
    sensor = ColorSensor()
    try:
        if not test_only:
            try:
                readings = calibrate(sensor, [n for n in config.COLOR_NAMES if not names or n in names])
            except KeyboardInterrupt:
                print("\nAbgebrochen, nichts gespeichert")
                return
            if "white" not in readings and "white" not in old:
                print("Ohne Weiß keine Kalibrierung, nichts gespeichert")
                return
            colors, stats = merge(old, old_stats, readings)
            data = {"settings": {"integration_ms": config.COLOR_INTEGRATION_MS, "gain": config.COLOR_GAIN},
                    "colors": colors, "stats": stats}
            path.parent.mkdir(exist_ok=True)
            path.write_text(json.dumps(data, indent=1) + "\n")
            print(f"\nGespeichert in {path.relative_to(config.PROJECT_DIR) if path.is_relative_to(config.PROJECT_DIR) else path}"
                  + (f" (neu: {', '.join(NAMES[n] for n in readings)})" if names else ""))
            warn_close(stats)
            sensor.calibration = build_calibration(colors, stats)
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
