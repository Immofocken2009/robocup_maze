"""Ultraschallsensoren kalibrieren: je Sensor Faktor und Versatz.

Roboterdienst vorher stoppen (er belegt die Pins):  sudo systemctl stop robocup-robot
Starten (im Projektordner):
    python3 -m tools.calibrate_ultrasonic           # alle drei
    python3 -m tools.calibrate_ultrasonic links     # nur einen (vorne, links oder rechts), die anderen bleiben
    python3 call.py --ultrasonicL                   # dasselbe über call.py (L, R, V = links, rechts, vorne)
Für jeden Sensor ein ebenes Brett (mindestens ca. 20 × 15 cm) nacheinander in 5, 10, 20 und 40 cm Abstand gerade vor
den Sensor stellen, gemessen von der Vorderseite des Sensors, jeweils Enter drücken. Das Programm legt je Sensor eine
Gerade durch die Messungen (richtig = Faktor × gemessen + Versatz) und speichert sie in data/ultrasonic.json
(config.US_CALIBRATION_FILE). Prüfen danach mit:  python3 -m tools.test_ultrasonic
Abbrechen mit Strg+C, dann wird nichts gespeichert.
"""
import json
import statistics
import sys
from datetime import datetime

import lgpio

import config
from hardware.ultrasonic import NAMES, SIDES, Ultrasonic

MAX_ERROR_CM = 1.0      # liegt eine Messung weiter von der Geraden weg: Hinweis, Brett prüfen


def fit_line(points: list[tuple[float, float]]) -> tuple[float, float, float]:
    """Ausgleichsgerade durch (gemessen, richtig). Ergebnis: Faktor, Versatz, größte Abweichung in cm."""
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    mx, my = sum(xs) / len(xs), sum(ys) / len(ys)
    factor = sum((x - mx) * (y - my) for x, y in points) / sum((x - mx) ** 2 for x in xs)
    offset = my - factor * mx
    error = max(abs(factor * x + offset - y) for x, y in points)
    return factor, offset, error


def measure(us: Ultrasonic, side: str) -> tuple[float, float] | None:
    """Median und Streuung (größte minus kleinste Messung) aus US_CALIBRATION_READINGS Messungen.
    None, wenn die Mehrheit kein Echo hatte."""
    values = [us.measure(side) for _ in range(config.US_CALIBRATION_READINGS)]
    hits = [v for v in values if v is not None]
    if len(hits) * 2 <= len(values):
        return None
    return statistics.median(hits), max(hits) - min(hits)


def calibrate_side(us: Ultrasonic, side: str) -> dict:
    name = NAMES[side]
    print(f"\nSensor {name}")
    points = []
    for cm in config.US_CALIBRATION_CM:
        while True:
            input(f"  Brett {cm:g} cm vor den Sensor {name} stellen, Enter ")
            result = measure(us, side)
            if result is None:
                print("  kein Echo: Brett gerade und groß genug? Nochmal.")
                continue
            measured, spread = result
            print(f"  gemessen {measured:.1f} cm (Streuung {spread:.1f} cm)")
            points.append((measured, cm))
            break
    factor, offset, error = fit_line(points)
    print(f"  richtig = {factor:.4f} × gemessen {offset:+.2f} cm, größte Abweichung {error:.2f} cm")
    if error > MAX_ERROR_CM:
        print(f"  Achtung: mehr als {MAX_ERROR_CM:g} cm Abweichung, Brett und Abstände prüfen, evtl. wiederholen")
    return {"factor": round(factor, 5), "offset": round(offset, 3), "max_error_cm": round(error, 3),
            "points": [[round(m, 2), t] for m, t in points]}


def run(sides: list[str]):
    """Die gewählten Sensoren ("front", "left", "right") kalibrieren und speichern (auch von call.py benutzt)."""
    try:
        us = Ultrasonic(calibration_file=None)      # Rohwerte
    except lgpio.error as e:
        print(f"Ultraschall-Pins nicht frei ({e}). Läuft der Roboterdienst? Erst: sudo systemctl stop robocup-robot")
        return
    try:
        results = {side: calibrate_side(us, side) for side in sides}
    except KeyboardInterrupt:
        print("\nAbgebrochen, nichts gespeichert")
        return
    finally:
        us.close()

    path = config.US_CALIBRATION_FILE
    data = {}
    if path.exists():
        try:
            data = json.loads(path.read_text())    # die nicht neu kalibrierten Sensoren behalten
        except ValueError:
            data = {}
    data.update(results)
    data["date"] = datetime.now().isoformat(timespec="seconds")
    path.parent.mkdir(exist_ok=True)
    path.write_text(json.dumps(data, indent=2, ensure_ascii=False) + "\n")
    print(f"\nGespeichert in {path.relative_to(config.PROJECT_DIR)}")
    for side in SIDES:
        if side in data:
            d = data[side]
            print(f"  {NAMES[side]:<7} Faktor {d['factor']:.4f}  Versatz {d['offset']:+.2f} cm  "
                  f"größte Abweichung {d['max_error_cm']:.2f} cm")


def main():
    names = {name: side for side, name in NAMES.items()}
    wanted = sys.argv[1:]
    if any(w not in names for w in wanted):
        print(__doc__)
        return
    run([side for side in SIDES if NAMES[side] in wanted] if wanted else list(SIDES))


if __name__ == "__main__":
    main()
