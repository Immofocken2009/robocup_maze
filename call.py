"""Programme des Roboters aufrufen.

Starten (im Projektordner, ohne sudo):
    python3 call.py --ultrasonicL     # linken Ultraschall kalibrieren
    python3 call.py --ultrasonicR     # rechten
    python3 call.py --ultrasonicV     # vorderen
    python3 call.py --rgb             # Farbsensor: alle Farben einmessen (Weiß, Schwarz, Blau, Silber, Rot)
    python3 call.py --rgb silber rot  # nur diese Farben neu, die anderen bleiben
Mehrere Ultraschall-Seiten zusammen gehen auch, z.B. --ultrasonicL --ultrasonicR; was nicht neu eingemessen wird,
bleibt. Das Kalibrieren selbst steckt in tools/calibrate_ultrasonic.py und tools/calibrate_color.py.
Roboterdienst vorher stoppen (er benutzt dieselben Sensoren):  sudo systemctl stop robocup-robot
"""
import argparse
import subprocess

from tools import calibrate_color, calibrate_ultrasonic

ULTRASONIC = {"ultrasonicV": "front", "ultrasonicL": "left", "ultrasonicR": "right"}
NAMES = {"front": "vorderen", "left": "linken", "right": "rechten"}


def service_running() -> bool:
    """Läuft der Roboterdienst? Er benutzt dieselben Sensoren."""
    try:
        return subprocess.run(["systemctl", "is-active", "--quiet", "robocup-robot"]).returncode == 0
    except OSError:
        return False


def main():
    parser = argparse.ArgumentParser(description="Programme des Roboters aufrufen.")
    for flag, side in ULTRASONIC.items():
        parser.add_argument(f"--{flag}", action="store_true", help=f"{NAMES[side]} Ultraschall kalibrieren")
    parser.add_argument("--rgb", nargs="*", metavar="FARBE", type=str.lower,
                        choices=sorted(calibrate_color.ARGS),
                        help="Farbsensor einmessen, ohne Farbe alle, sonst nur die genannten")
    args = parser.parse_args()
    sides = [side for flag, side in ULTRASONIC.items() if getattr(args, flag)]
    if not sides and args.rgb is None:
        parser.print_help()
        return
    if service_running():
        print("Der Roboterdienst läuft und benutzt die Sensoren. Erst: sudo systemctl stop robocup-robot")
        return
    if sides:
        calibrate_ultrasonic.run(sides)
    if args.rgb is not None:
        calibrate_color.run([calibrate_color.ARGS[a] for a in args.rgb] or None)


if __name__ == "__main__":
    main()
