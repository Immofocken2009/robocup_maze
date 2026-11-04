"""Programme des Roboters aufrufen.

Starten (im Projektordner):
    python3 call.py --ultrasonicL     # linken Ultraschall kalibrieren
    python3 call.py --ultrasonicR     # rechten
    python3 call.py --ultrasonicV     # vorderen
Mehrere zusammen gehen auch, z.B. --ultrasonicL --ultrasonicR. Das Kalibrieren selbst steckt in
tools/calibrate_ultrasonic.py (Ablauf siehe dort); die anderen Sensoren bleiben, wie sie kalibriert sind.
Roboterdienst vorher stoppen:  sudo systemctl stop robocup-robot
"""
import argparse

from tools import calibrate_ultrasonic

ULTRASONIC = {"ultrasonicV": "front", "ultrasonicL": "left", "ultrasonicR": "right"}
NAMES = {"front": "vorderen", "left": "linken", "right": "rechten"}


def main():
    parser = argparse.ArgumentParser(description="Programme des Roboters aufrufen.")
    for flag, side in ULTRASONIC.items():
        parser.add_argument(f"--{flag}", action="store_true", help=f"{NAMES[side]} Ultraschall kalibrieren")
    args = parser.parse_args()
    sides = [side for flag, side in ULTRASONIC.items() if getattr(args, flag)]
    if not sides:
        parser.print_help()
        return
    calibrate_ultrasonic.run(sides)


if __name__ == "__main__":
    main()
