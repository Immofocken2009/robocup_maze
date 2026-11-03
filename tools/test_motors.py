"""Motoren testen und einmessen. DER ROBOTER FÄHRT! Auf den Boden stellen, Platz lassen.

Starten (im Projektordner), in dieser Reihenfolge:
  python3 -m tools.test_motors richtung   # dreht jede Kette kurz: läuft sie vorwärts?  -> MOTOR_DIRECTION
  python3 -m tools.test_motors strecke    # fährt 60 cm nach Encoder, du misst nach     -> MOTOR_DEG_PER_CM
  python3 -m tools.test_motors drehen     # dreht 4 × 90° rechts, du schätzt den Winkel -> MOTOR_DEG_PER_90
Das Programm sagt am Ende, welcher Wert in config.py eingetragen werden soll.
Abbrechen jederzeit mit Strg+C, die Motoren gehen dann aus.
"""
import sys
import time

import config
from hardware.color_sensor import ColorSensor
from hardware.motors import Motors
from hardware.robot import RealRobot
from hardware.ultrasonic import Ultrasonic

DISTANCE_CM = 60.0


def ask_number(text: str) -> float:
    while True:
        try:
            return float(input(text).replace(",", "."))
        except ValueError:
            print("  bitte eine Zahl eingeben")


def direction(motors: Motors):
    result = {}
    for side, name in (("left", "linke"), ("right", "rechte")):
        input(f"Enter = {name} Kette 1 s drehen lassen ")
        motors.set_dps(200 if side == "left" else 0, 200 if side == "right" else 0)
        time.sleep(1)
        motors.stop()
        forward = input(f"Ist die {name} Kette vorwärts gelaufen? (j/n) ").strip().lower().startswith("j")
        result[side] = config.MOTOR_DIRECTION[side] * (1 if forward else -1)
    print(f"\nIn config.py eintragen:\nMOTOR_DIRECTION = {result}")


def distance(robot: RealRobot):
    input(f"Startpunkt markieren, Enter = {DISTANCE_CM:g} cm geradeaus fahren ")
    robot.drive_cm(DISTANCE_CM)
    measured = ask_number("Wie viele cm ist er wirklich gefahren? ")
    value = config.MOTOR_DEG_PER_CM * DISTANCE_CM / measured
    print(f"\nIn config.py eintragen:\nMOTOR_DEG_PER_CM = {value:.2f}")


def turn(robot: RealRobot):
    input("Blickrichtung merken, Enter = 4 × 90° nach rechts drehen ")
    for _ in range(4):
        robot.turn_right()
        time.sleep(0.3)
    measured = ask_number("Um wie viel Grad hat er sich insgesamt gedreht (genau eine Runde = 360)? ")
    value = config.MOTOR_DEG_PER_90 * 360 / measured
    print(f"\nIn config.py eintragen:\nMOTOR_DEG_PER_90 = {value:.0f}")


def main():
    steps = {"richtung": direction, "strecke": distance, "drehen": turn}
    if len(sys.argv) != 2 or sys.argv[1] not in steps:
        print(__doc__)
        return
    motors = Motors()
    parts = [motors]
    try:
        if sys.argv[1] == "richtung":
            direction(motors)
        else:
            us, color = Ultrasonic(), ColorSensor()
            parts += [us, color]
            steps[sys.argv[1]](RealRobot(motors, us, color))
    except KeyboardInterrupt:
        print("\nAbgebrochen")
    finally:
        for part in reversed(parts):
            part.close()


if __name__ == "__main__":
    main()
