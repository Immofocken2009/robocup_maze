"""Zentrale Einstellungen für den Roboter.

Alles, was du an deinem Roboter messen oder einstellen musst, steht hier.
Werte mit "MESSEN!" sind Schätzungen und müssen an deinen Roboter angepasst werden.
"""
from pathlib import Path

PROJECT_DIR = Path(__file__).resolve().parent

# --- BrickPi3 Motoren -------------------------------------------------------
LEFT_MOTOR_PORT = "A"   # "A", "B", "C" oder "D"
RIGHT_MOTOR_PORT = "D"
# 1 oder -1: so einstellen, dass positive Werte das Rad VORWÄRTS drehen
# (mit tools/test_motors.py prüfen)
LEFT_MOTOR_DIRECTION = 1
RIGHT_MOTOR_DIRECTION = 1

# --- Geometrie (cm) ---------------------------------------------------------
WHEEL_DIAMETER_CM = 5.6   # MESSEN! (LEGO-Rad 56x28 = 5.6 cm)
TRACK_WIDTH_CM = 12.0     # MESSEN! Abstand zwischen den Auflagepunkten der Räder
TILE_SIZE_CM = 30.0       # Kachelgröße laut Regelwerk

# Korrekturfaktoren nach dem Kalibrieren mit tools/test_motors.py:
# fährt der Roboter zu kurz -> DRIVE_FACTOR größer machen
# dreht er zu wenig        -> TURN_FACTOR größer machen
DRIVE_FACTOR = 1.0
TURN_FACTOR = 1.0

# --- Fahrregelung -----------------------------------------------------------
DRIVE_SPEED_DPS = 360     # Radgeschwindigkeit geradeaus (Grad/Sekunde)
TURN_SPEED_DPS = 250      # Radgeschwindigkeit beim Drehen
MIN_SPEED_DPS = 80        # langsamste Geschwindigkeit beim Anfahren/Bremsen
RAMP_DEG = 150            # über so viele Radgrad wird beschleunigt/gebremst
KP_STRAIGHT = 2.0         # hält beide Räder gleich weit (geradeaus fahren)
KP_LATERAL = 4.0          # zentriert zwischen den Seitenwänden (0 = aus)
MAX_CORRECTION_DPS = 80   # maximale Lenkkorrektur
LOOP_DT_S = 0.02          # Regelschleife: 50 Hz

# --- Ultraschall HC-SR04 (BCM-GPIO-Nummern, NICHT Pin-Nummern!) -------------
# ACHTUNG: Echo liefert 5V, der Pi 5 verträgt nur 3,3V -> Spannungsteiler!
# GPIO 7-11 sind vom BrickPi3 (SPI) belegt, GPIO 2/3 vom Farbsensor (I2C).
US_FRONT = {"trigger": 23, "echo": 24}   # ANPASSEN an deine Verkabelung
US_LEFT = {"trigger": 17, "echo": 27}    # ANPASSEN
US_RIGHT = {"trigger": 5, "echo": 6}     # ANPASSEN
US_MAX_DISTANCE_CM = 150
US_QUEUE_LEN = 3          # Glättung über so viele Messungen (mehr = ruhiger, aber träger)

WALL_THRESHOLD_CM = 20.0  # gemessener Abstand kleiner als das = Wand
# Abstand Sensor -> Wand, wenn der Roboter genau in der Kachelmitte steht:
SIDE_TARGET_CM = 8.0      # MESSEN!
FRONT_TARGET_CM = 7.0     # MESSEN!

# --- Farbsensor nach unten (TCS34725 über I2C) ------------------------------
COLOR_I2C_BUS = 1
COLOR_CALIBRATION_FILE = PROJECT_DIR / "data" / "color_calibration.json"
# Schwarz erst ab dieser Strecke prüfen: Der Sensor sitzt in der Robotermitte,
# eine schwarze Kachel beginnt also frühestens nach 15 cm.
BLACK_CHECK_FROM_CM = 10.0
BLACK_CONFIRM_READINGS = 2   # so viele Messungen hintereinander schwarz = schwarz

# --- Regeln -----------------------------------------------------------------
BLUE_WAIT_S = 5.0            # Wartezeit auf blauen Kacheln (Regelwerk prüfen!)
