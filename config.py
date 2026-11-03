"""Zentrale Einstellungen.

Werte mit "MESSEN!" sind Schätzungen und müssen an den echten Roboter angepasst werden.
"""
from pathlib import Path

PROJECT_DIR = Path(__file__).resolve().parent

# --- Spielfeld ---------------------------------------------------------------
TILE_SIZE_CM = 30.0

# --- Roboter-Maße (cm) -------------------------------------------------------
ROBOT_LENGTH_CM = 18.0    # MESSEN!
ROBOT_WIDTH_CM = 16.0     # MESSEN!

# Einbau der Sensoren
US_FRONT_INSET_CM = 3.0   # vorderer Ultraschall: mittig, 3 cm hinter der Vorderkante
US_SIDE_INSET_CM = 2.0    # seitliche Ultraschall: in der Längsmitte, 2 cm von der Seitenkante nach innen
COLOR_INSET_CM = 2.0      # Farbsensor: vorne mittig, 2 cm hinter der Vorderkante

# Abstand Sensor–Wand, wenn der Roboter mittig auf der Kachel steht
WALL_THICKNESS_CM = 2.0   # Regel 3.1: Durchgang 28 cm bei 30-cm-Kacheln, Wand ragt 1 cm in jede Kachel
FRONT_CENTER_CM = (TILE_SIZE_CM - WALL_THICKNESS_CM) / 2 - (ROBOT_LENGTH_CM / 2 - US_FRONT_INSET_CM)
SIDE_CENTER_CM = (TILE_SIZE_CM - WALL_THICKNESS_CM) / 2 - (ROBOT_WIDTH_CM / 2 - US_SIDE_INSET_CM)

# --- Taster (BCM-GPIO-Nummern) -----------------------------------------------
# 4er-Tastermodul (GND, K1-K4): beim Drücken wird der Pin mit GND verbunden -> interne Pull-ups
BUTTON_PINS = {"K1": 17, "K2": 27, "K3": 22, "K4": 23}
BUTTON_FUNCTIONS = {"start": "K1", "stop": "K2", "lop": "K3"}   # K4 frei

# --- Ultraschall -------------------------------------------------------------
# 3× RCWL-9610A im GPIO-Modus (Trig/Echo wie HC-SR04), Versorgung 3,3 V (BCM-GPIO-Nummern)
US_PINS = {
    "front": {"trigger": 5, "echo": 6},
    "left": {"trigger": 26, "echo": 20},
    "right": {"trigger": 19, "echo": 16},
}
US_MAX_DISTANCE_CM = 150.0  # weiter weg = "nichts gemessen"
US_ECHO_TIMEOUT_S = 0.05    # so lange auf das Echo warten, sonst "kein Echo"
US_CYCLE_S = 0.08           # Pause zwischen zwei Messungen (Messzyklus RCWL-9610A ~70 ms)
SPEED_OF_SOUND_M_S = 343.0  # bei ca. 20 °C
GPIO_CHIP = 0               # gpiochip0 = GPIO-Leiste des Pi 5 (pinctrl-rp1)
WALL_THRESHOLD_CM = 20.0    # Karte: in der Kachelmitte gemessener Abstand kleiner als das = Wand
US_READINGS = 3             # so viele Messungen pro Sensor, wenn im Stand Wände gemessen werden (Median)

# --- Farbsensor (TCS34725) -----------------------------------------------------
COLOR_I2C_BUS = 1
COLOR_I2C_ADDRESS = 0x29
COLOR_INTEGRATION_MS = 24.0     # Messdauer, 2,4 ms Schritte
COLOR_GAIN = 60                 # 1, 4, 16 oder 60
COLOR_CALIBRATION_FILE = PROJECT_DIR / "data" / "colors.json"   # tools/calibrate_color.py
COLOR_NAMES = ("white", "black", "blue", "silver", "red")
COLOR_STOP_COUNT = 2            # so viele Messungen hintereinander schwarz/rot, bevor angehalten wird

# --- Motoren (BrickPi3) --------------------------------------------------------
MOTOR_PORTS = {"left": "A", "right": "D"}
MOTOR_DIRECTION = {"left": 1, "right": 1}   # -1, wenn die Kette bei positiver Drehzahl rückwärts läuft. MESSEN!
MOTOR_DEG_PER_CM = 30.0     # Encodergrad pro cm Fahrstrecke. MESSEN! (tools.test_motors)
MOTOR_DEG_PER_90 = 450.0    # Encodergrad je Kette für eine 90°-Drehung auf der Stelle. MESSEN!

DRIVE_SPEED_DPS = 400       # Geradeaus (Grad pro Sekunde am Motor)
DRIVE_SLOW_DPS = 150        # kurz vor dem Ziel
DRIVE_SLOW_CM = 5.0         # so viele cm vor dem Ziel langsam fahren
TURN_SPEED_DPS = 300
TURN_SLOW_DPS = 100
TURN_SLOW_DEG = 60.0        # so viele Encodergrad vor dem Ziel langsam drehen

FRONT_TRUST_CM = 100.0      # Wand vorne weiter weg als das: nur nach Encoder fahren
STALL_S = 1.0               # Ketten drehen sich trotz Fahrbefehl so lange nicht: blockiert
STALL_DEG = 10.0            # weniger Encoder-Bewegung (beide Ketten zusammen) gilt als Stillstand
FORWARD_TOLERANCE_CM = 8.0  # der Abstand vorne darf die Kachelstrecke um höchstens so viel ändern

# PD-Regler beim Geradeausfahren. In der Fahrphysik-Simulation mit Rauschen/Schlupf vorab eingestellt,
# auf dem echten Feld nachstellen (Simulation kennt die echten Robotermaße noch nicht).
STEER_KP = 10.0             # Lenkkorrektur (dps) pro cm Abweichung von der Sollposition
STEER_KD = 300.0            # Lenkkorrektur (dps) pro cm Abweichung je cm Fahrt (Schrägstand)
STEER_D_WINDOW_CM = 4.0     # Schrägstand über so viele cm Fahrt bestimmen
STEER_ENC_K = 2.0           # ohne Seitenwand: Lenkkorrektur (dps) pro Encodergrad Unterschied
STEER_MAX_DPS = 120.0
STEER_SLEW_DPS_PER_CM = 40.0    # Lenkkorrektur ändert sich pro gefahrenem cm höchstens um so viel
STEER_WIDTH_JUMP_CM = 1.0       # Breiten-Wächter: links + rechts weicht um mehr von B ab -> Seite ignorieren
                                # (Fehlalarm ist harmlos: dann kurz nur an einer Wand)
STEER_STEADY_CM = 1.5           # zurückkehrende Wand: 2 Messungen hintereinander mit höchstens so viel Unterschied
STEER_SIDE_MAX_CM = 55.0        # Seitenwand weiter weg: zum Lenken zu ungenau
CORRIDOR_CM = 2 * SIDE_CENTER_CM                    # Startwert Gangbreite B = links + rechts (wird gemessen)
CORRIDOR_MAX_CM = CORRIDOR_CM + TILE_SIZE_CM / 2    # links + rechts größer = breiter Raum, nähere Wand zählt

# --- Regeln (Regelwerk 2026) -------------------------------------------------
BLUE_WAIT_S = 5.0           # auf blauer Kachel 5 s am Stück stehen (3.2.5)
TIME_LIMIT_S = 8 * 60       # 8 Minuten pro Spiel inkl. Kalibrieren (5.3.1)
CALIBRATION_TIME_S = 40.0   # so viel der 8 Minuten ist beim Start schon verbraucht
EXIT_SIGNAL_S = 10.0        # am Start blinken für den Exit-Bonus (5.6.12)
TIME_MARGIN_S = 20.0        # Sicherheitspuffer für den Heimweg

# Punkte (5.6)
POINTS_CHECKPOINT = 10
POINTS_BLUE = 30            # blaue Kachel beim ersten Besuch
POINTS_BLUE_REVISIT = 10    # Abzug je weiterem Besuch derselben blauen Kachel
BONUS_PER_SBV = 10          # Zuverlässigkeits- und Exit-Bonus je blauer Kachel, die genau einmal besucht wurde
PENALTY_PER_LOP = 15        # Abzug im Zuverlässigkeitsbonus je Lack of Progress

# --- Gefahrenzone ------------------------------------------------------------
RED_MODES = ("zuletzt", "meiden", "normal")
RED_MODE = "zuletzt"        # Standard, auf der Webseite vor jedem Lauf wählbar

# --- Wegplanung (Kosten für Dijkstra) ----------------------------------------
PLAN_COST_FORWARD = 1.0
PLAN_COST_TURN = 1.0            # pro 90°-Drehung
PLAN_COST_BLUE_REVISIT = 50.0   # bereits besuchte blaue Kachel erneut befahren: stark meiden

# --- Geschätzte Dauer der Aktionen (Sim-Zeit und Heimweg-Schätzung) ----------
MOVE_TIME_S = 3.5           # eine Kachel vorwärts inkl. Messen (Fahrphysik: 3,2 s + Wände 0,6 s). MESSEN!
TURN_TIME_S = 2.0           # eine 90°-Drehung (Fahrphysik: 1,9 s). MESSEN!
BACKOFF_TIME_S = 2.0        # schwarze/rote Kachel erkannt, anhalten und zurück. MESSEN!

# --- Zufallslabyrinth --------------------------------------------------------
FIELD_SIZES_2026 = [(8, 7), (8, 8), (9, 8), (7, 8), (4, 12)]   # offizielle Felder 2026
GENERATOR_DEFAULTS = {"width": 8, "height": 7, "black": 2, "blue": 2, "silver": 3, "zones": 1}
GENERATOR_MAX_SIZE = 16

# --- Webseite ----------------------------------------------------------------
WEB_HOST = "0.0.0.0"        # 0.0.0.0 = auch von anderen Geräten im Netz erreichbar
WEB_PORT = 8000

# --- Lüfter (Fan-Anschluss des Pi 5), eigene Regelung in hardware/fan.py -------------
FAN_STEPS = [(45.0, 75), (52.0, 125), (60.0, 175), (68.0, 250)]   # ab °C -> Leistung (0–255)
FAN_HYST_C = 5.0            # erst so viel unter der Schwelle eine Stufe zurück
FAN_FULL_C = 75.0           # ab hier (und bei Lesefehler) volle Leistung
FAN_INTERVAL_S = 2.0        # so oft regeln
CPU_HOT_C = 75.0            # darüber Warnung auf der Webseite

# --- Roboterprogramm (main.py) -------------------------------------------------
ROBOT_LINK_URL = f"ws://localhost:{WEB_PORT}/robot"   # dorthin schickt main.py seine Daten
ROBOT_LINK_RETRY_S = 2.0    # Webseite nicht erreichbar: so oft neu versuchen
ROBOT_PUBLISH_S = 0.1       # höchstens so oft Daten an die Webseite schicken
IDLE_SENSOR_S = 0.3         # im Stand die Sensoren so oft lesen (für die Anzeige)
HARDWARE_RETRY_S = 5.0      # Hardware antwortet nicht: so oft neu versuchen
BATTERY_LOW_V = 9.0         # darunter Warnung auf der Webseite
BATTERY_READ_S = 1.0        # Akkuspannung so oft lesen
