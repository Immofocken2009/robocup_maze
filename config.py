"""Zentrale Einstellungen.

Werte mit "MESSEN!" sind Schätzungen und müssen an den echten Roboter angepasst werden.
"""
import json
import os
from pathlib import Path

PROJECT_DIR = Path(__file__).resolve().parent

# --- Spielfeld ---------------------------------------------------------------
TILE_SIZE_CM = 30.0

# --- Roboter-Maße (cm) -------------------------------------------------------
# Außen über alles (mit überstehenden Teilen), gemessen 2026-11-04. Ketten mittig unter dem Roboter
# (Drehmitte = Mitte), Kettenmitten 15 cm auseinander.
ROBOT_LENGTH_CM = 16.5
ROBOT_WIDTH_CM = 19.5

# Einbau der Sensoren (Vorderkante = vorderster Punkt = Vorderseite des vorderen Ultraschalls)
US_FRONT_INSET_CM = 0.0   # vorderer Ultraschall: mittig, ragt 1 cm über das Gehäuse (vorderster Punkt)
US_SIDE_INSET_CM = 2.0    # seitliche Ultraschall: 2 cm von der Seitenkante nach innen ...
US_SIDE_BACK_CM = 1.0     # ... und 1 cm hinter der Vorderkante
US_SIDE_AHEAD_CM = ROBOT_LENGTH_CM / 2 - US_SIDE_BACK_CM   # so weit sitzen sie vor der Drehmitte
COLOR_INSET_CM = 1.5      # Farbsensor: vorne mittig, 1,5 cm hinter der Vorderkante

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
US_BLIND_WARN = "Alle drei Ultraschallsensoren ohne Echo: Versorgung (3,3 V, GND) und Kabel prüfen"
US_BLIND_STOP = 2           # so viele Wandmessungen hintereinander ganz ohne Echo (alle drei Sensoren) = Lauf anhalten
US_CALIBRATION_FILE = PROJECT_DIR / "data" / "ultrasonic.json"   # tools/calibrate_ultrasonic.py
US_CALIBRATION_CM = (5.0, 10.0, 20.0, 40.0)  # Kalibrieren: Brett in diesen Abständen vor die Sensorfläche
US_CALIBRATION_READINGS = 15                 # Kalibrieren: so viele Messungen je Abstand (Median)

# --- Farbsensor (TCS34725) -----------------------------------------------------
COLOR_I2C_BUS = 1
COLOR_I2C_ADDRESS = 0x29
COLOR_INTEGRATION_MS = 24.0     # Messdauer, 2,4 ms Schritte
COLOR_GAIN = 60                 # 1, 4, 16 oder 60
COLOR_CALIBRATION_FILE = PROJECT_DIR / "data" / "colors.json"   # tools/calibrate_color.py
COLOR_NAMES = ("white", "black", "blue", "silver", "red")
COLOR_STOP_COUNT = 2            # so viele Messungen hintereinander schwarz/rot, bevor angehalten wird

# --- Motoren (BrickPi3) --------------------------------------------------------
MOTOR_PORTS = {"left": "B", "right": "C"}   # geprüft 2026-11-04
MOTOR_DIRECTION = {"left": -1, "right": -1}  # -1, wenn die Kette bei positiver Drehzahl rückwärts läuft. Geprüft 2026-11-04
MOTOR_FLOAT_CHECK_S = 0.01  # Motoren stromlos: so lange warten, dann nachlesen, ob beide es melden
MOTOR_FLOAT_TRIES = 5       # so oft höchstens schicken (einzelne Nachrichten an den BrickPi3 gehen verloren)
MOTOR_DEG_PER_CM = 18.29    # Encodergrad pro cm Fahrstrecke. Eingemessen 2026-11-04 (1802 Grad = 98,5 cm)
MOTOR_DEG_PER_90 = 345.0    # Encodergrad je Kette für eine 90°-Drehung auf der Stelle. Eingemessen und bestätigt 2026-11-04 (360° am Stück)

# Drehzahlen, Encodergrad und Lenkwerte am 2026-11-04 auf MOTOR_DEG_PER_CM = 18,29 umgerechnet (vorher 30 geschätzt,
# alles × 0,61): der Roboter fährt damit körperlich genauso wie in der Simulation eingestellt (ca. 13 cm/s).
# Fahren und Drehen immer mit diesem Tempo bis zum Ziel, dort bremsen (kein Langsamwerden vorher)
DRIVE_SPEED_DPS = 244       # Geradeaus (Grad pro Sekunde am Motor)
TURN_SPEED_DPS = 183

FRONT_TRUST_CM = 100.0      # Wand vorne weiter weg als das: nur nach Encoder fahren
STALL_S = 1.0               # Ketten drehen sich trotz Fahrbefehl so lange nicht: blockiert
STALL_DEG = 6.0             # weniger Encoder-Bewegung (beide Ketten zusammen) gilt als Stillstand
FORWARD_TOLERANCE_CM = 8.0  # der Abstand vorne darf die Kachelstrecke um höchstens so viel ändern

# PD-Regler beim Geradeausfahren. In der Fahrphysik-Simulation mit Rauschen/Schlupf vorab eingestellt,
# auf dem echten Feld nachstellen (Simulation kennt die Maße, aber nicht das echte Fahrverhalten).
STEER_KP = 6.1              # Lenkkorrektur (dps) pro cm Abweichung von der Sollposition
STEER_KD = 183.0            # Lenkkorrektur (dps) pro cm Abweichung je cm Fahrt (Schrägstand)
STEER_D_WINDOW_CM = 4.0     # Schrägstand über so viele cm Fahrt bestimmen
STEER_ENC_K = 2.0           # ohne Seitenwand: Lenkkorrektur (dps) pro Encodergrad Unterschied (bleibt beim Umrechnen)
STEER_MAX_DPS = 73.0
STEER_SLEW_DPS_PER_CM = 24.0    # Lenkkorrektur ändert sich pro gefahrenem cm höchstens um so viel
# Einstellbar auf der Webseite (Reiter Einstellungen): Name -> (Bezeichnung, Einheit, kleinster, größter Wert, Hilfe).
# Die Webseite schreibt Abweichungen von den Werten oben nach SETTINGS_FILE, der Roboter liest sie vor jeder Fahrt.
SETTINGS_FILE = PROJECT_DIR / "data" / "settings.json"
STEER_SETTINGS = {
    "STEER_KP": ("Lenkstärke (Kp)", "dps pro cm", 0.0, 40.0,
                 "Wie stark er gegenlenkt, wenn er von der Mitte zwischen den Wänden abweicht."),
    "STEER_KD": ("Dämpfung (Kd)", "dps pro cm/cm", 0.0, 800.0,
                 "Wie stark er auf Schrägstand reagiert. Zu klein: pendelt. Zu groß: zittert."),
    "STEER_MAX_DPS": ("Größte Lenkung", "dps", 5.0, 150.0,
                      "So viel schneller darf eine Kette höchstens werden, wenn er lenkt."),
    "STEER_SLEW_DPS_PER_CM": ("Lenkänderung", "dps pro cm Fahrt", 1.0, 150.0,
                              "Wie schnell sich die Lenkung ändern darf. Klein: weich, aber träge."),
}


def read_settings(path: Path = SETTINGS_FILE) -> dict[str, float]:
    """Gespeicherte Einstellungen (nur bekannte Namen, nur Zahlen im erlaubten Bereich). Fehlt die Datei oder ist
    sie kaputt, gelten die Standardwerte (leeres Ergebnis)."""
    try:
        data = json.loads(Path(path).read_text())
    except (OSError, ValueError):
        return {}
    values = {}
    for name, (_label, _unit, low, high, _help) in STEER_SETTINGS.items():
        v = data.get(name) if isinstance(data, dict) else None
        if isinstance(v, (int, float)) and not isinstance(v, bool) and low <= v <= high:
            values[name] = float(v)
    return values


def write_settings(values: dict[str, float], path: Path = SETTINGS_FILE):
    """Einstellungen speichern (ganz oder gar nicht: erst in eine Hilfsdatei, dann umbenennen)."""
    path = Path(path)
    tmp = path.with_name(path.name + ".tmp")
    tmp.write_text(json.dumps(values, indent=2) + "\n")
    os.replace(tmp, path)


STEER_WIDTH_JUMP_CM = 1.0       # Breiten-Wächter: links + rechts weicht um mehr von B ab -> Seite ignorieren
                                # (Fehlalarm ist harmlos: dann kurz nur an einer Wand)
STEER_STEADY_CM = 1.5           # zurückkehrende Wand: 2 Messungen hintereinander mit höchstens so viel Unterschied
STEER_SIDE_MAX_CM = 55.0        # Seitenwand weiter weg: zum Lenken zu ungenau
CORRIDOR_CM = 2 * SIDE_CENTER_CM                    # Startwert Gangbreite B = links + rechts (wird gemessen)
CORRIDOR_MAX_CM = CORRIDOR_CM + TILE_SIZE_CM / 2    # links + rechts größer = breiter Raum, nähere Wand zählt

# Schrägstand aus den Seitenwänden (statt Ausrichten an Wänden), am Kachelende geradedrehen
HEADING_MIN_SAMPLES = 4         # so viele Seitenmessungen mit zwei Wänden braucht die Schätzung mindestens
HEADING_MIN_SPAN_CM = 10.0      # über so viele cm Fahrt verteilt
HEADING_FIX_MIN_DEG = 1.0       # kleinerer Schrägstand: nicht drehen
HEADING_FIX_MAX_DEG = 15.0      # höchstens so weit drehen

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

# --- Netzwerk (Webseite, Reiter „Netzwerk“) --------------------------------------
NET_WIFI_PREFIX = "robocup-"    # WLAN-Verbindungen, die die Seite anlegt, heißen so; nur diese löscht sie wieder
NET_WIFI_METRIC = 600           # Routenpriorität dieser Verbindungen; LAN hat 100 und gewinnt immer
NET_WIFI_POWERSAVE = 2          # WLAN-Energiesparen dieser Verbindungen: 2 = aus (NetworkManager: 0 Standard, 1 ignorieren, 3 an)
NET_QUERY_S = 15                # nmcli-Abfragen: so lange höchstens warten
NET_SCAN_S = 25                 # Suche nach Netzen
NET_CONNECT_S = 30              # Verbinden (nmcli -w)

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
