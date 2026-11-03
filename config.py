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

# --- Taster (BCM-GPIO-Nummern) -----------------------------------------------
# 4er-Tastermodul (GND, K1-K4): beim Drücken wird der Pin mit GND verbunden -> interne Pull-ups
BUTTON_PINS = {"K1": 17, "K2": 27, "K3": 22, "K4": 23}

# --- Ultraschall -------------------------------------------------------------
US_MAX_DISTANCE_CM = 150.0  # weiter weg = "nichts gemessen"
WALL_THRESHOLD_CM = 20.0    # gemessener Abstand kleiner als das = Wand

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
MOVE_TIME_S = 2.0           # eine Kachel vorwärts. MESSEN!
TURN_TIME_S = 1.5           # eine 90°-Drehung. MESSEN!
BACKOFF_TIME_S = 2.0        # schwarze/rote Kachel erkannt, anhalten und zurück. MESSEN!

# --- Zufallslabyrinth --------------------------------------------------------
FIELD_SIZES_2026 = [(8, 7), (8, 8), (9, 8), (7, 8), (4, 12)]   # offizielle Felder 2026
GENERATOR_DEFAULTS = {"width": 8, "height": 7, "black": 2, "blue": 2, "silver": 3, "zones": 1}
GENERATOR_MAX_SIZE = 16

# --- Webseite ----------------------------------------------------------------
WEB_HOST = "0.0.0.0"        # 0.0.0.0 = auch von anderen Geräten im Netz erreichbar
WEB_PORT = 8000
