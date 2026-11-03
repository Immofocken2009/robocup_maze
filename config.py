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

# --- Ultraschall -------------------------------------------------------------
US_MAX_DISTANCE_CM = 150.0  # weiter weg = "nichts gemessen"
WALL_THRESHOLD_CM = 20.0    # gemessener Abstand kleiner als das = Wand

# --- Regeln ------------------------------------------------------------------
BLUE_WAIT_S = 5.0           # Wartezeit auf blauen Kacheln (Regelwerk prüfen!)
TIME_LIMIT_S = 8 * 60       # Zeit für einen Lauf
TIME_MARGIN_S = 20.0        # Sicherheitspuffer für den Heimweg

# --- Wegplanung (Kosten für Dijkstra) ----------------------------------------
PLAN_COST_FORWARD = 1.0
PLAN_COST_TURN = 1.0        # pro 90°-Drehung
PLAN_COST_BLUE = 5.0        # Aufschlag für das Befahren einer blauen Kachel

# --- Geschätzte Dauer der Aktionen (Sim-Zeit und Heimweg-Schätzung) ----------
MOVE_TIME_S = 2.0           # eine Kachel vorwärts. MESSEN!
TURN_TIME_S = 1.5           # eine 90°-Drehung. MESSEN!
BLACK_TIME_S = 2.0          # in schwarze Kachel hinein und zurück. MESSEN!

# --- Webseite ----------------------------------------------------------------
WEB_HOST = "0.0.0.0"        # 0.0.0.0 = auch von anderen Geräten im Netz erreichbar
WEB_PORT = 8000
