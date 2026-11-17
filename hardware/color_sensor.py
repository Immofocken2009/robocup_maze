"""Farbsensor unten (TCS34725 an I2C-Bus 1, Adresse 0x29), über smbus2.

Rohwerte R, G, B, C (C = Helligkeit ohne Filter). Die Bodenfarbe ist die nächstliegende
kalibrierte Farbe aus data/colors.json (erstellt mit tools/calibrate_color.py), gemessen als Abstand
in Streuungen (Sigma) der Merkmale; ist keine Farbe nah genug, gilt der Boden als unsicher (None).
"""
import json
import time
from collections import Counter
from pathlib import Path

try:
    from smbus2 import SMBus
except ImportError:     # Rechner ohne smbus2 (Simulation, Tests): ColorSensor() schlägt dann fehl
    SMBus = None

import config
from navigation.maze_map import TileType

COLOR_TO_TILE = {"white": TileType.NORMAL, "black": TileType.BLACK, "blue": TileType.BLUE,
                 "silver": TileType.SILVER, "red": TileType.RED}

_COMMAND = 0x80
_AUTO_INCREMENT = 0x20
_ENABLE, _ATIME, _CONTROL, _ID, _CDATAL = 0x00, 0x01, 0x0F, 0x12, 0x14
_PON, _AEN = 0x01, 0x02
_GAINS = {1: 0, 4: 1, 16: 2, 60: 3}
_IDS = (0x44, 0x4D)     # TCS34725, TCS34727

Raw = tuple[int, int, int, int]   # R, G, B, C
Features = tuple[float, float, float, float]
STATS_KEY = "_stats"    # in der Kalibrierung: {Farbe: {"mean": [4], "std": [4]}}, nur wenn für alle Farben vorhanden


class ColorSensor:
    def __init__(self, calibration_file: Path = config.COLOR_CALIBRATION_FILE):
        self.bus = SMBus(config.COLOR_I2C_BUS)
        chip_id = self._read(_ID)
        if chip_id not in _IDS:
            raise RuntimeError(f"Farbsensor: unerwartete ID 0x{chip_id:02X}")
        cycles = max(1, min(256, round(config.COLOR_INTEGRATION_MS / 2.4)))
        self._write(_ATIME, 256 - cycles)
        self._write(_CONTROL, _GAINS[config.COLOR_GAIN])
        self._write(_ENABLE, _PON)
        time.sleep(0.003)
        self._write(_ENABLE, _PON | _AEN)
        self.integration_s = cycles * 0.0024
        time.sleep(self.integration_s + 0.003)

        self.calibration, self.warning = load_calibration(calibration_file)
        self.last_raw: Raw | None = None
        self.last_color: str | None = None

    def _read(self, reg: int) -> int:
        return self.bus.read_byte_data(config.COLOR_I2C_ADDRESS, _COMMAND | reg)

    def _write(self, reg: int, value: int):
        self.bus.write_byte_data(config.COLOR_I2C_ADDRESS, _COMMAND | reg, value)

    def read_raw(self) -> Raw:
        """Letzte fertige Messung des Sensors (misst ständig im Hintergrund)."""
        data = self.bus.read_i2c_block_data(config.COLOR_I2C_ADDRESS,
                                            _COMMAND | _AUTO_INCREMENT | _CDATAL, 8)
        c, r, g, b = (data[i] | data[i + 1] << 8 for i in range(0, 8, 2))
        self.last_raw = (r, g, b, c)
        return self.last_raw

    def read_color(self) -> str | None:
        """Nächstliegende kalibrierte Farbe ('white', 'black', ...), None ohne Kalibrierung oder wenn unsicher."""
        raw = self.read_raw()
        self.last_color = classify(raw, self.calibration) if self.calibration else None
        return self.last_color

    def read_tile(self) -> TileType:
        """Boden als Kacheltyp. Ohne Kalibrierung immer NORMAL."""
        return COLOR_TO_TILE.get(self.read_color(), TileType.NORMAL)

    def read_floor(self, n: int = 3) -> TileType:
        """Im Stand: n frische Messungen, die häufigste Farbe gewinnt."""
        tiles = []
        for _ in range(n):
            time.sleep(self.integration_s)
            tiles.append(self.read_tile())
        return Counter(tiles).most_common(1)[0][0]

    def close(self):
        try:
            self._write(_ENABLE, 0)
        finally:
            self.bus.close()


def build_calibration(colors: dict, stats: dict | None) -> dict:
    """Kalibrierung für `classify`: Rohmittel je Farbe, dazu unter STATS_KEY die Streuung, wenn sie für alle Farben da ist."""
    cal = {name: tuple(values) for name, values in colors.items()}
    if stats and all(name in stats for name in cal):
        cal[STATS_KEY] = {name: {"mean": list(stats[name]["mean"]), "std": list(stats[name]["std"])} for name in cal}
    return cal


def load_calibration(path: Path = config.COLOR_CALIBRATION_FILE) -> tuple[dict | None, str | None]:
    """Kalibrierwerte laden. Ergebnis: (Farben samt Streuung, Warnung)."""
    if not path.exists():
        return None, "Farbsensor nicht kalibriert (python3 -m tools.calibrate_color)"
    try:
        data = json.loads(path.read_text())
        colors = build_calibration(data["colors"], data.get("stats"))
        settings = data.get("settings", {})
    except (OSError, ValueError, KeyError, TypeError, AttributeError):
        # Unlesbare Datei: wie beim Ultraschall Warnung statt Ausnahme (sonst käme ColorSensor() nie zustande und der
        # Roboter würde nie starten). Ohne Kalibrierung gilt jeder Boden als weiß (read_tile -> NORMAL).
        return None, f"Farbkalibrierung unlesbar ({path.name}), Boden gilt als weiß (python3 -m tools.calibrate_color)"
    if (settings.get("integration_ms") != config.COLOR_INTEGRATION_MS
            or settings.get("gain") != config.COLOR_GAIN):
        return colors, "Farbsensor mit anderen Einstellungen kalibriert, bitte neu kalibrieren"
    missing = [n for n in ("white", "black") if n not in colors]
    if missing:
        return colors, f"Kalibrierung unvollständig, es fehlt: {', '.join(missing)}"
    if STATS_KEY not in colors:
        return colors, "Farbkalibrierung ohne Streuung, bitte neu einmessen"
    return colors, None


def features(raw: Raw, white_c: float) -> Features:
    """Farbanteile (unabhängig von der Helligkeit) und Helligkeit relativ zu Weiß."""
    r, g, b, c = raw
    c = max(c, 1)
    return (2 * r / c, 2 * g / c, 2 * b / c, min(3.0, c / max(white_c, 1)))


def sigma_distance(f: Features, mean, std) -> float:
    """Abstand eines Merkmalsvektors von einer Farbe in Streuungen (Streuung nicht kleiner als COLOR_STD_MIN)."""
    return sum(((a - m) / max(s, floor)) ** 2
               for a, m, s, floor in zip(f, mean, std, config.COLOR_STD_MIN)) ** 0.5


def classify(raw: Raw, calibration: dict) -> str | None:
    """Farbname. Mit Streuung (STATS_KEY): nächste Farbe nach normiertem Abstand, None wenn sie weiter als
    COLOR_REJECT_SIGMA weg ist. Ohne Streuung (alte Kalibrierung): nächste Farbe nach Merkmalsabstand, nie None."""
    stats = calibration.get(STATS_KEY)
    colors = {n: v for n, v in calibration.items() if n != STATS_KEY}
    white_c = colors["white"][3] if "white" in colors else max(v[3] for v in colors.values())
    f = features(raw, white_c)
    if stats:
        distances = {n: sigma_distance(f, stats[n]["mean"], stats[n]["std"]) for n in colors}
        best = min(distances, key=distances.get)
        return best if distances[best] <= config.COLOR_REJECT_SIGMA else None

    def distance(name: str) -> float:
        ref = features(colors[name], white_c)
        return sum((a - b) ** 2 for a, b in zip(f, ref))

    return min(colors, key=distance)
