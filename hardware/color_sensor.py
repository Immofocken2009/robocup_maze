"""Farbsensor unten (TCS34725 an I2C-Bus 1, Adresse 0x29), über smbus2.

Rohwerte R, G, B, C (C = Helligkeit ohne Filter). Die Bodenfarbe ist die nächstliegende
kalibrierte Farbe aus data/colors.json (erstellt mit tools/calibrate_color.py).
"""
import json
import time
from collections import Counter
from pathlib import Path

from smbus2 import SMBus

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
        """Nächstliegende kalibrierte Farbe ('white', 'black', ...), None ohne Kalibrierung."""
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


def load_calibration(path: Path = config.COLOR_CALIBRATION_FILE) -> tuple[dict[str, Raw] | None, str | None]:
    """Kalibrierwerte laden. Ergebnis: (Farben, Warnung)."""
    if not path.exists():
        return None, "Farbsensor nicht kalibriert (python3 -m tools.calibrate_color)"
    data = json.loads(path.read_text())
    colors = {name: tuple(values) for name, values in data["colors"].items()}
    settings = data.get("settings", {})
    if (settings.get("integration_ms") != config.COLOR_INTEGRATION_MS
            or settings.get("gain") != config.COLOR_GAIN):
        return colors, "Farbsensor mit anderen Einstellungen kalibriert, bitte neu kalibrieren"
    missing = [n for n in ("white", "black") if n not in colors]
    if missing:
        return colors, f"Kalibrierung unvollständig, es fehlt: {', '.join(missing)}"
    return colors, None


def features(raw: Raw, white_c: float) -> tuple[float, float, float, float]:
    """Farbanteile (unabhängig von der Helligkeit) und Helligkeit relativ zu Weiß."""
    r, g, b, c = raw
    c = max(c, 1)
    return (2 * r / c, 2 * g / c, 2 * b / c, min(3.0, c / max(white_c, 1)))


def classify(raw: Raw, calibration: dict[str, Raw]) -> str:
    white_c = calibration["white"][3] if "white" in calibration else max(v[3] for v in calibration.values())
    f = features(raw, white_c)

    def distance(name: str) -> float:
        ref = features(calibration[name], white_c)
        return sum((a - b) ** 2 for a, b in zip(f, ref))

    return min(calibration, key=distance)
