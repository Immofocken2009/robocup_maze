"""Auswertung eines Kamerabilds (RGB, uint8, Höhe x Breite x 3) mit numpy, ohne Hardware.

Liefert ein Dict mit allem, was die Webseite einzeichnet (Pixelkoordinaten im Bild):
  w, h, brightness, dark           Bildgröße, mittlere Helligkeit (0-255), zu dunkel für die Kantensuche
  vertical_edges                   senkrechte Kanten (Wandende, Ecke): x, y0, y1, share, bearing (Grad zur Fahrtrichtung,
                                   positiv = rechts)
  lines                            Kanten Boden/Wand als Geraden: x0, y0, x1, y1, n (Punkte), angle (Grad zur Waagerechten
                                   im Bild, positiv = nach rechts unten)
  patch                            Bildausschnitt unmittelbar vor dem Roboter (Kasten) mit mittlerer Farbe rgb
  motion                           mittlere Änderung gegenüber dem Bild davor (0-255), moving
Die Werte sind Kandidaten: Wände und Boden sind beide weiß, die Kanten hängen vom Licht ab.
"""
import math

import numpy as np

import config

BAND = (0.15, 0.85)             # Bildzeilen (Anteil der Höhe), in denen senkrechte Kanten gesucht werden
LINE_BAND = (0.30, 0.95)        # Bildzeilen, in denen die Kante Boden/Wand gesucht wird
LINE_STEP = 4                   # jede wievielte Spalte einen Punkt der Kante suchen
MAX_VERTICAL = 4
MAX_LINES = 2
PATCH = (0.40, 0.78, 0.60, 0.95)    # x0, y0, x1, y1 als Anteil von Breite/Höhe


def gray_of(rgb: np.ndarray) -> np.ndarray:
    return rgb[..., 0] * np.float32(0.299) + rgb[..., 1] * np.float32(0.587) + rgb[..., 2] * np.float32(0.114)


def vertical_edges(gx: np.ndarray, gy: np.ndarray, focal_px: float, width: int) -> list[dict]:
    """Spalten, in denen über einen großen Teil der Höhe waagerechte Helligkeitssprünge liegen (und nicht senkrechte)."""
    h = gx.shape[0]
    y0, y1 = int(BAND[0] * h), int(BAND[1] * h)
    gxb, gyb = gx[y0:y1], gy[y0:y1, 1:-1]
    rows = min(len(gxb), len(gyb))
    if rows == 0:
        return []
    mask = (gxb[:rows] > config.CAM_GRAD_MIN) & (gxb[:rows] > 2 * gyb[:rows])
    share = mask.mean(axis=0)
    share = np.convolve(share, np.ones(5) / 5, mode="same")
    found = []
    for i in range(len(share)):
        lo, hi = max(0, i - 6), min(len(share), i + 7)
        if share[i] >= config.CAM_VEDGE_SHARE and share[i] == share[lo:hi].max() and (
                i == lo or share[i] > share[lo:i].max()):
            ys = np.nonzero(mask[:, max(0, i - 1):i + 2].any(axis=1))[0]
            x = i + 1
            found.append({"x": x, "y0": int(y0 + ys.min()), "y1": int(y0 + ys.max()), "share": round(float(share[i]), 2),
                          "bearing": round(math.degrees(math.atan((x - width / 2) / focal_px)), 1)})
    found.sort(key=lambda e: -e["share"])
    return found[:MAX_VERTICAL]


def boundary_points(gy: np.ndarray, width: int) -> list[tuple[int, int]]:
    """Je Spalte (im Abstand LINE_STEP) die Zeile mit dem stärksten senkrechten Helligkeitssprung."""
    h = gy.shape[0]
    y0, y1 = int(LINE_BAND[0] * h), int(LINE_BAND[1] * h)
    points = []
    for x in range(LINE_STEP, width - LINE_STEP, LINE_STEP):
        column = gy[y0:y1, max(0, x - 2):x + 3].mean(axis=1)
        k = int(column.argmax())
        if column[k] > config.CAM_GRAD_MIN:
            points.append((x, y0 + k + 1))
    return points


def fit_lines(points: list[tuple[int, int]], width: int) -> list[dict]:
    """Bis zu MAX_LINES Geraden durch die Randpunkte (RANSAC mit festem Ablauf, daher wiederholbar)."""
    pts = list(points)
    lines = []
    for _ in range(MAX_LINES):
        if len(pts) < 8:
            break
        arr = np.array(pts, dtype=np.float32)
        best = None
        n = len(arr)
        for a in range(n):
            for b in range(a + max(3, n // 8), n, max(1, n // 12)):
                dx = arr[b, 0] - arr[a, 0]
                if dx < 1:
                    continue
                slope = (arr[b, 1] - arr[a, 1]) / dx
                err = np.abs(arr[:, 1] - (arr[a, 1] + slope * (arr[:, 0] - arr[a, 0])))
                inl = err <= config.CAM_LINE_TOL_PX
                count = int(inl.sum())
                if best is None or count > best[0]:
                    best = (count, inl)
        if best is None or best[0] < 8:
            break
        inl = best[1]
        xs, ys = arr[inl, 0], arr[inl, 1]
        if xs.max() - xs.min() < config.CAM_LINE_MIN_SPAN * width:
            break
        slope, icpt = np.polyfit(xs, ys, 1)
        x0, x1 = float(xs.min()), float(xs.max())
        lines.append({"x0": int(x0), "y0": int(round(icpt + slope * x0)), "x1": int(x1), "y1": int(round(icpt + slope * x1)),
                      "n": int(inl.sum()), "angle": round(math.degrees(math.atan(float(slope))), 1)})
        pts = [p for p, keep in zip(pts, inl) if not keep]
    return lines


class Analyzer:
    """Wertet Bild für Bild aus und merkt sich das vorige (für die Bewegung)."""

    def __init__(self, hfov_deg: float | None = None):
        self.hfov_deg = hfov_deg or config.CAM_HFOV_DEG
        self._prev: np.ndarray | None = None

    def analyse(self, rgb: np.ndarray) -> dict:
        gray = gray_of(rgb.astype(np.float32, copy=False))
        h, w = gray.shape
        focal = (w / 2) / math.tan(math.radians(self.hfov_deg / 2))
        brightness = float(gray.mean())
        out = {"w": w, "h": h, "brightness": round(brightness, 1), "dark": brightness < config.CAM_DARK_LEVEL,
               "vertical_edges": [], "lines": []}
        if not out["dark"]:
            gx = np.abs(gray[:, 2:] - gray[:, :-2])
            gy = np.abs(gray[2:, :] - gray[:-2, :])
            out["vertical_edges"] = vertical_edges(gx, gy, focal, w)
            out["lines"] = fit_lines(boundary_points(gy, w), w)
        px0, py0, px1, py1 = int(PATCH[0] * w), int(PATCH[1] * h), int(PATCH[2] * w), int(PATCH[3] * h)
        mean = rgb[py0:py1, px0:px1].reshape(-1, 3).mean(axis=0)
        out["patch"] = {"x0": px0, "y0": py0, "x1": px1, "y1": py1, "rgb": [int(round(float(v))) for v in mean]}
        small = gray[::4, ::4]
        motion = 0.0
        if self._prev is not None and self._prev.shape == small.shape:
            motion = float(np.abs(small - self._prev).mean())
        self._prev = small
        out["motion"] = round(motion, 2)
        out["moving"] = motion >= config.CAM_MOTION_MIN
        return out
