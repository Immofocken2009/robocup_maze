"""Lenkung beim Geradeausfahren: PD-Regler an den Seitenwänden (`Steering`) und Schätzung des Schrägstands
aus den Seitenwänden (`HeadingEstimate`). Reine Rechenklassen ohne Hardware.
"""
import math
from collections import deque

import config


def fit_slope(samples) -> float:
    """Steigung der Ausgleichsgeraden (kleinste Quadrate) durch (gefahrene cm, Abweichung)-Proben; 0 bei weniger als 2 Proben
    oder ohne Spannweite."""
    n = len(samples)
    if n < 2:
        return 0.0
    ms = sum(t for t, _ in samples) / n
    mo = sum(o for _, o in samples) / n
    var = sum((t - ms) ** 2 for t, _ in samples)
    return sum((t - ms) * (o - mo) for t, o in samples) / var if var > 0 else 0.0


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


STEER_TEXT = {"both": "lenkt auf die Mitte zwischen beiden Wänden", "left": "lenkt an der linken Wand",
              "right": "lenkt an der rechten Wand", "encoder": "geradeaus nach Encoder"}


class Steering:
    """PD-Regler für das Geradeausfahren. Ergebnis: Lenkkorrektur in dps, positiv = nach links
    (rechte Kette schneller).

    Grundregel: links und rechts gleich. Die Gangbreite B = links + rechts ändert sich beim
    seitlichen Wandern nicht, nur wenn eine Wand wegbricht oder auftaucht. Darauf beruht der
    Breiten-Wächter, der eine Seite ignoriert, sobald sie nicht mehr zu B passt. Fehlt eine Seite,
    hält er zur anderen B / 2. Eine Seite zählt erst nach zwei zusammenpassenden Messungen.
    Den D-Anteil gibt es nur, wenn beide Seiten zählen: Bei einer einzelnen Wand hält er sonst das
    Hochkriechen einer wegbrechenden Wand für Schrägstand.
    """

    def __init__(self, corridor_cm: float = config.CORRIDOR_CM):
        self.corridor = corridor_cm     # Gangbreite einer Kachel (links + rechts), wird nachgeführt
        self.width = corridor_cm        # Gangbreite B, solange beide Seiten zählen
        self.last: dict[str, float | None] = {"left": None, "right": None}
        self.trusted = {"left": False, "right": False}
        self.pending: dict[str, float | None] = {"left": None, "right": None}   # wartet auf Bestätigung
        self.mode: str | None = None    # both, left, right, encoder
        self.offset = 0.0               # letzte Abweichung von der Sollposition (cm, positiv = zu weit rechts)
        self.slope = 0.0                # letzter Schrägstand (Abweichung je cm Fahrt) für den D-Anteil
        self.raw = 0.0                  # Lenkkorrektur vor Begrenzung (dps)
        self.samples: deque[tuple[float, float]] = deque()   # (gefahrene cm, Abweichung cm)
        self.enc_ref = 0.0
        self.corr = 0.0
        self.corr_at = 0.0
        self.first: dict[str, float | None] = {"left": None, "right": None}   # gefahrene cm bei der ersten Messung der Seite
        self.hist: dict[str, deque[tuple[float, bool]]] = {"left": deque(), "right": deque()}   # (gefahrene cm, im Kontakt)
        self.contact: str | None = None     # Seite, die bei der letzten Messung an der Wand schleifte
        self._jump = False                  # nächste Lenkung ohne Begrenzung (Kontakt)

    def side(self, side: str, cm: float | None, travelled: float, enc: tuple[float, float]) -> float:
        """Neue Messung einer Seite. travelled: gefahrene cm, enc: Encodergrad seit dem Start."""
        other = "right" if side == "left" else "left"
        prev = self.last[side]
        self.last[side] = cm
        contact = cm is not None and cm <= config.STEER_CONTACT_CM
        self.contact = side if contact else None
        if contact:                                     # Kante schleift an der Wand: sofort ernst nehmen
            self.trusted[side] = True
            self.pending[side] = None
            self._jump = True
        elif cm is None or cm > config.STEER_SIDE_MAX_CM:
            self.trusted[side] = False
            self.pending[side] = None
        elif self.trusted[side]:
            if self.trusted[other]:
                diff = cm + self.last[other] - self.width
                if abs(diff) > config.STEER_WIDTH_JUMP_CM:
                    self._distrust(side, cm)            # Wand bricht weg oder taucht auf
                elif abs(diff) < config.STEER_WIDTH_JUMP_CM / 2:
                    self.width += 0.2 * diff            # langsam nachführen (Wände nicht ganz parallel)
            elif prev is not None and abs(cm - prev) > config.STEER_WIDTH_JUMP_CM:
                self._distrust(side, cm)
        elif self.pending[side] is not None and abs(cm - self.pending[side]) <= config.STEER_STEADY_CM:
            self.trusted[side] = True                   # zweite passende Messung: zählt wieder
            self.pending[side] = None
            if self.trusted[other] and not self._near(other):
                self.width = cm + self.last[other]
        else:
            self.pending[side] = cm
        if self.first[side] is None:
            self.first[side] = travelled
        self.hist[side].append((travelled, contact))
        if self.trusted["left"] and self.trusted["right"] and self.width <= config.CORRIDOR_MAX_CM \
                and not self._near("left") and not self._near("right"):     # schleifende Seite verfälscht B nicht
            self.corridor = self.width
        return self._control(travelled, enc, new_sample=True)

    def _near(self, side: str) -> bool:
        cm = self.last[side]
        return cm is not None and cm <= config.STEER_CONTACT_CM

    def sticking(self, travelled: float) -> str | None:
        """Seite, die schleift: seit mehr als STEER_CONTACT_DIST_CM Fahrt gemessen, und in den letzten
        STEER_CONTACT_DIST_CM cm waren mindestens STEER_CONTACT_SHARE der Messungen (mindestens 3) im Kontakt.
        Ein einzelner Ausreißer dazwischen setzt nichts zurück (Läufe 2026-11-11: zweimal bei 8,9 von 10 cm)."""
        for side in ("left", "right"):
            hist, first = self.hist[side], self.first[side]
            while hist and travelled - hist[0][0] > config.STEER_CONTACT_DIST_CM:
                hist.popleft()
            if first is None or travelled - first <= config.STEER_CONTACT_DIST_CM or len(hist) < 3:
                continue
            if sum(1 for _, c in hist if c) >= config.STEER_CONTACT_SHARE * len(hist):
                return side
        return None

    def hold(self, travelled: float, enc: tuple[float, float]) -> float:
        """Zwischen zwei Seitenmessungen (ohne Seitenwand wird über die Encoder weiter geregelt)."""
        return self._control(travelled, enc, new_sample=False)

    def _distrust(self, side: str, cm: float):
        self.trusted[side] = False
        self.pending[side] = cm

    def _offset(self) -> tuple[str, float]:
        """Abweichung von der Sollposition in cm, positiv = zu weit rechts."""
        left = self.last["left"] if self.trusted["left"] else None
        right = self.last["right"] if self.trusted["right"] else None
        half = self.corridor / 2
        if left is not None and self._near("left"):     # Kontakt: nur diese Wand zählt, Abstand B / 2
            return "left", _grid_offset(left, half)
        if right is not None and self._near("right"):
            return "right", -_grid_offset(right, half)
        if left is not None and right is not None and left + right <= config.CORRIDOR_MAX_CM:
            return "both", (left - right) / 2           # Gang: links und rechts gleich
        if left is not None and (right is None or left <= right):
            return "left", _grid_offset(left, half)     # sonst die nähere bzw. einzige Wand
        if right is not None:
            return "right", -_grid_offset(right, half)
        return "encoder", 0.0

    def _control(self, travelled: float, enc: tuple[float, float], new_sample: bool) -> float:
        mode, offset = self._offset()
        self.offset = offset
        if mode != self.mode:                           # Wechsel: D-Anteil und Encoder neu beginnen
            self.samples.clear()
            self.enc_ref = enc[0] - enc[1]
            self.mode = mode
        if mode == "encoder":
            raw = config.STEER_ENC_K * (enc[0] - enc[1] - self.enc_ref)
            self.slope = 0.0
        elif not new_sample:
            return self.corr
        else:
            self.samples.append((travelled, offset))
            while len(self.samples) > 2 and travelled - self.samples[1][0] >= config.STEER_D_WINDOW_CM:
                self.samples.popleft()
            slope = 0.0
            t0 = self.samples[0][0]
            if len(self.samples) >= 3 and travelled - t0 >= config.STEER_D_WINDOW_CM / 2:
                # Ausgleichsgerade über alle Proben im Fenster: die Differenz der Endpunkte gab bei 0,4 cm Messrauschen über
                # 2-4 cm Fahrt ein Lenkrauschen bis ca. 35 dps
                slope = fit_slope(self.samples)            # positiv = driftet nach rechts
            if mode != "both" or self.width > config.CORRIDOR_MAX_CM:
                slope = 0.0                                 # kein echter Gang (zwei nahe Wände): nur P
                self.samples.clear()                        # D-Fenster beginnt neu
            raw = config.STEER_KP * offset + config.STEER_KD * slope
            self.slope = slope
        self.raw = raw
        # Lenkänderung pro gefahrenem cm begrenzen: kein Ruck, auch nicht bei einer Fehlmessung
        allowed = config.STEER_SLEW_DPS_PER_CM * max(0.0, travelled - self.corr_at)
        if new_sample and self._jump:                   # Kontakt: sofort von der Wand weglenken
            allowed = float("inf")
        self._jump = False
        self.corr += _clamp(_clamp(raw, config.STEER_MAX_DPS) - self.corr, allowed)
        self.corr_at = travelled
        return self.corr


    def values(self) -> dict:
        """Alle Werte der Lenkung (für die Aufzeichnung)."""
        pd = self.mode not in (None, "encoder")
        return {"links": self.last["left"], "rechts": self.last["right"],
                "zählt_links": self.trusted["left"], "zählt_rechts": self.trusted["right"],
                "wartet_links": self.pending["left"], "wartet_rechts": self.pending["right"],
                "breite": self.width, "gang": self.corridor, "modus": self.mode,
                "kontakt": self.contact,
                "abweichung": self.offset, "schräg": self.slope,
                "p": config.STEER_KP * self.offset if pd else None,
                "d": config.STEER_KD * self.slope if pd else None,
                "encoder_ref": self.enc_ref, "soll": self.raw, "korrektur": self.corr}


class HeadingEstimate:
    """Schrägstand am Ende einer Fahrt, geschätzt aus den Seitenwänden (nur Messungen mit zwei Wänden).

    Die Abweichung (links − rechts) / 2 ändert sich pro gefahrenem cm um sin(Schrägwinkel). Was die
    Lenkung selbst dreht, zeigen die Encoder und wird abgezogen; übrig bleibt der Winkel beim Start,
    geschätzt per Ausgleichsgerade. Schrägstand am Ende = Winkel beim Start + Drehung laut Encoder.
    Die Seitensensoren sitzen US_SIDE_AHEAD_CM vor der Drehmitte: dreht die Lenkung, verschiebt das die
    Messung sofort um US_SIDE_AHEAD_CM · sin(Drehung); das wird ebenfalls abgezogen.
    """

    def __init__(self, direction: int = 1):
        # direction -1 = rückwärts: die Seitenabweichung wandert pro gefahrenem cm um −sin(Schrägwinkel) (Fahrt entgegen
        # der Blickrichtung), `travelled` zählt immer die Strecke (positiv), der Winkel kommt aus den echten Encodern
        self.direction = direction
        self.at = 0.0           # gefahrene cm beim letzten track()
        self.angle = 0.0        # Drehung seit dem Start laut Encoder (rad, positiv = nach rechts)
        self.drift = 0.0        # dadurch seitlich gewandert (cm)
        self.samples: list[tuple[float, float]] = []   # (gefahrene cm, Abweichung ohne Encoder-Anteil und Hebel)

    def track(self, travelled: float, enc: tuple[float, float]):
        """Bei jedem Schleifendurchlauf: enc = Encodergrad seit dem Start."""
        self.drift += self.direction * math.sin(self.angle) * (travelled - self.at)
        self.at = travelled
        self.angle = math.radians(90 * (enc[0] - enc[1]) / 2 / config.MOTOR_DEG_PER_90)

    def sample(self, offset: float):
        """Neue Abweichung, während beide Wände zählen."""
        lever = config.US_SIDE_AHEAD_CM * math.sin(self.angle)
        if self.direction < 0:      # rückwärts steht im Offset schon −Hebel statt +Hebel (`RealRobot._back_side`): zurückrechnen
            lever = -lever
        self.samples.append((self.at, offset - self.drift - lever))

    def degrees(self) -> float | None:
        """Schrägstand jetzt in Grad (positiv = nach rechts gedreht), None = zu wenig Messungen."""
        if len(self.samples) < config.HEADING_MIN_SAMPLES:
            return None
        s = [p[0] for p in self.samples]
        y = [p[1] for p in self.samples]
        if max(s) - min(s) < config.HEADING_MIN_SPAN_CM:
            return None
        ms, my = sum(s) / len(s), sum(y) / len(y)
        slope = self.direction * sum((a - ms) * (b - my) for a, b in zip(s, y)) / sum((a - ms) ** 2 for a in s)
        return math.degrees(math.asin(_clamp(slope, 1.0)) + self.angle)


def _grid_offset(cm: float, half: float) -> float:
    """Abstand zur Wand minus Sollabstand. Bei einer fernen Wand (Rasterabstand, weiter als eine halbe
    Kachel hinter der eigenen) auf ±STEER_GRID_MAX_CM begrenzt: der Schallkegel/Wandrand täuscht dort leicht."""
    off = cm - _grid(cm, half)
    if cm > half + config.TILE_SIZE_CM / 2:
        off = _clamp(off, config.STEER_GRID_MAX_CM)
    return off


def _grid(cm: float, half: float) -> float:
    """Sollabstand zu einer Wand, die (ganze Kacheln weiter) neben der eigenen Kachel steht."""
    return half + config.TILE_SIZE_CM * round((cm - half) / config.TILE_SIZE_CM)
