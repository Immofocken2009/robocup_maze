"""Rechenhilfen fürs Fahren: Haltepunkt im Kachelraster, Stillstandswächter, Zentrieren vor der Drehung.
Keine Hardware.
"""
import config


def stop_distance(front_cm: float | None, travelled: float) -> float | None:
    """Fahrstrecke (ab Start der Fahrt), bei der der Roboter mittig auf der nächsten Kachel steht,
    aus einer Messung vorne nach `travelled` cm. Die Wand darf mehrere Kacheln entfernt sein:
    gewählt wird die Kachelmitte, die am nächsten an einer Kachellänge liegt. So passt der
    Haltepunkt auch dann, wenn das Echo von einer anderen Wand oder Wandkante kommt als vorher.
    None = keine Wand in Reichweite."""
    if front_cm is None or front_cm > config.FRONT_TRUST_CM:
        return None
    ahead = travelled + front_cm - config.FRONT_CENTER_CM     # bis zur Mitte der Kachel vor der Wand
    tiles = round((ahead - config.TILE_SIZE_CM) / config.TILE_SIZE_CM)
    stop = ahead - tiles * config.TILE_SIZE_CM
    tolerance = config.FORWARD_TOLERANCE_CM
    return max(config.TILE_SIZE_CM - tolerance, min(config.TILE_SIZE_CM + tolerance, stop))


def stop_distance_back(front_cm: float | None, travelled: float) -> float | None:
    """Wie `stop_distance`, aber rückwärts: der Abstand vorne WÄCHST mit der gefahrenen Strecke. Fahrstrecke (ab Start der
    Fahrt), bei der der Roboter mittig auf der Kachel hinter ihm steht, aus einer Messung vorne nach `travelled` cm: die
    Kachelmitte (Abstand vorne = FRONT_CENTER_CM + n · Kachellänge), die ca. eine Kachellänge weiter weg liegt als am Start.
    None = keine Wand in Reichweite."""
    if front_cm is None or front_cm > config.FRONT_TRUST_CM:
        return None
    reached = travelled + config.FRONT_CENTER_CM - front_cm    # Strecke bis zur Mitte der Kachel direkt vor der Wand (n = 0)
    tiles = round((config.TILE_SIZE_CM - reached) / config.TILE_SIZE_CM)
    stop = reached + tiles * config.TILE_SIZE_CM
    tolerance = config.FORWARD_TOLERANCE_CM
    return max(config.TILE_SIZE_CM - tolerance, min(config.TILE_SIZE_CM + tolerance, stop))


class StallWatch:
    """Merkt, wenn sich die Ketten trotz Fahrbefehl länger nicht drehen (z.B. schräg an einer Wand)."""

    def __init__(self, clock):
        self.clock = clock
        self.ref: tuple[float, float] | None = None
        self.since = clock.monotonic()

    def stalled(self, enc: tuple[float, float]) -> bool:
        now = self.clock.monotonic()
        if self.ref is None or abs(enc[0] - self.ref[0]) + abs(enc[1] - self.ref[1]) > config.STALL_DEG:
            self.ref, self.since = enc, now
            return False
        return now - self.since >= config.STALL_S


def centering_plan(front: float | None, left: float | None, right: float | None) -> tuple[float, float]:
    """Wie weit zentrieren vor einer Drehung: (zurück in cm, seitlicher Versatz in cm, > 0 = nach links schieben).
    Nur, wenn eine Ecke anstoßen würde (vorne < TURN_FRONT_MIN_CM, Seite < TURN_SIDE_MIN_CM), und dann nur so weit wie
    nötig. Kein Echo oder offene Fläche (nichts unter WALL_THRESHOLD_CM): (0, 0)."""
    values = [v for v in (front, left, right) if v is not None]
    if not values or min(values) >= config.WALL_THRESHOLD_CM:
        return 0.0, 0.0
    back = 0.0
    if front is not None and front < config.TURN_FRONT_MIN_CM:
        back = min(config.FRONT_CENTER_CM - front, config.DEAD_END_MAX_BACK_CM)
    side = 0.0
    near_left = left is not None and left < config.TURN_SIDE_MIN_CM
    near_right = right is not None and right < config.TURN_SIDE_MIN_CM
    if near_left or near_right:
        if near_left and near_right:
            side = (left - right) / 2
        elif near_left:
            wall = right is not None and right < config.WALL_THRESHOLD_CM
            side = left - ((left + right) / 2 if wall else config.SIDE_CENTER_CM)
        else:
            wall = left is not None and left < config.WALL_THRESHOLD_CM
            side = ((left + right) / 2 if wall else config.SIDE_CENTER_CM) - right
        if abs(side) <= config.DEAD_END_SIDE_MIN_CM:
            side = 0.0
    return back, side
