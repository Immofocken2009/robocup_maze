"""Antrieb: zwei Ketten mit großen LEGO-Motoren am BrickPi3 (A = links, D = rechts).

Alle Werte sind so umgerechnet, dass positiv = vorwärts heißt (config.MOTOR_DIRECTION).
"""
import brickpi3

import config

SIDES = ("left", "right")


class Motors:
    def __init__(self):
        self.bp = brickpi3.BrickPi3()   # wirft eine Ausnahme, wenn der BrickPi3 nicht antwortet
        self.ports = {side: getattr(self.bp, f"PORT_{config.MOTOR_PORTS[side]}") for side in SIDES}
        self.sign = config.MOTOR_DIRECTION
        self.float()

    def set_dps(self, left: float, right: float):
        """Drehzahl in Grad pro Sekunde, positiv = vorwärts."""
        for side, dps in (("left", left), ("right", right)):
            self.bp.set_motor_dps(self.ports[side], int(dps * self.sign[side]))

    def positions(self) -> tuple[float, float]:
        """Encoder in Grad (links, rechts), positiv = vorwärts gefahren."""
        return tuple(self.bp.get_motor_encoder(self.ports[side]) * self.sign[side] for side in SIDES)

    def set_power_limit(self, percent: int):
        """Höchste Motorleistung in % (0 = ohne Grenze)."""
        for port in self.ports.values():
            self.bp.set_motor_limits(port, percent, 0)

    def stop(self):
        """Anhalten und bremsen."""
        for port in self.ports.values():
            self.bp.set_motor_power(port, 0)

    def float(self):
        """Motoren stromlos, Ketten frei drehbar."""
        for port in self.ports.values():
            self.bp.set_motor_power(port, self.bp.MOTOR_FLOAT)

    def battery(self) -> float:
        return self.bp.get_voltage_battery()

    def close(self):
        self.set_power_limit(0)
        self.float()
