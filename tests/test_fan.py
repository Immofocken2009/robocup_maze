"""Tests für die Lüfterstufen (hardware/fan.py)."""
import unittest

import config
from hardware.fan import FULL_PWM, fan_level, fan_pwm

STEPS = config.FAN_STEPS
HYST = config.FAN_HYST_C


def run(temps: list[float], level: int = 0) -> list[int]:
    levels = []
    for t in temps:
        level = fan_level(t, level)
        levels.append(level)
    return levels


class FanTest(unittest.TestCase):
    def test_off_below_first_step(self):
        self.assertEqual(fan_level(STEPS[0][0] - 0.1, 0), 0)
        self.assertEqual(fan_pwm(STEPS[0][0] - 0.1, 0), 0)

    def test_steps_up_at_thresholds(self):
        for i, (t, pwm) in enumerate(STEPS, start=1):
            self.assertEqual(fan_level(t, 0), i)
            self.assertEqual(fan_pwm(t, i), pwm)

    def test_hysteresis_going_down(self):
        t1 = STEPS[0][0]
        levels = run([t1, t1 - 1, t1 - HYST + 0.1, t1 - HYST - 0.1])
        self.assertEqual(levels, [1, 1, 1, 0])     # erst HYST unter der Schwelle wieder aus

    def test_no_flapping_at_threshold(self):
        t2 = STEPS[1][0]
        levels = run([t2, t2 - 0.3, t2 + 0.2, t2 - 0.4, t2])
        self.assertEqual(set(levels), {2})

    def test_full_speed_when_hot_or_unknown(self):
        self.assertEqual(fan_pwm(config.FAN_FULL_C, fan_level(config.FAN_FULL_C, 0)), FULL_PWM)
        self.assertEqual(fan_pwm(None, fan_level(None, 0)), FULL_PWM)


if __name__ == "__main__":
    unittest.main()
