"""4er-Tastermodul (GND, K1–K4). Schaltet gegen GND, interne Pull-ups (gedrückt = LOW).

Funktionen laut config.BUTTON_FUNCTIONS: K1 = Start, K2 = Stopp, K3 = Lack of Progress, K4 frei.
"""
from collections.abc import Callable

from gpiozero import Button

import config


class Buttons:
    def __init__(self, on_press: Callable[[str], None]):
        """on_press bekommt die Funktion ('start', 'stop', 'lop'), läuft im gpiozero-Thread."""
        functions = {key: name for name, key in config.BUTTON_FUNCTIONS.items()}
        self.buttons = {key: Button(pin, pull_up=True, bounce_time=0.03)
                        for key, pin in config.BUTTON_PINS.items()}
        for key, button in self.buttons.items():
            if key in functions:
                button.when_pressed = lambda _b, f=functions[key]: on_press(f)

    def states(self) -> dict[str, bool]:
        """Welche Taster gerade gedrückt sind."""
        return {key: button.is_pressed for key, button in self.buttons.items()}

    def close(self):
        for button in self.buttons.values():
            button.close()
