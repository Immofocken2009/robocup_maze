"""Testet das 4er-Tastermodul (K1-K4).

Starten (im Projektordner):  python3 -m tools.test_buttons
Jeden Taster ein paar Mal drücken, beenden mit Strg+C.
"""
import time

from gpiozero import Button

import config


def main():
    buttons = {name: Button(pin, pull_up=True, bounce_time=0.03)
               for name, pin in config.BUTTON_PINS.items()}
    presses = {name: 0 for name in buttons}
    start = time.monotonic()

    def stamp() -> str:
        return f"{time.monotonic() - start:6.2f} s"

    def pressed(name):
        presses[name] += 1
        print(f"{stamp()}  {name} (GPIO {config.BUTTON_PINS[name]}) gedrückt   [{presses[name]}×]")

    def released(name):
        print(f"{stamp()}  {name} (GPIO {config.BUTTON_PINS[name]}) losgelassen")

    for name, button in buttons.items():
        button.when_pressed = lambda _b, n=name: pressed(n)
        button.when_released = lambda _b, n=name: released(n)

    print("Zustand beim Start:")
    stuck = []
    for name, button in buttons.items():
        state = "GEDRÜCKT" if button.is_pressed else "nicht gedrückt"
        print(f"  {name}  GPIO {config.BUTTON_PINS[name]:>2}  {state}")
        if button.is_pressed:
            stuck.append(name)
    if stuck:
        print(f"\nAchtung: {', '.join(stuck)} meldet schon ohne Drücken 'gedrückt'.\n"
              "Entweder wird der Taster gerade gedrückt, oder das Modul schaltet nicht gegen GND\n"
              "(dann wäre pull_up=False nötig) oder die Verkabelung stimmt nicht.")

    print("\nJetzt jeden Taster drücken. Beenden mit Strg+C.\n")
    try:
        while True:
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass

    print("\nErgebnis:")
    for name, count in presses.items():
        result = f"{count}× gedrückt, OK" if count else "NICHT erkannt, Verkabelung prüfen"
        print(f"  {name}  GPIO {config.BUTTON_PINS[name]:>2}  {result}")
    for button in buttons.values():
        button.close()


if __name__ == "__main__":
    main()
