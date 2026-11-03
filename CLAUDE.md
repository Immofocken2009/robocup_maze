# RoboCup Junior Rescue Maze – Projektregeln

## Arbeitsregeln (vom Projektinhaber festgelegt, gelten immer)

1. **Vor jeder Änderung an Code wird der aktuelle Stand ins lokale Git committet.**
   Erst committen, dann ändern. (Lokales Repo, Branch `main`, kein Remote.)
2. **Alle grundlegenden Regeln und Umsetzungsentscheidungen kommen in diese `CLAUDE.md`.**
   Wird etwas Neues festgelegt, wird es hier eingetragen.
3. **Alle Strukturvorgaben kommen vom Projektinhaber.** Claude legt keine eigene
   Ordner-/Datei-/Modulstruktur fest, sondern fragt nach oder setzt die Vorgabe um.
4. **Bei umsetzungstechnischen Fragen wird immer der Projektinhaber gefragt.**
   Claude entscheidet keine technischen Ansätze (Algorithmen, Bibliotheken,
   Regelungen, Pins, ...) eigenmächtig.

## Hardware (Stand 2026-11-03)

- Raspberry Pi 5, Debian 13 (trixie), Python 3.13
- BrickPi3 (SPI, `/dev/spidev0.1`, belegt GPIO 7–11), Bibliothek `brickpi3` ist installiert
- Antrieb: große LEGO-Motoren am BrickPi3
- Sensoren an den GPIOs des Pi:
  - 1× RGB-Farbsensor nach unten, in der Robotermitte. Vermutlich I2C (z.B. TCS34725),
    noch nicht bestätigt. I2C ist auf dem Pi derzeit **deaktiviert**.
  - 3× Ultraschall: vorne, links, rechts. **Noch nicht verbunden**, Modell/Pins offen.
- Kamera vorhanden, Nutzung noch offen
- Sprache: Python

## Aktuelle Phase

Erkundungslogik **konzipieren und erklären, nicht programmieren**.
Projektstruktur: Claude hat einen Vorschlag gemacht, die Entscheidung des Projektinhabers steht aus.

## Umsetzungsentscheidungen

(noch keine. Wird ergänzt, sobald der Projektinhaber etwas festlegt)

## Bekannte Stolpersteine

- `lgpio` (Backend von `gpiozero` auf dem Pi 5) legt Dateien im aktuellen
  Arbeitsverzeichnis an. Programme müssen aus einem beschreibbaren Ordner gestartet werden.
