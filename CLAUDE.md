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
  - 1× RGB-Farbsensor nach unten, **vorne mittig, ca. 2 cm hinter der Vorderkante**.
    Vermutlich I2C (z.B. TCS34725), noch nicht bestätigt. I2C ist auf dem Pi derzeit **deaktiviert**.
  - 1× **PmodGYRO** (Digilent, 3-Achsen-Gyroskop), über **I2C** angeschlossen.
  - 3× Ultraschall: vorne, links, rechts. **Noch nicht verbunden**, Modell/Pins offen.
- Kamera vorhanden, Nutzung noch offen
- Sprache: Python

## Projektstruktur (vom Projektinhaber freigegeben)

```
robocup/
├── CLAUDE.md
├── config.py              # alle Einstellungen: Pins, Maße, Schwellwerte
├── main.py                # startet den Wettkampflauf
├── hardware/              # spricht direkt mit der Hardware
│   ├── motors.py          #   BrickPi3: fahren, drehen
│   ├── ultrasonic.py      #   3× Ultraschall
│   └── color_sensor.py    #   Farbsensor unten
├── navigation/            # reine Logik, kennt keine Hardware
│   ├── maze_map.py        #   die Karte
│   ├── pathfinding.py     #   Wegplanung
│   └── explorer.py        #   Erkundungslogik
├── simulation/            # Erkundung ohne echten Roboter testen
│   ├── sim_robot.py
│   └── mazes/             #   Testlabyrinthe als Textdateien
├── tools/                 # Testprogramme (Motoren, Sensoren, Farbkalibrierung)
├── tests/                 # automatische Tests der Logik
└── data/                  # Kalibrierwerte
```

Grundsatz: `navigation/` importiert nie etwas aus `hardware/`, damit die Logik im Simulator läuft.
Kamera später in einem eigenen Ordner (z.B. `vision/`); dafür ist noch eine Vorgabe nötig.

## Aktuelle Phase

Nur der **Erkundungsalgorithmus** wird programmiert, mit normalem Fahren:
**keine Rampen, keine Opfer**. Getestet wird in der Simulation mit Webseiten-Ansicht.

## Umsetzungsentscheidungen

### Erkundungsalgorithmus (freigegeben)

Karte und Grenzkacheln (Frontier) statt Wandfolgen.

- **Karte:** Gitter, Start = (0, 0), Startblickrichtung = „Norden“ (roboterintern).
  Pro Kachel: besucht ja/nein, Bodentyp (normal/schwarz/blau/silber).
  Pro Wand zwischen zwei Kacheln: Wand / offen / unbekannt, einmal pro Kante gespeichert.
- **Start:** Boden und Wände messen, 90° drehen, nochmal messen (kein Sensor hinten).
- **Hauptschleife:**
  1. Grenzkacheln suchen (unbesucht, über offene Wand von besuchter Kachel erreichbar)
  2. keine mehr → zurück zum Start → Ende
  3. günstigsten Weg zur nächsten Grenzkachel planen
  4. Weg abfahren
  5. auf neuer Kachel Boden und Wände messen, eintragen
- **Wegplanung:** Dijkstra über (Kachel, Blickrichtung). Kosten: geradeaus 1,
  90°-Drehung 1, blaue Kachel mit Aufschlag. Wege nur über besuchte Kacheln.
- **Sonderfälle:**
  - Schwarz: beim Fahren Boden prüfen → stoppen, zurück, als schwarz markieren, nie wieder einplanen
  - Blau: bei jedem Befahren warten (Wartezeit laut aktuellem Regelwerk)
  - Silber: Checkpoint, Kartenstand speichern
  - Widerspruch zur Karte (z.B. Wand, wo „offen“ eingetragen): Karte korrigieren, neu planen
  - Messrauschen: Wände nur beim ersten Besuch eintragen bzw. mehrfach messen
  - Zeitlimit: Heimweg-Zeit laufend schätzen, bei knapper Restzeit zum Start (Exit-Bonus)
  - Position: Kacheln per Encoder zählen, an Wänden korrigieren (Abstand vorne, mittig zwischen Seitenwänden)
- **Lack of Progress:** Der Roboter behält die **ganze Karte** (kein Zurückspringen auf den Checkpoint-Stand).
- **Später:** Rampen/Etagen (dritte Koordinate), Opfererkennung
  (pro Wand merken, ob schon abgesucht).

### Simulation

- Die Simulation wird auf einer **Webseite** angezeigt: Kacheln, Wände, Roboter usw.

## Offene Fragen an den Projektinhaber

- Lack of Progress: Taster als Signal?
- Farbsensor: Modell bestätigen
- Ultraschall: Modell und Pins (noch nicht verbunden)

## Bekannte Stolpersteine

- `lgpio` (Backend von `gpiozero` auf dem Pi 5) legt Dateien im aktuellen
  Arbeitsverzeichnis an. Programme müssen aus einem beschreibbaren Ordner gestartet werden.
- HC-SR04-Ultraschall (falls verwendet): Echo liefert 5 V, Pi 5 verträgt nur 3,3 V → Spannungsteiler.
