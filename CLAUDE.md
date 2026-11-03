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
- Antrieb: große LEGO-Motoren am BrickPi3, **2 Ketten**, angetrieben am **vorderen Kettenende**
  (dort sitzt der Motor). Ports: **A = links, D = rechts**. Länge/Breite/Kettenabstand noch messen.
- Sensoren an den GPIOs des Pi:
  - 1× RGB-Farbsensor nach unten, **vorne mittig, ca. 2 cm hinter der Vorderkante**.
    **Bestätigt:** erkennt die Farbe einer Kachel, bevor der Roboter zur Hälfte drauf ist
    (also bevor sie als „befahren“ zählt, Regel 5.4.4). Schwarz/Rot: erkennen, anhalten, zurück.
    **TCS34725** bestätigt (I2C-Bus 1, Adresse 0x29, ID 0x44). Liefert Werte (Stand 2026-11-03).
    LED auf dem Board ist an.
  - 1× **PmodGYRO** (Digilent, 3-Achsen-Gyroskop), über **I2C** angeschlossen.
    **Defekt** (Stand 2026-11-03). Ersatz: **gleiches Modell (PmodGYRO)** wird nachbestellt. Bis dahin kein Gyro.
    Anschluss: **J2** (nicht J1!), Reihenfolge SCL, SDA, GND, VCC (Pin 1 = eckiges Pad) →
    Pi Pin 5 (SCL), Pin 3 (SDA), Pin 6 (GND), Pin 1 (3,3 V). Beide J2-Reihen sind gleich belegt.
    Erwartet: Adresse 0x69, WHO_AM_I (Register 0x0F) = 0xD3.
  - 4er-Tastermodul (GND, K1–K4): K1 = GPIO 17, K2 = GPIO 27, K3 = GPIO 22, K4 = GPIO 23.
    Schaltet gegen GND, interne Pull-ups (gedrückt = LOW). Test: `python3 -m tools.test_buttons`
    **Belegung am 2026-11-03 geprüft** (K1–K4 nacheinander gedrückt, korrekt erkannt, kein Prellen).
    **Funktion (vom Projektinhaber entschieden):** K1 = Start, K2 = Stopp, K3 = Lack of Progress, K4 = frei.
    (Hinweis auf Regel 4.2.8 – ein Taster für Start und LoP – wurde gegeben; Projektinhaber bleibt bei dieser Belegung.)
  - 3× Ultraschall **RCWL-9610A**: vorne, links, rechts. **GPIO-Modus** (Trig/Echo, wie HC-SR04),
    Versorgung **3,3 V** (Echo dann 3,3 V, kein Spannungsteiler). Pins in `config.py` (`US_PINS`):
    vorne Trig 5 / Echo 6, links Trig 26 / Echo 20, rechts Trig 19 / Echo 16.
    Datenblatt: 2,8–5,5 V, blinder Bereich ~2 cm, Messzyklus ~70 ms (GPIO), I2C-Adresse fest 0x57 (daher kein I2C).
    **Messung mit `lgpio`** (nicht `gpiozero.DistanceSensor`, das hängt ohne Echo): Sensoren nacheinander,
    Echo-Flanken mit Kernel-Zeitstempel, Timeout 50 ms = kein Echo. Test: `python3 -m tools.test_ultrasonic`
    Stand 2026-11-03: **vorne** misst (stabil ±0,5 cm), **links** geprüft (3–68 cm, folgt der Hand,
    ~2 % ohne Echo, vereinzelte Ausreißer), **rechts** geprüft (14–66 cm, ~4 % ohne Echo;
    Nahbereich < 14 cm noch nicht getestet).
    **Einbau:** vorne mittig, 3 cm hinter der Vorderkante; links und rechts in der Längsmitte,
    je 2 cm von der Seitenkante nach innen.
- Kamera vorhanden, Nutzung noch offen
- Sprache: Python

## Projektstruktur (vom Projektinhaber freigegeben)

```
robocup/
├── CLAUDE.md
├── config.py              # alle Einstellungen: Pins, Maße, Schwellwerte, Zeiten
├── main.py                # Roboterprogramm: wartet auf K1, fährt den Wettkampflauf
├── robocup-web.service    # systemd: Webseite beim Booten starten
├── robocup-robot.service  # systemd: main.py beim Booten starten
├── hardware/              # spricht direkt mit der Hardware
│   ├── motors.py          #   BrickPi3: fahren, drehen
│   ├── ultrasonic.py      #   3× Ultraschall
│   ├── color_sensor.py    #   Farbsensor unten
│   ├── buttons.py         #   Taster K1–K4
│   └── robot.py           #   echter Roboter: setzt navigation/robot_interface.py um
├── navigation/            # reine Logik, kennt keine Hardware
│   ├── maze_map.py        #   die Karte
│   ├── pathfinding.py     #   Wegplanung
│   ├── explorer.py        #   Erkundungslogik
│   └── robot_interface.py #   Schnittstelle Explorer <-> Roboter (Simulation und echter Roboter)
├── simulation/            # Erkundung ohne echten Roboter testen
│   ├── sim_robot.py
│   ├── maze_generator.py  #   Zufallslabyrinthe nach Regelwerk 2026
│   └── mazes/             #   Testlabyrinthe als JSON
├── web/                   # Webseite (flach)
│   ├── server.py          #   aiohttp: liefert Seite + WebSocket
│   ├── index.html
│   ├── app.js
│   └── style.css
├── tools/                 # Testprogramme (Motoren, Sensoren, Farbkalibrierung)
├── tests/                 # automatische Tests der Logik (unittest)
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
- Webserver: **aiohttp** (apt: `python3-aiohttp`), liefert Seite und **WebSocket (live)** auf einem Port.
- Erreichbar **auch von anderen Geräten** im Netz (Server lauscht auf `0.0.0.0`).
- Funktionen der Seite: Start / Pause / Schritt, Geschwindigkeit, Labyrinth auswählen,
  Karte vs. Wirklichkeit (Unbekanntes ausgegraut, Grenzkacheln und geplanter Weg markiert),
  Sensorwerte.
- Bodentypen: normal, schwarz, blau, silber, rot (Eingang Gefahrenzone). Keine Rampen, keine Opfer.
- Sensorwerte zunächst **exakt** (Rauschen kommt später).
- **8-Minuten-Zeitlimit mit Sim-Zeit:** Jede Aktion kostet eine geschätzte Zeit (Werte in `config.py`),
  der Roboter fährt rechtzeitig heim.
- Testlabyrinthe als **JSON**: `width`, `height`, `start`, `tiles` mit `x`, `y`, `walls`, `floor`
  (Format siehe unten).
- Tests mit **unittest** (Standardbibliothek).
- Die Simulation arbeitet kachelweise (eine Aktion = eine Kachel bzw. eine 90°-Drehung).
  Simulierte Ultraschallwerte = Abstand von der Sensorposition (Robotermaße in `config.py`) bis zur nächsten Wand.
- Alle Browser sehen dieselbe Simulation (gemeinsamer Zustand auf dem Server).

### Echter Roboter (freigegeben 2026-11-03)

- **Bibliotheken:** `brickpi3` (Motoren), `smbus2` (Farbsensor), `lgpio` (Ultraschall), `gpiozero` (Taster).
- **Geradeaus:** PD-Regler (geändert 2026-11-03, Vorgabe Projektinhaber):
  - **Grundregel: links und rechts gleich** (Abweichung = (links − rechts) / 2). Keine feste
    Wandgrenze (z.B. 20 cm) beim Lenken. Die 20-cm-Grenze gilt nur für die Karte (Messung in Kachelmitte).
  - **Gangbreite B = links + rechts** wird mitgeführt; sie ändert sich beim seitlichen Wandern nicht,
    nur wenn sich eine Wand ändert.
  - **Breite Räume** (B deutlich größer als ein Gang einer Kachel): Abstand zur **näheren** Wand
    = halbe zuletzt gemessene Gangbreite (bei weiter entfernter Wand plus ganze Kacheln), damit er
    mittig auf seiner Kachel bleibt.
  - **Gegen Ausscheren, wenn eine Wand wegbricht:**
    1. Breiten-Wächter: weicht links + rechts um mehr als ca. 1 cm von B ab, wird die Seite, die sich
       gerade geändert hat, ignoriert (seitliches Wandern löst das nie aus).
    2. Eine Seite fehlt: Abstand zur verbliebenen Wand = B / 2 (fährt in derselben Spur weiter).
    3. Beide fehlen: Ketten über die Encoder gleich halten.
    4. Zurückkehrende Wand zählt erst nach 2 zusammenpassenden Messungen, dann neue Breite B.
    5. Lenkänderung begrenzt (pro gefahrenem cm), damit Fehlmessungen nie einen Ruck geben.
    6. D-Anteil beginnt nach jedem Wechsel neu.
  - Der D-Anteil nutzt die Änderung der Abweichung pro gefahrenem cm (erkennt Schrägstand).
- **Kachelende nach Abstand:** Ist vorne eine Wand in Reichweite (auch mehrere Kacheln entfernt),
  wird angehalten, wenn der Abstand vorne zur Kachelmitte passt (zwischen zwei Messungen mit Encoder
  weitergerechnet). Ohne Wand in Reichweite: nach Encoder-Strecke. Encoder begrenzt Ausreißer.
- **Drehen:** über die Encoder, danach **an den Wänden ausrichten**:
  Wand vorne → langsam dagegen fahren, bis die Ketten stehen, dann zurück auf Kachelmitte.
  Sonst Wand hinten (laut Karte) → rückwärts dagegen, dann vor auf Kachelmitte.
  (Verworfen: Minimum des Abstands vorne suchen. Der breite Schallkegel liefert immer den
  senkrechten Abstand, Schrägstand ist im Stand nicht messbar.)
- **Farbsensor:** Kalibrierwerkzeug in `tools/` misst Weiß/Schwarz/Blau/Silber/Rot auf dem echten
  Boden und speichert nach `data/`. Beim Fahren wird die nächstliegende kalibrierte Farbe gewählt.
- **Exit-Signal:** wird ignoriert (keine Blinkhardware), der Roboter bleibt am Start einfach stehen.
- **Ablauf:**
  - Beim Booten startet `main.py` (systemd) und wartet auf **K1** (Start).
  - **K2** (Stopp): Lauf abbrechen, Motoren aus, zurück in den Zustand wie nach dem Booten
    (Karte leer, wartet auf K1). Das Programm selbst läuft weiter.
  - **K3** (Lack of Progress): Roboter hält an und wartet. Nach dem Umsetzen nochmal **K3** → weiter.
    Er nimmt an, dass er auf dem letzten besuchten Checkpoint steht, mit der Blickrichtung, mit der er
    dort zuletzt angekommen ist. Ohne Checkpoint: am Start, Blickrichtung wie zu Beginn des Laufs.
    Die Karte bleibt.
- **Zwei Programme**, beide per systemd beim Booten gestartet:
  `robocup-web.service` (Webseite, `python3 -m web.server`) und `robocup-robot.service` (`main.py`).
  `main.py` verbindet sich per **WebSocket** mit `ws://localhost:8000/robot` und schickt jede Änderung.
  Der Roboter fährt auch, wenn die Webseite nicht läuft, und verbindet sich später neu.
- **Webseite, Ansicht „Roboter“:** Live-Karte des echten Roboters, aktuelle Aktion, erkannte Dinge
  (Wände, Boden, Checkpoints), Sensorwerte, Ereignisse. Daneben bleibt die Ansicht „Simulation“.
- **Gefahrenzone beim echten Roboter:** auf der Webseite vor dem Lauf wählbar, sonst `config.RED_MODE`.
- **Akku und Verbrauch:** Der BrickPi3 misst nur Spannungen, keinen Strom. Die Webseite zeigt die
  **Akkuspannung in V** (oben in der Leiste, Warnung unter `config.BATTERY_LOW_V`) und statt des Stroms
  die **Motorleistung in % pro Kette** (vom BrickPi3, kein echter Stromwert). Ein Strommodul
  (z.B. INA226) wurde angeboten, ist aber vorerst nicht gewünscht.

### Gestaltung der Webseite

- **Schlicht**, soll nicht nach typischer KI-Seite aussehen.
- Farben aus dem echten Feld: weiße Kacheln, erkannte Wände in **Holzton**, Roboter **LEGO-gelb**
  mit dunklem Pfeil, Bodenfarben schwarz/blau/silber/rot. Oberfläche kühles Grau, Schiefer-Dunkel als Text-/Knopffarbe.
- Keine Web-Schriften (Pi evtl. ohne Internet), Systemschrift, Zahlen mit fester Breite.
- Keine Karten mit Schatten, keine Großbuchstaben-Überschriften, keine Monospace-Labels, keine Verläufe.
- Hell- und Dunkelmodus, auf dem Handy einspaltig, reduzierte Bewegung wird respektiert.

### Zufallslabyrinthe (`simulation/maze_generator.py`)

- Erzeugt nach dem **Regelwerk 2026** (eine Ebene): Gänge mit Sackgassen und Schleifen,
  Foyers (offene Räume), freistehende Wände (Inseln/Floating Tiles), Gefahrenzone
  (komplett ummauert, rote Eingangskachel, blockiert nichts, Start nicht darin), Start beliebig.
- **Größe frei einstellbar** auf der Webseite; offizielle 2026-Feldgrößen als Vorschlag
  (8×7, 8×8, 9×8, 7×8, 4×12).
- **Anzahl** schwarz/blau/silber/Gefahrenzonen auf der Webseite einstellbar.
  Schwarze Kacheln werden nur so gesetzt, dass alle anderen Kacheln erreichbar bleiben („lösbar“).
- **Nicht** erzeugt (spätere Phase): Bodenwellen, Hindernisse, Treppen, Brücken, Rampen, Opfer.
- **Seed** für jedes Labyrinth (wiederholbar) + **Speichern-Knopf** (JSON in `simulation/mazes/`).

### Lauf-Regeln in der Simulation

- **Gefahrenzone:** Verhalten **vor jedem Lauf auf der Webseite wählbar**: zuletzt erkunden / meiden / normal.
- **Exit:** Am Start **10 s blinken** (Regel 5.6.12). Zählt zur Sim-Zeit und wird beim Heimweg eingeplant.
- **Blau:** Bereits besuchte blaue Kacheln werden bei der Wegplanung **stark gemieden** (2026: −10 Punkte je Wiederholung).
- **Kalibrieren:** **40 s** der 8 Minuten gelten als verbraucht (Uhr startet bei 0:40).
- **Punkte nach Regelwerk 2026** werden live angezeigt.

### Regelwerk 2026: relevante Fakten

Quelle: [RCJ Rescue Maze Rules 2026](https://junior.robocup.org/wp-content/uploads/2026/02/RCJRescueMaze2026-final.pdf)
(Stand 2026-03-29), Wettkampffelder: [rescue.rcj.cloud 2026](https://rescue.rcj.cloud/events/2026/RoboCup2026/maze/practice)

- Kachel 30 × 30 cm, Wände ≥ 15 cm hoch, 30 cm lang, auf Kachelkanten; Durchgang 28 cm (3.1, 3.3)
- Schwarz = Loch; „besucht“ (mehr als halber Roboter drauf) = Lack of Progress (3.2.3, 5.4.4, 5.5)
- Blau: 5 s am Stück stehen, bevor die nächste Kachel besucht wird; sonst LoP (3.2.5, 5.5)
- Silber = Checkpoint, 10 Punkte (3.2.4, 5.6.11)
- Rot = Eingang Gefahrenzone; ummauert, blockiert nichts, Start nicht darin (3.5)
- Start beliebig im Feld (3.3.5); 8 Minuten pro Spiel inkl. Kalibrieren (5.3.1)
- Punkte (ohne Opfer/Rampen/Treppen/Bodenwellen):
  - Checkpoint 10 je Kachel
  - Blaue Kachel: 30 beim ersten Besuch, −10 je weiterem Besuch derselben Kachel, min. 0 (5.6.6)
  - Zuverlässigkeitsbonus = SVI×10 + SRD×10 + SBV×10 − LoP×15, nicht negativ (5.6.7)
  - Exit-Bonus = SVI×10 + SBV×10 + SSN×5 + SRN×5, wenn am Start 10 s geblinkt (5.6.12)
  - Auslegung: SBV = Anzahl blauer Kacheln, die genau einmal besucht wurden
- Offizielle Felder 2026: 4×12 bis 9×8, ca. 44 Kacheln auf 2–3 Ebenen,
  1–4 Checkpoints, 1–3 schwarz, 2–6 blau, 1–2 rot

### JSON-Format der Testlabyrinthe

```json
{
  "width": 4, "height": 3,
  "start": [0, 0],
  "start_dir": "N",
  "tiles": [
    {"x": 0, "y": 0, "walls": "SW", "floor": "normal"},
    {"x": 1, "y": 1, "walls": "",   "floor": "black"}
  ]
}
```

- `x` nach rechts (Osten), `y` nach oben (Norden), (0, 0) = unten links. Wie in der Roboterkarte.
- `walls`: Buchstaben N/E/S/W. Eine Wand gilt, wenn **eine** der beiden Nachbarkacheln sie einträgt.
- Ränder des Labyrinths sind immer Wand.
- `floor`: `normal`, `black`, `blue`, `silver`, `red`. `start_dir` (optional, Standard `N`): Startblickrichtung.
- Optional bei Zufallslabyrinthen: `seed` und `params` (Einstellungen des Generators).

## Befehle (im Projektordner ausführen)

- Taster testen: `python3 -m tools.test_buttons`
- Ultraschall testen: `python3 -m tools.test_ultrasonic`
- Farbsensor kalibrieren: `python3 -m tools.calibrate_color` (nur Live-Anzeige: `--test`) → `data/colors.json`
- Motoren einmessen (Roboter fährt!): `python3 -m tools.test_motors richtung|strecke|drehen`
  → Werte für `MOTOR_DIRECTION`, `MOTOR_DEG_PER_CM`, `MOTOR_DEG_PER_90` in `config.py`

- Webseite von Hand starten: `python3 -m web.server`, dann im Browser `http://<Pi-IP>:8000`
- Roboterprogramm von Hand starten: `python3 main.py` (wartet auf K1)
- Autostart einrichten (einmalig, braucht sudo):
  `sudo systemctl enable --now /home/immo/robocup/robocup-web.service /home/immo/robocup/robocup-robot.service`
- Dienste: `systemctl status robocup-robot`, Ausgaben `journalctl -u robocup-robot -f`,
  nach Codeänderungen `sudo systemctl restart robocup-robot robocup-web`
  (Läuft der Dienst, belegt er GPIO/BrickPi: vor `tools.*` oder `python3 main.py` erst `sudo systemctl stop robocup-robot`.)
- Tests: `python3 -m unittest discover -s tests`

## Offene Fragen an den Projektinhaber


## Bekannte Stolpersteine

- I2C ist eingeschaltet (`dtparam=i2c_arm=on`), Bus 1 = GPIO 2 (SDA) / GPIO 3 (SCL).
- Gyro-Reihe an J2 verdreht gesteckt → SDA hängt auf LOW, der ganze I2C-Bus (auch Farbsensor) ist blockiert.
  Prüfen mit `pinctrl get 2,3`: im Ruhezustand müssen beide `hi` sein.

- `lgpio` (Backend von `gpiozero` auf dem Pi 5) legt Dateien im aktuellen
  Arbeitsverzeichnis an. Programme müssen aus einem beschreibbaren Ordner gestartet werden.
- RCWL-9610A nur an 3,3 V betreiben: an 5 V wäre auch das Echo-Signal 5 V → würde einen Spannungsteiler brauchen.
