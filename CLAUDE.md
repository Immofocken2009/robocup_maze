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
  (dort sitzt der Motor). Ports (geändert 2026-11-04): **B = links, C = rechts**.
  Beide am 2026-11-04 geprüft: laufen bei positiver Drehzahl rückwärts, daher `MOTOR_DIRECTION` = −1 für beide
  (danach je Kette über `hardware/motors.py` vorwärts bestätigt).
  **Strecke eingemessen 2026-11-04:** `MOTOR_DEG_PER_CM` = **18,29** (Fahrt nach Encoder mit halbem Tempo:
  1802 Encodergrad = 98,5 cm; vorher geschätzt 30). Dabei ca. **10° nach rechts gedriftet**, obwohl beide
  Encoder gleich liefen (1802 / 1801), siehe Offene Fragen.
  **Drehen eingemessen 2026-11-04:** `MOTOR_DEG_PER_90` = **345** (360° am Stück, normales Tempo, ohne Bremse:
  1430 Encodergrad, ca. 10–15° zu weit, also ca. 372°). Danach nur 2 Encodergrad ausgerollt (ca. 0,5°).
  Frühere Messungen: 297 (4 × 90° mit Langsamwerden: 1090 Grad = ca. 330°), 357 (360° am Stück mit Bremse:
  1250 Grad = ca. 315°; dabei lief links 1311, rechts 1188). Die Ketten rutschen beim Drehen also je nach
  Fahrweise sehr unterschiedlich. **Bestätigt 2026-11-04:** 360° am Stück mit 345 (1381 Encodergrad) endet genau in
  Startrichtung. Mit 4 × 90° wie im Lauf noch nicht geprüft.
  Die Strecke (`MOTOR_DEG_PER_CM`) wurde noch mit Langsamwerden vor dem Ziel eingemessen. Nachmessen.
- **Maße (gemessen 2026-11-04):** außen über alles mit überstehenden Teilen **16,5 cm lang, 19,5 cm breit**.
  Ketten mittig unter dem Roboter (Drehmitte = Mitte), **Kettenmitten 15 cm auseinander**,
  **Ketten von Achse zu Achse 10,5 cm lang** (beide gleich). Wird im Code nicht gebraucht: wie weit die Ketten
  beim Drehen rutschen, steckt im eingemessenen `MOTOR_DEG_PER_90` (ohne Rutschen wären es π/4 · 15 cm ·
  `MOTOR_DEG_PER_CM`, mit Rutschen mehr).
  „Vorderkante“ = vorderster Punkt = Vorderseite des vorderen Ultraschalls (ragt 1 cm über das Gehäuse).
  Beim Drehen auf der Stelle beschreiben die Ecken einen Kreis mit 12,8 cm Radius; bis zur Wand sind es von der
  Kachelmitte 14 cm, es bleiben also nur ca. 1,2 cm Luft (steht er nicht mittig, streift er).
- Sensoren an den GPIOs des Pi:
  - 1× RGB-Farbsensor nach unten, **vorne mittig, 1,5 cm hinter der Vorderkante, 1,5 cm über dem Boden**.
    **Bestätigt:** erkennt die Farbe einer Kachel, bevor der Roboter zur Hälfte drauf ist
    (also bevor sie als „befahren“ zählt, Regel 5.4.4). Schwarz/Rot: erkennen, anhalten, zurück.
    **TCS34725** bestätigt (I2C-Bus 1, Adresse 0x29, ID 0x44). Liefert Werte (Stand 2026-11-03).
    LED auf dem Board ist an.
  - **Gyro: MPU-6050** auf dem Board **DEBO SENS 3AXISH** (reichelt, 3-Achsen-Gyro + 3-Achsen-Beschleunigung),
    **eingeplant statt des PmodGYRO** (entschieden 2026-11-03). **Noch nicht angeschlossen, Code noch nicht
    umgesetzt** (siehe „Gyro MPU-6050 – NOCH UMZUSETZEN“).
    Anschluss geplant an I2C-Bus 1, parallel zum Farbsensor: VCC → Pin 1 (3,3 V), GND → Pin 6 oder 9,
    SCL → Pin 5 (GPIO 3), SDA → Pin 3 (GPIO 2); AD0, XDA, XCL, INT frei lassen.
    Erwartet: Adresse **0x68**, WHO_AM_I (Register 0x75) = 0x68 (Nachbau-Chips melden evtl. anderes).
    Prüfen: `i2cdetect -y -r 1` muss 0x29 (Farbsensor) und 0x68 zeigen.
  - Früher: **PmodGYRO** (Digilent) an J2, Adresse 0x69, **defekt**. Wird nicht mehr verwendet.
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
    **Einbau (gemessen 2026-11-04):** vorne mittig an der Vorderkante, 7 cm hoch; links und rechts vorne,
    1 cm hinter der Vorderkante, je 2 cm von der Seitenkante nach innen, 10 cm hoch (rechts gespiegelt zu links).
    Die Seitensensoren sitzen damit 7,25 cm vor der Drehmitte (`US_SIDE_AHEAD_CM`).
    **Zuordnung am 2026-11-04 geprüft** (Hand nacheinander vor vorne, links, rechts: jeweils nur der richtige
    Sensor reagiert, über die Werte des laufenden Dienstes auf der Webseite).
- Lüfter am **Fan-Anschluss des Pi 5** (Kernel-Gerät `pwmfan`, Drehzahl lesbar in U/min).
- Kamera vorhanden, Nutzung noch offen
- Sprache: Python

## Projektstruktur (vom Projektinhaber freigegeben)

```
robocup/
├── CLAUDE.md
├── config.py              # alle Einstellungen: Pins, Maße, Schwellwerte, Zeiten
├── main.py                # Roboterprogramm: wartet auf K1, fährt den Wettkampflauf
├── call.py                # Programme aufrufen: --ultrasonicL/R/V = Ultraschall links/rechts/vorne kalibrieren,
│                          #   --rgb [farbe ...] = Farbsensor einmessen; später auch Gyro (GEPLANT)
├── robocup-web.service    # systemd: Webseite beim Booten starten
├── robocup-robot.service  # systemd: main.py beim Booten starten
├── hardware/              # spricht direkt mit der Hardware
│   ├── motors.py          #   BrickPi3: fahren, drehen
│   ├── ultrasonic.py      #   3× Ultraschall
│   ├── color_sensor.py    #   Farbsensor unten
│   ├── buttons.py         #   Taster K1–K4
│   ├── fan.py             #   Lüfter am Fan-Anschluss, CPU-Temperatur
│   ├── run_log.py         #   Aufzeichnung eines Laufs (Hardware-Grenze + Rechenwerte), Lesen
│   ├── gyro.py            #   MPU-6050: Drehrate aus dem FIFO, Nullpunkt, Winkel  (GEPLANT, noch umzusetzen)
│   └── robot.py           #   echter Roboter: setzt navigation/robot_interface.py um
├── navigation/            # reine Logik, kennt keine Hardware
│   ├── maze_map.py        #   die Karte
│   ├── pathfinding.py     #   Wegplanung
│   ├── explorer.py        #   Erkundungslogik
│   └── robot_interface.py #   Schnittstelle Explorer <-> Roboter (Simulation und echter Roboter)
├── simulation/            # Erkundung ohne echten Roboter testen
│   ├── sim_robot.py       #   Labyrinth, Steuerung, kachelweiser Roboter (nur für die Tests)
│   ├── sim_hardware.py    #   Fahrphysik: nachgebildete Ketten, Ultraschall, Farbsensor, Sim-Uhr
│   ├── maze_generator.py  #   Zufallslabyrinthe nach Regelwerk 2026
│   └── mazes/             #   Testlabyrinthe als JSON
├── web/                   # Webseite (flach)
│   ├── server.py          #   aiohttp: liefert Seite + WebSocket
│   ├── index.html
│   ├── app.js
│   └── style.css
├── tools/                 # Testprogramme (Motoren, Sensoren, Farbkalibrierung), calibrate_ultrasonic.py
│                          #   (Ultraschall kalibrieren), replay_run.py (Lauf nachrechnen),
│                          #   test_gyro.py (GEPLANT, noch umzusetzen)
├── tests/                 # automatische Tests der Logik (unittest)
├── data/                  # Kalibrierwerte (colors.json, ultrasonic.json)
└── logs/                  # Aufzeichnungen der echten Läufe (ein Lauf = eine .jsonl.gz-Datei, nicht in Git)
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
- **Fahrphysik mit dem echten Fahrcode** (seit 2026-11-03): Der simulierte Roboter hat eine stufenlose
  Position und Blickrichtung und fährt mit `hardware/robot.py` (PD-Lenkung, Breiten-Wächter, Halt nach
  Abstand, Schwarz/Rot erkennen). Nur Motoren, Ultraschall, Farbsensor und Uhr sind in
  `simulation/sim_hardware.py` nachgebildet. Änderungen am Fahrcode wirken so automatisch in der Simulation.
  - Ketten: Differentialantrieb, Wandkontakt pro Kette (Ketten blockieren an Wänden).
  - Ultraschall: Schallkegel (nächster Wandpunkt im Kegel), dadurch kriecht der Wert an Wandkanten hoch wie echt.
  - **Messrauschen** (Ultraschall) und **Kettenschlupf** auf der Webseite an/aus schaltbar.
  - Sim-Uhr: `RealRobot` bekommt eine austauschbare Uhr; die Simulation läuft in Sim-Zeit (Tempo einstellbar).
  - „Schritt“ = eine Aktion des Roboters (Kachel fahren, drehen, messen, warten).
  - Die Webseite zeigt **nur noch die Fahrphysik**. Der kachelweise `SimRobot` bleibt nur für die
    automatischen Tests (schnell, viele Zufallsläufe).
- **8-Minuten-Zeitlimit mit Sim-Zeit:** Jede Aktion kostet eine geschätzte Zeit (Werte in `config.py`),
  der Roboter fährt rechtzeitig heim.
- Testlabyrinthe als **JSON**: `width`, `height`, `start`, `tiles` mit `x`, `y`, `walls`, `floor`
  (Format siehe unten).
- Tests mit **unittest** (Standardbibliothek).
- Kachelweiser `SimRobot` (Tests): eine Aktion = eine Kachel bzw. eine 90°-Drehung, Ultraschall =
  Abstand von der Sensorposition (Robotermaße in `config.py`) bis zur nächsten Wand.
- Alle Browser sehen dieselbe Simulation (gemeinsamer Zustand auf dem Server).

### Echter Roboter (freigegeben 2026-11-03)

- **Bibliotheken:** `brickpi3` (Motoren), `smbus2` (Farbsensor), `lgpio` (Ultraschall), `gpiozero` (Taster).
- **Drehzahlen beim Einmessen umrechnen** (entschieden 2026-11-04): Ändert sich `MOTOR_DEG_PER_CM`, werden alle
  Werte in `config.py`, die in Drehzahl (dps) oder Encodergrad angegeben sind, mit demselben Faktor umgerechnet
  (Fahr-/Drehtempo, `STALL_DEG`, `STEER_KP/KD/MAX/SLEW`, Schätzung `MOTOR_DEG_PER_90`;
  `STEER_ENC_K` bleibt). So fährt der Roboter körperlich wie in der Simulation eingestellt (ca. 13 cm/s).
- **Kein Langsamwerden vor dem Ziel** (entschieden 2026-11-04, Vorgabe Projektinhaber): Fahren und Drehen immer mit
  normalem Tempo (`DRIVE_SPEED_DPS`, `TURN_SPEED_DPS`) bis zum Ziel, dann Motoren stromlos (ausrollen).
  (Vorher: letzte 5 cm bzw. letzte Encodergrad einer Drehung langsam.) Nach jeder Aktion hält er weiterhin an.
- **Nirgends bremsen, außer der Projektinhaber sagt es** (entschieden 2026-11-04): statt `motors.stop()` (aktive
  Bremse) immer `motors.float()` (stromlos, rollt aus), auch bei K2/K3, Zeitende, schwarz/rot erkannt, blockiert,
  Warten und in `tools/test_motors.py`. `Motors.stop()` bleibt nur für den Fall, dass er es anordnet.
- **Abschalten sicher machen** (entschieden 2026-11-04): `Motors.float()` schickt „stromlos“ für beide Ports in
  **einer** Nachricht an den BrickPi3, liest danach den Status nach (`MOTOR_FLOAT_CHECK_S`) und wiederholt den
  Befehl, bis beide stromlos melden (höchstens `MOTOR_FLOAT_TRIES`, sonst Warnung im Log). Grund: einzelne
  Schreibbefehle an den BrickPi3 gehen verloren (siehe Bekannte Stolpersteine); einmal lief dadurch nach einer
  Drehung die linke Kette mit vollem Tempo weiter.
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
    **D-Anteil nur, wenn beide Seiten zählen.** Zählt nur eine Wand, lenkt er nur mit P
    (sonst hält er das langsame Hochkriechen einer wegbrechenden, v.a. weit entfernten Wand für
    Schrägstand und schert aus; Nachteil: an einer einzelnen Wand pendelt er eher).
- **Kachelende nach Abstand:** Ist vorne eine Wand in Reichweite (auch mehrere Kacheln entfernt),
  wird angehalten, wenn der Abstand vorne zur Kachelmitte passt (zwischen zwei Messungen mit Encoder
  weitergerechnet). Ohne Wand in Reichweite: nach Encoder-Strecke. Encoder begrenzt Ausreißer.
  Der Haltepunkt wird **bei jeder Messung vorne neu** ins Kachelraster gelegt (die Kachelmitte, die am
  nächsten an 30 cm liegt). Grund (aus der Fahrphysik-Simulation): der Sensor hört oft erst ein Wandende
  schräg voraus und später die eigentliche Wand; ein einmal am Start festgelegtes Ziel passte dann nicht.
- **Ketten blockiert:** Drehen sich die Ketten trotz Fahrbefehl `STALL_S` lang nicht (z.B. schräg an einer
  Wand), bricht die Fahrt ab: zurück und als Wand melden (Explorer plant neu). Drehen und Zurückfahren
  brechen ebenso ab, statt bis zum Zeitende zu drücken.
- **Drehen:** über die Encoder. **Kein Ausrichten an Wänden** (geändert 2026-11-03, Vorgabe Projektinhaber):
  weder vorne andrücken noch rückwärts an die Wand hinten fahren. Grund: an der Wand drehen die Ketten
  durch, die Encoder laufen weiter, der Stillstand wird nicht erkannt und er hängt sich auf.
  Schrägstand korrigiert die Lenkung beim nächsten Fahren an den Seitenwänden.
  (Verworfen: Andrücken vorne/hinten; umdrehen und vorne andrücken; Minimum des Abstands vorne
  suchen, weil der breite Schallkegel immer den senkrechten Abstand liefert.)
- **Schrägstand aus den Seitenwänden** (entschieden 2026-11-03, statt Ausrichten): Beim Fahren mit zwei
  Wänden (Gang) ändert sich (links − rechts) / 2 pro gefahrenem cm um den Schrägwinkel. Daraus wird per
  Ausgleichsgerade geschätzt, wie schräg er am Kachelende steht (was die Lenkung selbst gedreht hat,
  rechnen die Encoder heraus). Am Kachelende dreht er sich um diesen Winkel auf der Stelle gerade
  (ab `HEADING_FIX_MIN_DEG`, höchstens `HEADING_FIX_MAX_DEG`). Eine oder keine Wand: keine Korrektur.
  Kein Wandkontakt. **Hebel herausgerechnet** (entschieden 2026-11-04): die Seitensensoren sitzen vor der
  Drehmitte, dreht die Lenkung, verschiebt das die Messung sofort um `US_SIDE_AHEAD_CM` · sin(Drehung laut
  Encoder); das wird vor der Ausgleichsgeraden abgezogen. **Wird durch das Gyro ersetzt, sobald es umgesetzt ist** (dann nur noch Rückfall ohne Gyro).
- **Gyro MPU-6050 – NOCH UMZUSETZEN** (Art und Weise entschieden 2026-11-03, Code noch nicht gebaut):
  - **Dateien:** `hardware/gyro.py` (Treiber) und `tools/test_gyro.py` (Live-Anzeige; findet Achse und Vorzeichen,
    wenn man den Roboter von Hand dreht; misst den Skalierungsfaktor). Ergebnisse als Werte in `config.py`
    (Einbaulage = Achse + Vorzeichen, Skalierungsfaktor).
    **Auch über `call.py` aufrufbar** (Vorgabe Projektinhaber 2026-11-04), wie Ultraschall und Farbsensor;
    Name des Schalters noch festlegen (z.B. `--gyro`).
  - **Bibliothek:** `smbus2` direkt (Register selbst lesen, wie beim Farbsensor), keine pip-Bibliothek.
  - **Auslesen über den FIFO des Sensors:** Der MPU-6050 misst selbst im festen Takt (vorgesehen 200 Hz, nur die
    Drehrate der Hochachse, Messbereich ±500 °/s = 65,5 pro °/s) und puffert (1024 Byte, bei 2 Byte je Messung
    ca. 2,5 s). Der Fahrcode holt bei jeder Abfrage alle neuen Messungen ab und summiert mit exakt bekanntem
    Zeitabstand zum Winkel. Kein eigener Thread. Muss also mindestens alle ca. 2 s abgefragt werden (auch beim
    Warten und im Stand); Pufferüberlauf erkennen und melden.
  - **Skalierung:** mit `tools/test_gyro.py` einmessen: Roboter von Hand genau eine volle Umdrehung drehen (z.B. an
    einer Kante ausgerichtet), das Programm berechnet den Faktor für `config.py` (Gyros weichen bis ca. ±3 % ab).
  - **Nullpunkt** (Drehrate im Stillstand): laufend beim Warten auf K1 messen und im Lauf bei jedem Stillstand
    nachführen (Wände messen, blau warten, ...), wenn die Encoder stehen. Kostet keine Laufzeit, gleicht
    Temperaturdrift aus.
  - **Nutzung:**
    1. **Drehungen auf 90° nach Gyro** statt nach Encoder (behebt den Kettenschlupf beim Drehen).
    2. **Blickrichtung über den ganzen Lauf** mitführen: jede Drehung zielt auf die nächste volle 90°-Richtung,
       Fehler summieren sich nicht. Am Kachelende auf die volle Richtung geradedrehen; **ersetzt die Schätzung
       aus den Seitenwänden** (die bleibt nur als Rückfall ohne Gyro).
    3. **Geradeaus ohne Seitenwände:** Richtung per Gyro halten statt beide Ketten per Encoder gleich.
    4. **Später (Rampen):** Neigung über den Beschleunigungssensor erkennen. Nur vorgemerkt.
  - **Ohne Gyro oder bei Ausfall:** weiterfahren wie bisher (Drehen nach Encoder, Schrägstand aus den
    Seitenwänden, geradeaus nach Encoder) und Warnung auf der Webseite. K1 startet auch ohne Gyro.
  - **Nach Lack of Progress:** beim zweiten K3 gilt die aktuelle Gyro-Richtung als genau ausgerichtet (der Roboter
    wurde von Hand gerade hingestellt). Die Karte bekommt wie bisher die Richtung vom Checkpoint.
  - **Aufzeichnung:** die FIFO-Abfragen gehören zur Hardware-Grenze; in `hardware/run_log.py` (`Recorder.attach`)
    mitschreiben und in `tools/replay_run.py` nachbilden, damit das Nachrechnen 1:1 bleibt.
- **Ultraschall-Kalibrierung** (entschieden 2026-11-04): je Sensor **Faktor und Versatz**, richtig = Faktor ×
  gemessen + Versatz (Abstand von der Vorderseite des Sensors). Einmessen mit `tools/calibrate_ultrasonic.py`:
  ebenes Brett nacheinander in 5, 10, 20, 40 cm (`US_CALIBRATION_CM`) vor jeden Sensor, je 15 Messungen (Median),
  Ausgleichsgerade; Hinweis bei mehr als 1 cm Abweichung. Gespeichert in `data/ultrasonic.json` (mit den
  Messpunkten). Einzelne Sensoren neu kalibrierbar, die anderen bleiben. `hardware/ultrasonic.py` korrigiert jede
  Messung. Ohne Kalibrierung (oder für einen fehlenden Sensor): Rohwerte und Warnung auf der Webseite.
  Die Kalibrierung steht im Kopf jeder Aufzeichnung. Die Simulation misst ohne Fehler und braucht keine.
- **Farbsensor einmessen** (entschieden 2026-11-04): `call.py --rgb` misst alle fünf Bodenfarben (Weiß, Schwarz, Blau,
  Silber, Rot; Blau/Silber/Rot überspringbar), einzelne Farben können neu eingemessen werden, danach Live-Erkennung.
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
- **Lüfter (entschieden 2026-11-03):** **eigene Regelung in Python** (`hardware/fan.py`, eigener Thread in
  `main.py`, alle `FAN_INTERVAL_S`). Stufen ab **45 / 52 / 60 / 68 °C** (Leistung 75 / 125 / 175 / 250 von 255,
  wie die Kernel-Stufen), 5 °C Hysterese. Lesefehler oder ab 75 °C: volle Leistung.
  Der Kernel kann auf diesem Pi nicht auf Fremdsteuerung umgestellt werden (nur `step_wise`). Darum schaltet
  `robocup-robot.service` beim Start die Kernel-Regelung ab (`thermal_zone0/mode = disabled`) und gibt
  Schreibrechte auf den Lüfter; beim Beenden oder Absturz schaltet systemd sie wieder ein, dann regelt der
  Kernel normal. `main.py` von Hand gestartet: nur Anzeige, der Kernel regelt.
  Webseite (Roboter-Ansicht): CPU-Temperatur oben, Abschnitt „System“ mit Drehzahl, Stufe, wer regelt.
- **Aufzeichnung jedes Laufs (entschieden 2026-11-03):** damit ein Lauf ohne Verbindung danach 1:1
  nachvollzogen werden kann.
  - Automatisch von K1 bis Ende/K2 nach `logs/<Datum>_<Uhrzeit>.jsonl.gz` (JSON-Zeilen, gzip). Wird laufend
    auf die Karte geschrieben, bei Stromausfall ist alles bis kurz davor lesbar. **Nichts wird automatisch gelöscht.**
  - **Eingänge an der Hardware-Grenze:** jeder Aufruf von Uhr (`monotonic`/`sleep`), Encoder, Motorbefehle,
    Ultraschall (mit Echo-Pulsbreite), Farbsensor-Rohwerte, Taster (Unterbrechung, Antwort im Lack of Progress).
    Kopf: Datum, Git-Stand, alle `config`-Werte, Farbkalibrierung, gemessene Gangbreite, Gefahrenzonen-Modus.
    Aufrufe für die Webseite (Akku, Motorleistung, Anzeige im Stand) zählen nicht dazu.
  - **Zusätzlich alle Rechenwerte:** Lenkung (Messung, welche Seite zählt, B, Abweichung, Schräg, P/D, Korrektur),
    Haltepunkt, Strecke, Farbprüfung, Drehen, Schrägstand-Schätzung, Wände, Planung (Ziele, Aktionen,
    Zeitschätzung), der Zustand wie auf der Webseite (alle `ROBOT_PUBLISH_S`), Ereignisse, Fehler mit Traceback.
  - **Nachrechnen:** `python3 -m tools.replay_run [Datei]` lässt den echten Code mit den aufgezeichneten
    Eingängen nochmal laufen und vergleicht jeden Motorbefehl und Rechenwert (meldet die erste Abweichung).
    Dafür laufen alle Zeiten des Fahrcodes über die austauschbare Uhr (auch das Bodenmessen im Stand).
  - **Anschauen:** Webseite, dritter Reiter „Aufzeichnung“: Lauf auswählen, Zeitleiste (abspielen, spulen,
    Aktion vor/zurück), Karte, Werte und Rechnung zum Zeitpunkt, Diagramme, Rohdaten um den Zeitpunkt, Download.
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
- **Exit:** Der Heimweg plant 10 s Exit-Signal ein (Regel 5.6.12). Da der echte Roboter nicht blinkt,
  gibt es in der Fahrphysik **keinen Exit-Bonus** (der kachelweise Test-`SimRobot` zählt ihn noch).
- **Schiedsrichter in der Fahrphysik:** Kachel besucht, wenn die Robotermitte darauf ist. Schwarz befahren
  oder blaue Kachel vor 5 s verlassen = Lack of Progress (wird gezählt, der Lauf geht weiter).
- **Blau:** Bereits besuchte blaue Kacheln werden bei der Wegplanung **stark gemieden** (2026: −10 Punkte je Wiederholung).
- **Kalibrieren:** **40 s** der 8 Minuten gelten als verbraucht (Uhr startet bei 0:40).
- **Punkte nach Regelwerk 2026** werden live angezeigt.

### Regelwerk 2026: relevante Fakten

Quelle: [RCJ Rescue Maze Rules 2026](https://junior.robocup.org/wp-content/uploads/2026/02/RCJRescueMaze2026-final.pdf)
(Stand 2026-03-29), Wettkampffelder: [rescue.rcj.cloud 2026](https://rescue.rcj.cloud/events/2026/RoboCup2026/maze/practice)

- Kachel 30 × 30 cm, Wände ≥ 15 cm hoch, 30 cm lang, auf Kachelkanten; Durchgang 28 cm (3.1, 3.3)
  → in `config.py` Wanddicke 2 cm (`WALL_THICKNESS_CM`), Sollabstände Sensor–Wand rechnen damit.
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
- Ultraschall kalibrieren: `python3 -m tools.calibrate_ultrasonic` (nur einer: `... links`) → `data/ultrasonic.json`,
  oder einzeln über `python3 call.py --ultrasonicL` / `--ultrasonicR` / `--ultrasonicV` (links/rechts/vorne, auch mehrere).
  **Ohne sudo starten**, sonst gehören `data/` und die Datei root und spätere Läufe können nicht speichern.
- Farbsensor kalibrieren: `python3 -m tools.calibrate_color` (nur Live-Anzeige: `--test`) → `data/colors.json`,
  oder `python3 call.py --rgb` (alle fünf Farben), `python3 call.py --rgb silber rot` (nur diese neu, die anderen bleiben;
  geht nur, wenn die gespeicherte Kalibrierung mit den jetzigen `COLOR_*`-Einstellungen gemessen wurde).
  `call.py` bricht ab, solange `robocup-robot` läuft (benutzt dieselben Sensoren).
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
- Lauf nachrechnen: `python3 -m tools.replay_run` (neueste Aufzeichnung) oder `python3 -m tools.replay_run logs/<Datei>`,
  alles als Text: `--text`

## Offene Fragen an den Projektinhaber

- Drift beim Geradeausfahren: ca. 10° nach rechts auf 98,5 cm, obwohl beide Encoder gleich liefen. Mechanik prüfen
  (Kettenspannung, etwas schleift)? Wiederholbar? Ausgleich (z.B. Faktor pro Kette) oder Seitenwände/Gyro?
- Schrägstand-Schätzung aus den Seitenwänden unterschätzt in der Fahrphysik (ohne Rauschen) um ca. 40 %
  (aus 10° Startschräge: geschätzt −0,66°, wirklich −1,10°), der Fehler wächst mit der Lenkbewegung.
  Vermutete Ursache: links/rechts werden abwechselnd gemessen, die Abweichung nutzt eine ältere Seite, die
  herausgerechnete Encoder-Drehung passt dann zeitlich nicht. Untersuchen/beheben?
  (Bestätigt: ohne Lenkbewegung trifft die Schätzung genau, 2,98° statt 3,0°.)
- Auslaufen nach jeder Drehung (kein Langsamwerden, keine Bremse): in der Fahrphysik dreht er jedes Mal ca. 2° zu
  weit, unabhängig vom Winkel. Bei 90° fängt das Einmessen es auf, bei der kleinen Korrektur am Kachelende nicht
  (1° befohlen → ca. 3°). Am echten Roboter gemessen (2026-11-04): nur 2 Encodergrad (ca. 0,5°), die Simulation
  übertreibt also. Simulation anpassen? Ausgleich (Vorhalt) nötig?

Zum Gyro (vor dem Umsetzen klären):
- Simulation: Gyro in `simulation/sim_hardware.py` nachbilden (Rauschen, Nullpunktdrift, Skalierungsfehler)?
- Anzeige auf der Webseite (Richtung, Drehrate, Nullpunkt; Roboter-Ansicht und Aufzeichnung)?
- Woran gilt das Gyro als ausgefallen (keine Antwort, Pufferüberlauf, passt nicht zu den Encodern)?
- Regelwerte für Drehen und Richtung halten (Abbremsen vor dem Ziel, Toleranz, Verstärkung) in der Fahrphysik einstellen?

## Noch umzusetzen

- [ ] **Gyro MPU-6050** anschließen und umsetzen wie unter „Gyro MPU-6050 – NOCH UMZUSETZEN“ beschrieben:
  `hardware/gyro.py`, `tools/test_gyro.py` (Achse, Vorzeichen, Skalierung einmessen; auch über `call.py`), Nutzung im Fahrcode
  (Drehen, Blickrichtung, geradeaus ohne Wände), Nullpunkt nachführen, Rückfall ohne Gyro, Aufzeichnung/Nachrechnen,
  Tests. Vorher die offenen Fragen oben klären.


## Bekannte Stolpersteine

- **BrickPi3 verliert einzelne Schreibbefehle** (gemessen 2026-11-04, ohne zweites Programm): von zwei Nachrichten
  direkt hintereinander ging die zweite in ca. 5 % verloren; auch eine einzelne Nachricht direkt nach anderen
  manchmal (14 von 500). Lesebefehle prüft die Bibliothek (Antwort 0xA5), Schreibbefehle nicht. Drehzahlen beim
  Fahren schickt der Code alle 10 ms neu (harmlos), das Abschalten wird nachgeprüft. Der gemeldete Status kann
  bis ca. 50 ms hinterherhinken. Greifen zwei Programme gleichzeitig zu (Dienst + Testprogramm), gehen mehr
  verloren: vor Motortests immer `sudo systemctl stop robocup-robot`.

- I2C ist eingeschaltet (`dtparam=i2c_arm=on`), Bus 1 = GPIO 2 (SDA) / GPIO 3 (SCL).
- Gyro-Reihe an J2 verdreht gesteckt → SDA hängt auf LOW, der ganze I2C-Bus (auch Farbsensor) ist blockiert.
  Prüfen mit `pinctrl get 2,3`: im Ruhezustand müssen beide `hi` sein.

- `lgpio` (Backend von `gpiozero` auf dem Pi 5) legt Dateien im aktuellen
  Arbeitsverzeichnis an. Programme müssen aus einem beschreibbaren Ordner gestartet werden.
- RCWL-9610A nur an 3,3 V betreiben: an 5 V wäre auch das Echo-Signal 5 V → würde einen Spannungsteiler brauchen.
