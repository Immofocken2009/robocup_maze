"""Aufzeichnung eines Laufs, damit er danach 1:1 nachvollzogen werden kann.

Eine Datei pro Lauf: logs/<Datum>_<Uhrzeit>.jsonl.gz, eine JSON-Zeile pro Eintrag ("k" = Art):
  head   Kopf: Datum, Git-Stand, alle config-Werte, Farbkalibrierung, Startzustand des Roboters
  io     ein Aufruf an der Hardware-Grenze: src (z.B. "motors.positions"), a = Argumente, r = Ergebnis
         (Uhr, Encoder, Motorbefehle, Ultraschall, Farbsensor-Rohwerte, Taster-Unterbrechung)
  flow   Ablauf von außen: Antwort auf den Taster im Lack of Progress
  calc   ein Rechenschritt des Fahrcodes oder Explorers: what = was, dazu alle Werte
  state  Zustand wie auf der Webseite (msg); map und events fehlen, wenn sie sich nicht geändert haben
         (dann gilt der letzte Zustand, der sie enthält)
  log    Meldung aus logging (Warnungen, Fehler mit Traceback)
  end    Ende des Laufs
t = Sekunden seit Beginn der Aufzeichnung nach der Uhr des Roboters.

Aus den io- und flow-Einträgen rechnet tools/replay_run.py den Lauf mit dem echten Code nach.
Aufgezeichnet wird nur im Arbeits-Thread des Laufs und nicht innerhalb von paused() (Anzeige).
"""
import gzip
import json
import logging
import os
import queue
import subprocess
import threading
import time
import zlib
from contextlib import contextmanager
from datetime import datetime
from enum import Enum
from pathlib import Path

import config

LOG_DIR = config.PROJECT_DIR / "logs"
SUFFIX = ".jsonl.gz"
FLUSH_S = 1.0       # so oft wird auf die Karte geschrieben (Strom weg: bis dahin lesbar)


def jsonable(value):
    """In reine JSON-Werte umwandeln (Tupel -> Liste, Enum -> Wert, Schlüssel -> Text)."""
    if isinstance(value, Enum):
        return value.value
    if isinstance(value, dict):
        return {str(jsonable(k)): jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [jsonable(v) for v in value]
    if isinstance(value, (set, frozenset)):
        return sorted(jsonable(v) for v in value)
    if value is None or isinstance(value, (bool, int, float, str)):
        return value
    return str(value)


class RunLog:
    """Schreibt Einträge als JSON-Zeilen in eine gzip-Datei. Packen und Schreiben auf die Karte laufen in
    einem eigenen Thread, damit die Regelschleife des Fahrcodes nie auf die SD-Karte wartet."""

    def __init__(self, path: Path):
        path.parent.mkdir(exist_ok=True)
        self.path = path
        self._queue: queue.SimpleQueue[str | None] = queue.SimpleQueue()
        self._closed = False
        self._file = gzip.open(path, "wt", compresslevel=6, encoding="utf-8")
        self._writer = threading.Thread(target=self._write_loop, name="run_log", daemon=True)
        self._writer.start()

    def write(self, record: dict):
        if not self._closed:
            self._queue.put(json.dumps(record, ensure_ascii=False, separators=(",", ":"), default=str))

    def _write_loop(self):
        synced = time.monotonic()
        while True:
            try:
                line = self._queue.get(timeout=FLUSH_S)
            except queue.Empty:
                line = ""
            if line is None:
                break
            if line:
                self._file.write(line + "\n")
            if time.monotonic() - synced >= FLUSH_S:
                synced = time.monotonic()
                self._file.flush()      # gzip-Sync-Punkt: bis hierher auch ohne Dateiende lesbar
                os.fsync(self._file.buffer.fileobj.fileno())
        self._file.close()
        fd = os.open(self.path, os.O_RDONLY)
        try:
            os.fsync(fd)
        finally:
            os.close(fd)

    def close(self):
        """Alles Gepufferte schreiben und die Datei abschließen."""
        if not self._closed:
            self._closed = True
            self._queue.put(None)
            self._writer.join(timeout=10)


class Clock:
    """Uhr des echten Roboters. Der Recorder schreibt jeden Aufruf mit."""

    def monotonic(self) -> float:
        return time.monotonic()

    def sleep(self, seconds: float):
        time.sleep(seconds)


class _Handler(logging.Handler):
    def __init__(self, recorder: "Recorder"):
        super().__init__(logging.INFO)
        self.recorder = recorder
        self.setFormatter(logging.Formatter("%(name)s: %(message)s"))

    def emit(self, record: logging.LogRecord):
        self.recorder.write("log", level=record.levelname, text=self.format(record))


class Recorder:
    """Schreibt mit, was der Fahrcode mit der Hardware austauscht und was er rechnet."""

    def __init__(self):
        self.log: RunLog | None = None
        self._thread: int | None = None
        self._paused = 0
        self._depth = 0                 # > 0: gerade in einem aufgezeichneten Aufruf
        self._time = time.monotonic     # Roboter-Uhr ohne Aufzeichnung (für t)
        self._t0 = 0.0
        self._handler = _Handler(self)
        self._last_state: dict[str, str] = {}
        self.closed = threading.Event()
        self.closed.set()

    # --- Einhängen ---------------------------------------------------------------

    def instrument(self, obj, name: str, methods: list[str], sparse: tuple[str, ...] = (),
                   extra: dict | None = None):
        """Methoden eines Objekts mitschreiben. sparse: nur aufzeichnen, wenn das Ergebnis nicht None ist
        (z.B. Taster-Unterbrechung, sonst wäre es ein Eintrag pro Prüfung). extra: Methode -> Funktion,
        die zusätzliche Werte zum Eintrag liefert (z.B. Echo-Pulsbreite)."""
        extra = extra or {}
        for method in methods:
            setattr(obj, method, self._wrap(f"{name}.{method}", getattr(obj, method),
                                            method in sparse, extra.get(method)))

    def instrument_clock(self, clock, name: str = "clock"):
        self._time = clock.monotonic
        self.instrument(clock, name, ["monotonic", "sleep"])

    def attach(self, robot):
        """Alle Hardware-Grenzen eines RealRobot mitschreiben (dieselbe Liste für Roboter und Simulation):
        Uhr, Encoder, Motorbefehle, Ultraschall, Farbsensor-Rohwerte, Taster-Unterbrechung, Rechenschritte."""
        self.instrument_clock(robot.clock)
        self.instrument(robot.motors, "motors", ["positions", "set_dps", "stop", "float"])
        us = robot.us
        extra = {"measure": lambda: {"ns": us.last_echo_ns}} if hasattr(us, "last_echo_ns") else None
        self.instrument(us, "us", ["ready", "measure"], extra=extra)
        self.instrument(robot.color, "color", ["read_raw"])
        self.instrument(robot, "robot", ["_read_interrupt"], sparse=("_read_interrupt",))
        robot.trace = self.calc

    def _wrap(self, src: str, fn, sparse: bool, extra):
        def call(*args):
            # Nur der äußerste Aufruf zählt (z.B. nicht die Uhr, die ein Sensor intern benutzt)
            if self._depth or not self.recording():
                return fn(*args)
            self._depth += 1
            try:
                result = fn(*args)
            finally:
                self._depth -= 1
            if not (sparse and result is None):
                record = {"src": src, "a": jsonable(args), "r": jsonable(result)}
                if extra:
                    record.update(jsonable(extra()))
                self.write("io", **record)
            return result
        return call

    # --- Lauf --------------------------------------------------------------------

    def start(self, path: Path | None = None, **head):
        """Neue Aufzeichnung (im Arbeits-Thread aufrufen)."""
        self.stop()
        self.log = RunLog(path or LOG_DIR / f"{datetime.now():%Y-%m-%d_%H-%M-%S}{SUFFIX}")
        self.closed.clear()
        self._thread = threading.get_ident()
        self._t0 = self._time()
        self._last_state = {}
        self.log.write(jsonable({"k": "head", "version": 1, "file": self.log.path.name,
                                 "date": datetime.now().isoformat(timespec="seconds"),
                                 **_git(), "config": config_values(), **head}))
        logging.getLogger().addHandler(self._handler)

    def start_run(self, robot, red_mode: str, path: Path | None = None):
        """Aufzeichnung eines Laufs mit allem, was zum Nachrechnen über den Startzustand nötig ist."""
        color = robot.color
        self.start(path, red_mode=red_mode, corridor_cm=robot.corridor_cm, color_calibration=color.calibration,
                   color_integration_s=color.integration_s, color_warning=color.warning)

    def stop(self, **end):
        if self.log is None:
            return
        logging.getLogger().removeHandler(self._handler)
        self.write("end", **end)
        log, self.log = self.log, None
        log.close()
        self.closed.set()

    def recording(self) -> bool:
        return self.log is not None and not self._paused and threading.get_ident() == self._thread

    @contextmanager
    def paused(self):
        """Hardware-Aufrufe für die Anzeige: nicht mitschreiben (gehören nicht zum Lauf)."""
        self._paused += 1
        try:
            yield
        finally:
            self._paused -= 1

    # --- Einträge ----------------------------------------------------------------

    def write(self, kind: str, **values):
        log = self.log
        if log is not None:
            log.write({"k": kind, "t": round(self._time() - self._t0, 4), **values})

    def calc(self, what: str, **values):
        """Rechenschritt (vom Fahrcode und Explorer aufgerufen)."""
        if self.recording():
            self.write("calc", what=what, **jsonable(values))

    def flow(self, what: str, result):
        """Antwort von außen, die den Ablauf steuert; gibt sie unverändert zurück."""
        if self.log is not None:
            self.write("flow", src=what, r=jsonable(result))
        return result

    def state(self, msg: dict):
        if self.log is None:
            return
        msg = dict(msg)
        for key in ("map", "events"):           # groß und selten geändert: nur bei Änderung
            text = json.dumps(msg.get(key), sort_keys=True)
            if text == self._last_state.get(key):
                msg.pop(key, None)
            else:
                self._last_state[key] = text
        self.write("state", msg=msg)


def config_values() -> dict:
    return jsonable({name: getattr(config, name) for name in dir(config) if name.isupper()})


def _git() -> dict:
    try:
        run = lambda *args: subprocess.run(["git", *args], cwd=config.PROJECT_DIR, capture_output=True,
                                           text=True, timeout=2).stdout
        changed = [line[3:] for line in run("status", "--porcelain").splitlines() if line.strip()]
        return {"git": run("rev-parse", "--short", "HEAD").strip(), "git_changed": changed}
    except (OSError, subprocess.SubprocessError):
        return {"git": None, "git_changed": []}


# --- Lesen ---------------------------------------------------------------------------

def read_lines(path: Path) -> tuple[list[str], bool]:
    """Alle vollständigen Zeilen einer Aufzeichnung. Zweiter Wert: Datei bis zum Ende lesbar
    (False = mittendrin abgebrochen, z.B. Strom weg; dann ist alles bis zum letzten Schreiben da)."""
    lines: list[str] = []
    complete = True
    try:
        with gzip.open(path, "rt", encoding="utf-8") as f:
            for line in f:
                if not line.endswith("\n"):
                    complete = False        # letzte Zeile abgeschnitten
                    break
                lines.append(line)
    except (EOFError, OSError, zlib.error):
        complete = False
    return lines, complete


def read_run(path: Path) -> tuple[list[dict], bool]:
    """Alle Einträge einer Aufzeichnung. Zweiter Wert: vollständig (mit Ende-Eintrag)."""
    lines, complete = read_lines(path)
    records: list[dict] = []
    for line in lines:
        try:
            records.append(json.loads(line))
        except json.JSONDecodeError:
            complete = False
            break
    return records, complete and bool(records) and records[-1].get("k") == "end"


def list_runs() -> list[Path]:
    """Aufzeichnungen, neueste zuerst."""
    return sorted(LOG_DIR.glob(f"*{SUFFIX}"), reverse=True)
