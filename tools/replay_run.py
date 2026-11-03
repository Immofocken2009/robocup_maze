"""Einen aufgezeichneten Lauf (logs/) mit dem echten Code nachrechnen.

  python3 -m tools.replay_run                  neueste Aufzeichnung nachrechnen
  python3 -m tools.replay_run logs/<Datei>     bestimmte Aufzeichnung
  python3 -m tools.replay_run --calc [Datei]   dabei jeden Rechenschritt ausgeben
  python3 -m tools.replay_run --text [Datei]   alles als lesbaren Text ausgeben (ohne Nachrechnen)

Fahrcode (hardware/robot.py) und Explorer laufen wie auf dem Roboter, bekommen aber statt der Hardware
die aufgezeichneten Werte (Uhr, Encoder, Ultraschall, Farbsensor, Taster). Jeder Hardware-Aufruf, jeder
Motorbefehl und jeder Rechenschritt wird mit der Aufzeichnung verglichen, die erste Abweichung gemeldet.
Gerechnet wird mit den config-Werten von damals. Abweichungen gibt es, wenn sich der Code seit der
Aufzeichnung geändert hat (Git-Stand steht im Kopf der Aufzeichnung).
"""
import argparse
import json
import subprocess
import sys
from pathlib import Path

import config
from hardware.color_sensor import ColorSensor
from hardware.robot import RealRobot
from hardware.run_log import jsonable, list_runs, read_run
from hardware.ultrasonic import SIDES, Ultrasonic
from navigation.explorer import Explorer


class Divergence(BaseException):
    """Code und Aufzeichnung laufen auseinander. BaseException, damit der Fehlerfang im Lauf
    (except Exception in main.competition) sie nicht schluckt."""

    def __init__(self, text: str, t: float | None, exhausted: bool = False):
        super().__init__(text)
        self.text = text
        self.t = t
        self.exhausted = exhausted      # die Aufzeichnung ist zu Ende, der Code will weiter


def _same(a, b) -> bool:
    return json.dumps(a, sort_keys=True) == json.dumps(b, sort_keys=True)


def _fmt(value) -> str:
    return json.dumps(value, ensure_ascii=False)


class Replay:
    """Liefert die aufgezeichneten Eingänge der Reihe nach und prüft, was der Code tut."""

    def __init__(self, records: list[dict], show_calc: bool = False):
        self.head = records[0]
        self.inputs = [r for r in records if r["k"] in ("io", "flow")]
        self.calcs = [r for r in records if r["k"] == "calc"]
        self.end = next((r for r in records if r["k"] == "end"), None)
        self.i = 0
        self.c = 0
        self.show_calc = show_calc

    def last_t(self) -> float | None:
        return self.inputs[self.i - 1]["t"] if self.i else None

    def take(self, src: str, args: tuple = ()):
        args = jsonable(args)
        if self.i >= len(self.inputs):
            raise Divergence(f"Die Aufzeichnung ist zu Ende, der Code ruft aber noch {src}{_fmt(args)} auf.",
                             self.last_t(), exhausted=True)
        rec = self.inputs[self.i]
        if rec["src"] != src or not _same(rec.get("a", []), args):
            raise Divergence(f"Aufgezeichnet: {rec['src']}{_fmt(rec.get('a', []))}\n"
                             f"Der Code ruft:  {src}{_fmt(args)}", rec["t"])
        self.i += 1
        return rec.get("r")

    def sparse(self, src: str):
        """Eingang, der nur aufgezeichnet wurde, wenn er nicht None war (Taster-Unterbrechung)."""
        if self.i < len(self.inputs) and self.inputs[self.i]["src"] == src:
            self.i += 1
            return self.inputs[self.i - 1]["r"]
        return None

    def calc(self, what: str, **values):
        got = jsonable(values)
        if self.c >= len(self.calcs):
            raise Divergence(f"Die Aufzeichnung hat keine Rechenschritte mehr, der Code rechnet noch '{what}'.",
                             self.last_t(), exhausted=True)
        rec = self.calcs[self.c]
        expected = {k: v for k, v in rec.items() if k not in ("k", "t", "what")}
        if rec["what"] != what or not _same(expected, got):
            lines = [f"Rechenschritt aufgezeichnet: '{rec['what']}', nachgerechnet: '{what}'"]
            for key in sorted(set(expected) | set(got)):
                if not _same(expected.get(key), got.get(key)):
                    lines.append(f"  {key}: damals {_fmt(expected.get(key))}, jetzt {_fmt(got.get(key))}")
            raise Divergence("\n".join(lines), rec["t"])
        if self.show_calc:
            print(f"{rec['t']:9.4f}  {what:12} " + "  ".join(f"{k}={_fmt(v)}" for k, v in got.items()))
        self.c += 1


class ReplayClock:
    def __init__(self, replay: Replay):
        self.replay = replay

    def monotonic(self) -> float:
        return self.replay.take("clock.monotonic")

    def sleep(self, seconds: float):
        self.replay.take("clock.sleep", (seconds,))


class ReplayMotors:
    def __init__(self, replay: Replay):
        self.replay = replay

    def positions(self) -> tuple[float, float]:
        return tuple(self.replay.take("motors.positions"))

    def set_dps(self, left: float, right: float):
        self.replay.take("motors.set_dps", (left, right))

    def stop(self):
        self.replay.take("motors.stop")

    def float(self):
        self.replay.take("motors.float")


class ReplayUltrasonic:
    median = Ultrasonic.median          # gleiche Auswertung wie beim echten Sensor
    read_all = Ultrasonic.read_all

    def __init__(self, replay: Replay):
        self.replay = replay
        self.last: dict[str, float | None] = {side: None for side in SIDES}

    def ready(self) -> bool:
        return self.replay.take("us.ready")

    def measure(self, side: str) -> float | None:
        cm = self.replay.take("us.measure", (side,))
        self.last[side] = cm
        return cm


class ReplayColor:
    read_color = ColorSensor.read_color     # gleiche Auswertung wie beim echten Sensor
    read_tile = ColorSensor.read_tile

    def __init__(self, replay: Replay):
        head = replay.head
        self.replay = replay
        self.calibration = head.get("color_calibration")
        self.integration_s = head["color_integration_s"]
        self.warning = head.get("color_warning")
        self.last_raw = None
        self.last_color = None

    def read_raw(self) -> tuple[int, int, int, int]:
        self.last_raw = tuple(self.replay.take("color.read_raw"))
        return self.last_raw


def apply_config(recorded: dict) -> list[str]:
    """config-Werte von damals einsetzen. Ergebnis: Namen der Werte, die sich seitdem geändert haben."""
    changed = []
    for name, value in recorded.items():
        if not hasattr(config, name):
            continue
        current = getattr(config, name)
        if _same(jsonable(current), value):
            continue
        changed.append(f"{name}: damals {_fmt(value)}, jetzt {_fmt(jsonable(current))}")
        setattr(config, name, _like(current, value))
    return changed


def _like(current, value):
    """Aufgezeichneten Wert in den Typ des aktuellen bringen (Tupel, Pfade)."""
    if isinstance(current, Path):
        return Path(value)
    if isinstance(current, tuple) and isinstance(value, list):
        ref = current[0] if current else None
        return tuple(_like(ref, v) for v in value)
    if isinstance(current, list) and isinstance(value, list):
        ref = current[0] if current else None
        return [_like(ref, v) for v in value]
    if isinstance(current, dict) and isinstance(value, dict):
        return {k: _like(current.get(k), v) for k, v in value.items()}
    return value


def replay_run(records: list[dict], show_calc: bool = False) -> tuple[bool, str]:
    """Lauf nachrechnen. Ergebnis: (identisch, Bericht)."""
    import main     # erst hier: main zieht aiohttp und gpiozero nach

    replay = Replay(records, show_calc)
    head = replay.head
    robot = RealRobot(ReplayMotors(replay), ReplayUltrasonic(replay), ReplayColor(replay),
                      clock=ReplayClock(replay))
    robot.corridor_cm = head["corridor_cm"]
    robot._read_interrupt = lambda: replay.sparse("robot._read_interrupt")
    robot.trace = replay.calc
    explorer = Explorer(robot, red_mode=head["red_mode"], trace=replay.calc)
    counts = lambda: f"{replay.i} von {len(replay.inputs)} Hardware-Aufrufen, {replay.c} von {len(replay.calcs)} Rechenschritten"
    try:
        stopped = main.competition(robot, explorer, lambda: replay.take("lop_wait"))
    except Divergence as d:
        if replay.end is None and d.exhausted:
            return True, (f"Die Aufzeichnung bricht bei t = {d.t:.2f} s ab (Strom weg oder Programm beendet).\n"
                          f"Bis dahin identisch nachgerechnet: {counts()}.")
        where = f"bei t = {d.t:.4f} s" if d.t is not None else "am Anfang"
        action = _action_at(records, d.t)
        return False, (f"ABWEICHUNG {where}" + (f" (Aktion: {action})" if action else "") + f"\n{d.text}\n"
                       f"Bis dahin identisch: {counts()}.\n" + _context(replay))
    problems = []
    if replay.i < len(replay.inputs):
        problems.append(f"Der Code ist fertig, die Aufzeichnung hat aber noch {len(replay.inputs) - replay.i} "
                        f"Hardware-Aufrufe (nächster: {replay.inputs[replay.i]['src']} bei t = "
                        f"{replay.inputs[replay.i]['t']:.4f} s).")
    if replay.c < len(replay.calcs):
        problems.append(f"Die Aufzeichnung hat noch {len(replay.calcs) - replay.c} Rechenschritte mehr.")
    end = replay.end
    if end and (end.get("mode") != explorer.mode or end.get("stopped") != stopped):
        problems.append(f"Ergebnis damals: {end.get('mode')} (K2: {end.get('stopped')}), "
                        f"jetzt: {explorer.mode} (K2: {stopped}).")
    result = (f"Ergebnis: {explorer.mode}, {len(explorer.maze.visited)} Kacheln besucht"
              + (", mit K2 abgebrochen" if stopped else ""))
    if problems:
        return False, "ABWEICHUNG\n" + "\n".join(problems) + f"\n{counts()}\n{result}"
    if end is None:
        return True, f"Identisch nachgerechnet ({counts()}), aber die Aufzeichnung hat kein Ende.\n{result}"
    return True, f"Identisch nachgerechnet: {counts()}.\n{result}"


def _action_at(records: list[dict], t: float | None) -> str | None:
    action = None
    for r in records:
        if t is not None and r.get("t", 0.0) > t:
            break
        if r["k"] == "state":
            action = f"{r['msg'].get('action')}, {r['msg'].get('detail')}"
    return action


def _context(replay: Replay) -> str:
    lines = ["Die letzten Hardware-Aufrufe davor:"]
    for rec in replay.inputs[max(0, replay.i - 8):replay.i]:
        lines.append(f"  {rec['t']:9.4f}  {rec['src']}{_fmt(rec.get('a', []))} -> {_fmt(rec.get('r'))}")
    return "\n".join(lines)


def print_text(records: list[dict]):
    """Alle Einträge als lesbaren Text, eine Zeile pro Eintrag."""
    for r in records:
        k, t = r["k"], r.get("t", 0.0)
        if k == "head":
            print(f"Aufzeichnung {r.get('file')}  vom {r.get('date')}  Git {r.get('git')}"
                  + (f" (geändert: {', '.join(r['git_changed'])})" if r.get("git_changed") else ""))
            for key, value in r.items():
                if key not in ("k", "config"):
                    print(f"  {key}: {_fmt(value)}")
            for key, value in r.get("config", {}).items():
                print(f"  config.{key} = {_fmt(value)}")
        elif k == "io":
            extra = {key: v for key, v in r.items() if key not in ("k", "t", "src", "a", "r")}
            args = ", ".join(_fmt(a) for a in r.get("a", []))
            print(f"{t:9.4f}  io    {r['src']}({args}) -> {_fmt(r.get('r'))}"
                  + "".join(f"  {key}={_fmt(v)}" for key, v in extra.items()))
        elif k == "flow":
            print(f"{t:9.4f}  flow  {r['src']} -> {_fmt(r.get('r'))}")
        elif k == "calc":
            values = "  ".join(f"{key}={_fmt(v)}" for key, v in r.items() if key not in ("k", "t", "what"))
            print(f"{t:9.4f}  calc  {r['what']:12} {values}")
        elif k == "state":
            m = r["msg"]
            print(f"{t:9.4f}  state {m.get('status')} | {m.get('action')} | {m.get('detail')} | "
                  f"Position {m.get('pose')} | {m.get('mode')}")
        elif k == "log":
            print(f"{t:9.4f}  log   {r.get('level')} {r.get('text')}")
        elif k == "end":
            print(f"{t:9.4f}  ende  " + "  ".join(f"{key}={_fmt(v)}" for key, v in r.items() if key not in ("k", "t")))


def main():
    parser = argparse.ArgumentParser(description="Aufgezeichneten Lauf nachrechnen (siehe Kopf der Datei).")
    parser.add_argument("file", nargs="?", help="Aufzeichnung (Standard: die neueste in logs/)")
    parser.add_argument("--text", action="store_true", help="alles als Text ausgeben, nicht nachrechnen")
    parser.add_argument("--calc", action="store_true", help="beim Nachrechnen jeden Rechenschritt ausgeben")
    args = parser.parse_args()

    if args.file:
        path = Path(args.file)
    else:
        runs = list_runs()
        if not runs:
            sys.exit("Keine Aufzeichnung in logs/ gefunden.")
        path = runs[0]
    records, complete = read_run(path)
    if not records or records[0].get("k") != "head":
        sys.exit(f"{path}: keine lesbare Aufzeichnung.")
    if args.text:
        print_text(records)
        return

    head = records[0]
    print(f"Aufzeichnung {path.name} vom {head.get('date')}, {len(records)} Einträge"
          + ("" if complete else ", unvollständig (mittendrin abgebrochen)"))
    try:
        now = subprocess.run(["git", "rev-parse", "--short", "HEAD"], cwd=config.PROJECT_DIR,
                             capture_output=True, text=True).stdout.strip()
    except OSError:
        now = None
    if head.get("git") and now and head["git"] != now:
        print(f"Code damals Git {head['git']}, jetzt {now}. Exakt der damalige Code: git checkout {head['git']}")
    if head.get("git_changed"):
        print(f"Damals nicht committete Dateien: {', '.join(head['git_changed'])}")
    changed = apply_config(head.get("config", {}))
    if changed:
        print("config seitdem geändert, nachgerechnet wird mit den Werten von damals:")
        for line in changed:
            print(f"  {line}")
    same, report = replay_run(records, show_calc=args.calc)
    print(report)
    sys.exit(0 if same else 1)


if __name__ == "__main__":
    main()
