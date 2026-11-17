"""Aufzeichnungen (Ansicht „Aufzeichnung“): geladene Läufe aus logs/ aufbereiten, Pfade /api/runs."""
import asyncio
import bisect
import json
import math
import re
import threading
import time
from pathlib import Path

from aiohttp import web

from hardware.run_log import LOG_DIR, list_runs, read_lines
from web.api import _json

# --- Aufzeichnungen (Ansicht „Aufzeichnung“) ------------------------------------------

RUN_NAME = re.compile(r"^[\w.-]+\.jsonl\.gz$")
CHART_POINTS = 3000         # so viele Punkte je Kurve höchstens an den Browser
RAW_BEFORE, RAW_AFTER = 80, 25      # Rohdaten um den Zeitpunkt


def _io_key(rec: dict) -> str:
    if rec["src"] == "us.measure" and rec.get("a"):
        return f"us.measure:{rec['a'][0]}"
    return rec["src"]


def _thin(points: list, limit: int) -> list:
    """Kurve ausdünnen, ohne Spitzen zu verlieren: je Abschnitt kleinster und größter Wert (Lücken bleiben)."""
    if len(points) <= limit:
        return points
    size = math.ceil(len(points) / (limit / 2))
    result = []
    for i in range(0, len(points), size):
        chunk = points[i:i + size]
        values = [p for p in chunk if p[1] is not None]
        picks = {id(p): p for p in (min(values, key=lambda p: p[1]), max(values, key=lambda p: p[1]))} if values else {}
        gap = next((p for p in chunk if p[1] is None), None)
        if gap is not None:
            picks[id(gap)] = gap
        result.extend(sorted(picks.values(), key=lambda p: p[0]))
    return result


class Recording:
    """Eine geladene Aufzeichnung, aufbereitet für die Ansicht. Die Einträge bleiben als Textzeilen im
    Speicher (ein ganzer Lauf hat einige 100 000) und werden erst beim Abfragen eines Zeitpunkts gelesen."""

    def __init__(self, path: Path):
        self.path = path
        self.lines, readable = read_lines(path)
        self.times: list[float] = []
        self.head: dict = {}
        self.end: dict | None = None
        self.calc: dict[str, tuple[list, list]] = {}    # was -> (Zeiten, Zeilennummern)
        self.io: dict[str, tuple[list, list]] = {}      # Quelle -> (Zeiten, Zeilennummern)
        self.frames: list[tuple[float, int, int, int]] = []   # (t, Zeile, Zeile mit map, Zeile mit events)
        self.marks: list[dict] = []                     # Beginn jeder Aktion
        self.counts: dict[str, int] = {}
        series = {name: [] for name in ("vorne", "links", "rechts", "abweichung", "motor_links", "motor_rechts")}
        xs, ys = [0], [0]
        map_line = events_line = -1
        action = None
        for n, line in enumerate(self.lines):
            try:
                r = json.loads(line)
            except json.JSONDecodeError:
                del self.lines[n:]
                readable = False
                break
            k, t = r["k"], r.get("t", 0.0)
            self.times.append(t)
            self.counts[k] = self.counts.get(k, 0) + 1
            if k == "head":
                self.head = r
            elif k == "end":
                self.end = r
            elif k == "calc":
                times, rows = self.calc.setdefault(r["what"], ([], []))
                times.append(t)
                rows.append(n)
                if r["what"] == "seite":
                    series["abweichung"].append(
                        [t, r.get("abweichung") if r.get("modus") in ("both", "left", "right") else None])
            elif k == "io":
                times, rows = self.io.setdefault(_io_key(r), ([], []))
                times.append(t)
                rows.append(n)
                src = r["src"]
                if src == "us.measure":
                    name = {"front": "vorne", "left": "links", "right": "rechts"}.get(r["a"][0])
                    if name:
                        series[name].append([t, r.get("r")])
                elif src in ("motors.set_dps", "motors.stop", "motors.float"):
                    left, right = r["a"] if src == "motors.set_dps" else (0, 0) if src == "motors.stop" else (None, None)
                    series["motor_links"].append([t, left])
                    series["motor_rechts"].append([t, right])
            elif k == "state":
                msg = r["msg"]
                if "map" in msg:
                    map_line = n
                    for tile in msg["map"].get("tiles", []):
                        xs.append(tile["x"])
                        ys.append(tile["y"])
                    for x, y in msg["map"].get("frontier", []):
                        xs.append(x)
                        ys.append(y)
                if "events" in msg:
                    events_line = n
                pose = msg.get("pose") or {}
                xs.append(pose.get("x", 0))
                ys.append(pose.get("y", 0))
                self.frames.append((t, n, map_line, events_line))
                if msg.get("action") != action:
                    action = msg.get("action")
                    self.marks.append({"t": t, "action": action, "detail": msg.get("detail")})
        self.complete = readable and self.end is not None
        self.frame_t = [f[0] for f in self.frames]
        self.bounds = {"x0": min(xs), "y0": min(ys), "w": max(xs) - min(xs) + 1, "h": max(ys) - min(ys) + 1}
        self.series = {name: _thin(points, CHART_POINTS) for name, points in series.items()}

    def record(self, n: int) -> dict:
        return json.loads(self.lines[n])

    def summary(self) -> dict:
        head = {k: v for k, v in self.head.items() if k not in ("config", "color_calibration")}
        return {
            "name": self.path.name, "head": head, "config": self.head.get("config", {}),
            "calibration": self.head.get("color_calibration"), "end": self.end, "complete": self.complete,
            "duration": max(self.times) if self.times else 0.0, "entries": len(self.lines), "counts": self.counts,
            "bounds": self.bounds, "marks": self.marks, "series": self.series,
        }

    def frame_at(self, t: float) -> tuple[float | None, dict | None]:
        """Zustand der Webseite zum Zeitpunkt t (map und events aus dem letzten Zustand, der sie hatte)."""
        i = bisect.bisect_right(self.frame_t, t)
        if not i:
            return None, None
        frame_t, n, map_line, events_line = self.frames[i - 1]
        msg = self.record(n)["msg"]
        if "map" not in msg:
            msg["map"] = self.record(map_line)["msg"]["map"] if map_line >= 0 else \
                {"tiles": [], "walls": [], "frontier": [], "planned": [], "checkpoint": None}
        if "events" not in msg:
            msg["events"] = self.record(events_line)["msg"]["events"] if events_line >= 0 else []
        return frame_t, msg

    def at(self, t: float) -> dict:
        """Alles zum Zeitpunkt t: Zustand der Webseite, letzte Rechenschritte, letzte Hardware-Werte, Rohdaten."""
        def latest(index: dict) -> dict:
            result = {}
            for key, (times, rows) in index.items():
                i = bisect.bisect_right(times, t)
                if i:
                    result[key] = self.record(rows[i - 1])
            return result

        frame_t, frame = self.frame_at(t)
        i = bisect.bisect_right(self.times, t)
        start = max(1, i - RAW_BEFORE)      # ohne Kopf
        raw = []
        for n in range(start, min(len(self.lines), i + RAW_AFTER)):
            rec = self.record(n)
            if rec["k"] == "state":
                msg = rec["msg"]
                rec = {"k": "state", "t": rec["t"], "status": msg.get("status"), "action": msg.get("action"),
                       "detail": msg.get("detail"), "mode": msg.get("mode")}
            raw.append(rec)
        return {"t": t, "frame_t": frame_t, "frame": frame, "calc": latest(self.calc), "io": latest(self.io),
                "raw": raw, "cursor": i - 1 - start}     # cursor: letzter Rohdaten-Eintrag bis t


class Recordings:
    """Liste der Aufzeichnungen; die zuletzt geöffnete bleibt geladen (neu, wenn die Datei gewachsen ist)."""

    def __init__(self):
        self._lock = threading.Lock()
        self._cache: tuple[str, float, Recording] | None = None

    def list(self) -> list[dict]:
        now = time.time()
        result = []
        for path in list_runs():
            stat = path.stat()
            result.append({"name": path.name, "size": stat.st_size, "growing": now - stat.st_mtime < 5})
        return result

    def get(self, name: str) -> Recording | None:
        path = LOG_DIR / name
        if not RUN_NAME.match(name) or not path.is_file():
            return None
        mtime = path.stat().st_mtime
        with self._lock:
            if self._cache and self._cache[0] == name and self._cache[1] == mtime:
                return self._cache[2]
            recording = Recording(path)
            self._cache = (name, mtime, recording)
            return recording


async def runs_list(request: web.Request) -> web.Response:
    return _json({"runs": request.app["recordings"].list()})


async def _recording(request: web.Request) -> Recording:
    recording = await asyncio.to_thread(request.app["recordings"].get, request.match_info["name"])
    if recording is None or not recording.lines:
        raise web.HTTPNotFound(text="Aufzeichnung nicht gefunden")
    return recording


async def run_summary(request: web.Request) -> web.Response:
    return _json((await _recording(request)).summary())


async def run_at(request: web.Request) -> web.Response:
    try:
        t = float(request.query.get("t", "0"))
    except ValueError:
        raise web.HTTPBadRequest(text="t muss eine Zahl sein")
    return _json((await _recording(request)).at(t))


async def run_download(request: web.Request) -> web.FileResponse:
    name = request.match_info["name"]
    path = LOG_DIR / name
    if not RUN_NAME.match(name) or not path.is_file():
        raise web.HTTPNotFound(text="Aufzeichnung nicht gefunden")
    return web.FileResponse(path, headers={"Content-Disposition": f'attachment; filename="{name}"'})


def add_run_routes(app: web.Application):
    app.router.add_get("/api/runs", runs_list)
    app.router.add_get("/api/runs/{name}", run_summary)
    app.router.add_get("/api/runs/{name}/at", run_at)
    app.router.add_get("/api/runs/{name}/download", run_download)
