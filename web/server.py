"""Webseite: Ansicht „Roboter“ (live vom echten Roboter), „Simulation“ und „Aufzeichnung“.

Startet beim Booten (robocup-web.service), von Hand im Projektordner:  python3 -m web.server
Liefert die Seite aus und schickt jeden Schritt live per WebSocket (/ws) an die Browser.
Alle verbundenen Browser sehen dieselbe Simulation. Sie fährt mit Fahrphysik und dem echten
Fahrcode (simulation/sim_hardware.py, hardware/robot.py).
Das Roboterprogramm (main.py) verbindet sich über /robot (nur vom Pi selbst) und schickt seinen
Zustand, der unverändert an alle Browser weitergeht.
„Aufzeichnung“ zeigt die aufgezeichneten Läufe aus logs/ (hardware/run_log.py) über /api/runs.
"""
import argparse
import asyncio
import bisect
import json
import logging
import math
import re
import socket
import threading
import time
from pathlib import Path

from aiohttp import WSCloseCode, WSMsgType, web

import config
from hardware.robot import TimeUp
from hardware.run_log import LOG_DIR, list_runs, read_lines
from navigation.explorer import Explorer
from navigation.maze_map import Dir
from simulation.maze_generator import generate
from simulation.sim_hardware import SimDrivingRobot, SimHardware
from simulation.sim_robot import MAZES_DIR, SimController, SimStopped, SimWorld, list_mazes, score_2026

PUBLISH_S = 0.05    # Fahrphysik: höchstens so oft (Echtzeit) den Zustand an die Browser schicken

WEB_DIR = Path(__file__).resolve().parent
log = logging.getLogger("web")


class SimSession:
    def __init__(self, loop: asyncio.AbstractEventLoop):
        self.loop = loop
        self.queues: set[asyncio.Queue] = set()   # eine Warteschlange pro Browser
        self.speed = 2.0
        self.red_mode = config.RED_MODE
        self.noise = False          # Messrauschen am Ultraschall
        self.slip = False           # Kettenschlupf
        self._publish_t = 0.0
        self.thread: threading.Thread | None = None
        self.controller: SimController | None = None
        self.generation = 0      # verhindert, dass ein alter Sim-Thread noch Daten schickt
        self.last_state: dict | None = None
        self.notes: list[str] = []
        self.generate(dict(config.GENERATOR_DEFAULTS))

    # --- Steuerung (läuft im Event-Loop) --------------------------------------

    def handle(self, msg: dict):
        cmd = msg.get("cmd")
        if cmd == "load" and msg.get("maze") in list_mazes():
            name = msg["maze"]
            self.load(json.loads((MAZES_DIR / f"{name}.json").read_text()), name, saved=True)
        elif cmd == "generate":
            self.generate(msg)
        elif cmd == "save":
            self.save()
        elif cmd == "reset":
            self.load(self.data, self.name, self.saved)
        elif cmd == "red_mode" and msg.get("value") in config.RED_MODES and self.thread is None:
            self.red_mode = msg["value"]
            self._new_run()
        elif cmd == "start" and not self.finished:
            self._ensure_thread()
            self.controller.start()
        elif cmd == "pause":
            self.controller.pause()
        elif cmd == "step" and not self.finished:
            self._ensure_thread()
            self.controller.step()
        elif cmd == "speed":
            self.speed = min(100.0, max(0.1, float(msg.get("value", 1))))
            self.controller.set_speed(self.speed)
        elif cmd in ("noise", "slip"):
            setattr(self, cmd, bool(msg.get("value")))
            self.hardware.set_noise(self.noise)
            self.hardware.set_slip(self.slip)
        self.send_all(self.control_msg())

    def generate(self, params: dict):
        def number(key: str, low: int, high: int) -> int:
            value = int(params.get(key, config.GENERATOR_DEFAULTS.get(key, low)))
            return min(high, max(low, value))

        size = config.GENERATOR_MAX_SIZE
        seed = params.get("seed")
        data, warnings = generate(
            number("width", 2, size), number("height", 2, size),
            black=number("black", 0, 20), blue=number("blue", 0, 20),
            silver=number("silver", 0, 20), zones=number("zones", 0, 3),
            seed=int(seed) if seed not in (None, "") else None)
        name = f"zufall_{data['width']}x{data['height']}_{data['seed']}"
        self.load(data, name, saved=False, notes=warnings)

    def save(self):
        path = MAZES_DIR / f"{self.name}.json"
        path.write_text(json.dumps(self.data, indent=1, ensure_ascii=False) + "\n")
        self.saved = True
        self.notes = [f"Gespeichert als simulation/mazes/{path.name}"]
        self.send_all(self.world_msg())

    def load(self, data: dict, name: str, saved: bool, notes: list[str] | None = None):
        self._stop()
        self.data = data
        self.name = name
        self.saved = saved
        self.notes = notes or []
        self.world = SimWorld.from_dict(data, name)
        self.reachable = len(self.world.reachable_tiles())
        self.floating = len(self.world.reachable_tiles() - self.world.linear_tiles())
        self.send_all(self.world_msg())
        self._new_run()

    def _new_run(self):
        """Neuer Lauf auf dem aktuellen Labyrinth (Roboter am Start, Karte leer)."""
        self._stop()
        self.generation += 1
        gen = self.generation
        self.finished = False
        self.controller = SimController(self.speed)
        self.hardware = SimHardware(self.world, self.controller, noise=self.noise, slip=self.slip,
                                    on_tick=lambda: self._publish(gen, force=False))
        self.robot = SimDrivingRobot(self.hardware, self.controller,
                                     on_change=lambda: self._publish(gen, force=False))
        self.explorer = Explorer(self.robot, red_mode=self.red_mode,
                                 on_update=lambda: self._publish(gen))
        self.hardware.body.on_event = self._referee_event
        self.robot.start_clock()
        self._publish(gen)

    def _referee_event(self, text: str):
        t = int(self.robot.time())
        self.explorer.events.append(f"{t // 60}:{t % 60:02d}  Schiedsrichter: {text}")

    def _ensure_thread(self):
        if self.thread is None:
            self.thread = threading.Thread(target=self._run, args=(self.generation,), daemon=True)
            self.thread.start()

    def _stop(self):
        if self.controller:
            self.controller.stop()
        if self.thread:
            self.thread.join(timeout=2)
        self.thread = None

    # --- Simulation (läuft im Sim-Thread) -------------------------------------

    def _run(self, gen: int):
        try:
            self.explorer.run()
        except SimStopped:
            return
        except TimeUp:
            self.explorer.mode = "Zeit abgelaufen"
            self.explorer.events.append("8:00  Zeit abgelaufen, Spiel beendet")
        except Exception:
            log.exception("Fehler in der Simulation")
            self.explorer.events.append("Fehler in der Simulation, siehe Server-Ausgabe")
        if gen == self.generation:
            self.finished = True
            self._publish(gen)
            self.loop.call_soon_threadsafe(lambda: self.send_all(self.control_msg()))

    def _publish(self, gen: int, force: bool = True):
        if gen != self.generation:
            return
        now = time.monotonic()
        if not force and now - self._publish_t < PUBLISH_S:
            return
        self._publish_t = now
        state = self.state_msg()
        self.loop.call_soon_threadsafe(self._store_and_send, state)

    def _store_and_send(self, state: dict):
        self.last_state = state
        self.send_all(state)

    # --- Nachrichten ----------------------------------------------------------

    def settings_msg(self) -> dict:
        return {"type": "settings", "field_sizes": config.FIELD_SIZES_2026,
                "defaults": config.GENERATOR_DEFAULTS, "max_size": config.GENERATOR_MAX_SIZE,
                "red_modes": config.RED_MODES}

    def world_msg(self) -> dict:
        w = self.world
        tiles = [{"x": x, "y": y, "floor": kind.value,
                  "walls": "".join(d.name for d in Dir if w.wall((x, y), d))}
                 for (x, y), kind in w.tiles.items()]
        return {"type": "world", "name": self.name, "saved": self.saved,
                "seed": self.data.get("seed"), "params": self.data.get("params"),
                "width": w.width, "height": w.height, "start": list(w.start),
                "tiles": tiles, "reachable": self.reachable, "floating": self.floating,
                "notes": self.notes}

    def control_msg(self) -> dict:
        c = self.controller
        return {"type": "control", "mazes": list_mazes(), "current": self.name,
                "running": c.running and not self.finished, "started": self.thread is not None,
                "finished": self.finished, "speed": self.speed, "red_mode": self.red_mode,
                "noise": self.noise, "slip": self.slip}

    def state_msg(self) -> dict:
        w, r, e, hw = self.world, self.robot, self.explorer, self.hardware
        to_world = w.to_world
        tiles = [{"x": to_world(p)[0], "y": to_world(p)[1], "floor": kind.value,
                  "visited": p in e.maze.visited}
                 for p, kind in e.maze.tiles.items()]
        walls = [{"x": to_world(p)[0], "y": to_world(p)[1], "dir": w.to_world_dir(d).name,
                  "wall": present}
                 for p, d, present in e.maze.walls()]
        checkpoint = list(to_world(e.last_checkpoint)) if e.last_checkpoint else None
        body = hw.body
        return {
            "type": "state",
            "robot": hw.pose(),
            "robot_size": {"len": config.ROBOT_LENGTH_CM / config.TILE_SIZE_CM,
                           "wid": config.ROBOT_WIDTH_CM / config.TILE_SIZE_CM},
            "beams": hw.beams(),
            "mode": e.mode,
            "action": r.action,
            "detail": r.detail,
            "time": r.time(),
            "time_limit": config.TIME_LIMIT_S,
            "known_tiles": tiles,
            "known_walls": walls,
            "frontier": [list(to_world(p)) for p in e.maze.frontier()],
            "planned": [list(to_world(p)) for p in e.planned],
            "checkpoint": checkpoint,
            "sensors": {**hw.ultrasonic.last, "color": hw.color.read_tile().value,
                        "encoders": [round(v) for v in body.encoder],
                        "heading_error": round(((body.heading - 90 * int(w.to_world_dir(e.heading))) + 180) % 360 - 180, 1)},
            "wall_threshold": config.WALL_THRESHOLD_CM,
            "stats": {"visited": len(e.maze.visited), "reachable": self.reachable, "actions": r.actions},
            "score": score_2026(w.tiles, body.visits, body.lop, exit_done=False),
            "events": list(e.events)[-30:],
            "anim_ms": 60,
        }

    def send_all(self, msg: dict):
        self.send_text(json.dumps(msg))

    def send_text(self, data: str):
        for q in self.queues:
            q.put_nowait(data)


class RobotRelay:
    """Verbindung zum Roboterprogramm: Zustand an alle Browser, Befehle zum Roboter."""

    def __init__(self, session: SimSession):
        self.session = session
        self.ws: web.WebSocketResponse | None = None
        self.last: str | None = None    # letzter Zustand des Roboters (für neue Browser)

    def link_msg(self) -> dict:
        return {"type": "robot_link", "connected": self.ws is not None}

    def handle(self, msg: dict):
        """Befehl aus dem Browser an den Roboter weitergeben."""
        if msg.get("cmd") == "robot_red_mode" and msg.get("value") in config.RED_MODES and self.ws is not None:
            asyncio.ensure_future(self.ws.send_str(json.dumps({"cmd": "red_mode", "value": msg["value"]})))


# --- HTTP / WebSocket ---------------------------------------------------------

async def websocket(request: web.Request) -> web.WebSocketResponse:
    session: SimSession = request.app["session"]
    relay: RobotRelay = request.app["robot"]
    ws = web.WebSocketResponse(heartbeat=20)
    await ws.prepare(request)
    request.app["sockets"].add(ws)

    queue: asyncio.Queue = asyncio.Queue()
    for msg in (session.settings_msg(), session.world_msg(), session.last_state,
                session.control_msg(), relay.link_msg()):
        if msg:
            queue.put_nowait(json.dumps(msg))
    if relay.last:
        queue.put_nowait(relay.last)
    session.queues.add(queue)

    async def writer():
        while True:
            await ws.send_str(await queue.get())

    writer_task = asyncio.create_task(writer())
    try:
        async for msg in ws:
            if msg.type == WSMsgType.TEXT:
                try:
                    data = json.loads(msg.data)
                    if str(data.get("cmd", "")).startswith("robot_"):
                        relay.handle(data)
                    else:
                        session.handle(data)
                except (ValueError, TypeError, AttributeError):
                    log.warning("Ungültige Nachricht: %s", msg.data)
    finally:
        session.queues.discard(queue)
        request.app["sockets"].discard(ws)
        writer_task.cancel()
    return ws


async def robot_link(request: web.Request) -> web.WebSocketResponse:
    """Hier verbindet sich main.py (nur vom Pi selbst erlaubt)."""
    if request.remote not in ("127.0.0.1", "::1"):
        raise web.HTTPForbidden(text="Nur für das Roboterprogramm auf dem Pi")
    relay: RobotRelay = request.app["robot"]
    ws = web.WebSocketResponse(heartbeat=20)
    await ws.prepare(request)
    request.app["sockets"].add(ws)
    if relay.ws is not None:
        await relay.ws.close()
    relay.ws = ws
    relay.session.send_all(relay.link_msg())
    log.info("Roboterprogramm verbunden")
    try:
        async for msg in ws:
            if msg.type == WSMsgType.TEXT:
                relay.last = msg.data
                relay.session.send_text(msg.data)
    finally:
        request.app["sockets"].discard(ws)
        if relay.ws is ws:
            relay.ws = None
            relay.session.send_all(relay.link_msg())
            log.info("Roboterprogramm getrennt")
    return ws


# --- Aufzeichnungen (Ansicht „Aufzeichnung“) ------------------------------------------

RUN_NAME = re.compile(r"^[\w.-]+\.jsonl\.gz$")
CHART_POINTS = 3000         # so viele Punkte je Kurve höchstens an den Browser
RAW_BEFORE, RAW_AFTER = 80, 25      # Rohdaten um den Zeitpunkt


def _finite(value):
    """NaN/unendlich gibt es in JSON nicht: durch null ersetzen."""
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        return {k: _finite(v) for k, v in value.items()}
    if isinstance(value, list):
        return [_finite(v) for v in value]
    return value


def _json(data) -> web.Response:
    return web.Response(text=json.dumps(_finite(data), ensure_ascii=False), content_type="application/json")


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


def static(name: str):
    async def handler(_request):
        return web.FileResponse(WEB_DIR / name)
    return handler


async def on_startup(app: web.Application):
    app["session"] = SimSession(asyncio.get_running_loop())
    app["robot"] = RobotRelay(app["session"])
    app["sockets"] = set()
    app["recordings"] = Recordings()


async def on_shutdown(app: web.Application):
    app["session"]._stop()
    for ws in list(app["sockets"]):     # offene Verbindungen schließen, sonst wartet aiohttp
        await ws.close(code=WSCloseCode.GOING_AWAY)


def local_ip() -> str:
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(("10.255.255.255", 1))   # schickt nichts, ermittelt nur die eigene Adresse
            return s.getsockname()[0]
    except OSError:
        return "localhost"


def main():
    parser = argparse.ArgumentParser(description="Webseite: echter Roboter und Simulation")
    parser.add_argument("--host", default=config.WEB_HOST)
    parser.add_argument("--port", type=int, default=config.WEB_PORT)
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(name)s: %(message)s")
    logging.getLogger("aiohttp.access").setLevel(logging.WARNING)

    app = web.Application()
    app.router.add_get("/", static("index.html"))
    app.router.add_get("/app.js", static("app.js"))
    app.router.add_get("/style.css", static("style.css"))
    app.router.add_get("/ws", websocket)
    app.router.add_get("/robot", robot_link)
    app.router.add_get("/api/runs", runs_list)
    app.router.add_get("/api/runs/{name}", run_summary)
    app.router.add_get("/api/runs/{name}/at", run_at)
    app.router.add_get("/api/runs/{name}/download", run_download)
    app.on_startup.append(on_startup)
    app.on_shutdown.append(on_shutdown)

    print(f"Webseite läuft:  http://localhost:{args.port}   (im Netz: http://{local_ip()}:{args.port})",
          flush=True)
    web.run_app(app, host=args.host, port=args.port, print=None, shutdown_timeout=2)


if __name__ == "__main__":
    main()
