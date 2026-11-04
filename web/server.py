"""Webseite: Ansicht „Roboter“ (live vom echten Roboter), „Simulation“, „Aufzeichnung“ und „Netzwerk“.

Startet beim Booten (robocup-web.service), von Hand im Projektordner:  python3 -m web.server
Liefert die Seite aus und schickt jeden Schritt live per WebSocket (/ws) an die Browser.
Alle verbundenen Browser sehen dieselbe Simulation. Sie fährt mit Fahrphysik und dem echten
Fahrcode (simulation/sim_hardware.py, hardware/robot.py).
Das Roboterprogramm (main.py) verbindet sich über /robot (nur vom Pi selbst) und schickt seinen
Zustand, der unverändert an alle Browser weitergeht.
„Aufzeichnung“ zeigt die aufgezeichneten Läufe aus logs/ (hardware/run_log.py) über /api/runs.
„Netzwerk“ verwaltet WLAN-Verbindungen über NetworkManager (nmcli) über /api/network.
"""
import argparse
import asyncio
import bisect
import json
import logging
import math
import os
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


def _json(data, status: int = 200) -> web.Response:
    return web.Response(text=json.dumps(_finite(data), ensure_ascii=False), content_type="application/json",
                        status=status)


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


# --- Netzwerk (Reiter „Netzwerk“) -----------------------------------------------------------
#
# WLAN über NetworkManager (nmcli, ohne Shell). Die Seite legt WLAN-Verbindungen an (Name robocup-<Netz>), startet
# und löscht sie. LAN behält Vorrang: Verbindungen der Seite bekommen Routenmetrik 600, LAN hat 100.
# Sie laufen ohne WLAN-Energiesparen (das kann die Live-Seite über WLAN verzögern).
# Ein Passwort steht nur während des Aufrufs in der Befehlszeile, nie in Meldungen oder im Log.
# Der Dienst läuft ohne angemeldete Sitzung: ohne robocup-network.rules (polkit) verweigert NetworkManager Änderungen.

NMCLI = "nmcli"
UUID = re.compile(r"^[0-9a-fA-F]{8}(-[0-9a-fA-F]{4}){3}-[0-9a-fA-F]{12}$")
SETUP_COMMAND = (f"sudo install -m 644 {config.PROJECT_DIR / 'robocup-network.rules'} "
                 "/etc/polkit-1/rules.d/50-robocup-network.rules")
NEEDED_PERMISSIONS = ("org.freedesktop.NetworkManager.settings.modify.system",
                      "org.freedesktop.NetworkManager.network-control",
                      "org.freedesktop.NetworkManager.wifi.scan")


class NetworkError(Exception):
    """Fehler mit Text für die Anzeige (enthält nie ein Passwort oder die Befehlszeile)."""

    def __init__(self, message: str, status: int = 400, setup: bool = False):
        super().__init__(message)
        self.status = status
        self.setup = setup


async def run_command(*command: str, timeout: float = config.NET_QUERY_S) -> tuple[int, str, str]:
    """Programm ohne Shell ausführen. Ergebnis: (Rückgabewert, Ausgabe, Fehlerausgabe); Meldungen englisch, UTF-8."""
    env = {k: v for k, v in os.environ.items() if k != "LANG" and not k.startswith("LC_")}
    env.update(LANG="C.UTF-8", LC_MESSAGES="C")
    try:
        proc = await asyncio.create_subprocess_exec(*command, stdin=asyncio.subprocess.DEVNULL,
                                                    stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.PIPE,
                                                    env=env)
    except FileNotFoundError:
        raise NetworkError(f"{command[0]} ist nicht installiert", 503) from None
    try:
        out, err = await asyncio.wait_for(proc.communicate(), timeout)
    except asyncio.TimeoutError:
        proc.kill()
        await proc.wait()
        raise NetworkError("Keine Antwort vom Netzwerk (Zeitüberschreitung)", 504) from None
    return proc.returncode, out.decode(errors="replace"), err.decode(errors="replace")


def split_terse(line: str) -> list[str]:
    """Eine Zeile aus „nmcli -t“ in Felder teilen. Doppelpunkt und Backslash im Text sind mit Backslash maskiert."""
    fields, current, i = [], [], 0
    while i < len(line):
        ch = line[i]
        if ch == "\\" and i + 1 < len(line):
            current.append(line[i + 1])
            i += 2
            continue
        if ch == ":":
            fields.append("".join(current))
            current = []
        else:
            current.append(ch)
        i += 1
    fields.append("".join(current))
    return fields


def fields_of(line: str, count: int) -> list[str]:
    return (split_terse(line) + [""] * count)[:count]


def explain(error: str, secret: str | None = None) -> NetworkError:
    """Fehlermeldung von nmcli in verständlichen Text übersetzen (das Passwort wird entfernt)."""
    text = " ".join(error.split())
    if secret:
        text = text.replace(secret, "***")
    low = text.lower()
    if "not authorized" in low or "insufficient privileges" in low or "authorization failed" in low:
        return NetworkError("Keine Berechtigung, das Netzwerk zu ändern. Einmal einrichten: " + SETUP_COMMAND,
                            403, setup=True)
    if "not running" in low or "could not create nmclient" in low:
        return NetworkError("NetworkManager läuft nicht", 503)
    if "secrets were required" in low or "no secrets" in low:
        return NetworkError("Passwort falsch oder fehlt")
    if "no network with ssid" in low:
        return NetworkError("Netz nicht gefunden (Name falsch oder zu weit weg)")
    if "timeout" in low or "timed out" in low:
        return NetworkError("Verbinden hat zu lange gedauert (Passwort falsch oder Netz zu weit weg)")
    if "no suitable device" in low or "device" in low and "not found" in low:
        return NetworkError("Kein passender WLAN-Adapter gefunden", 503)
    return NetworkError("Fehler von NetworkManager: " + (text[:200] or "unbekannt"))


def wifi_security(security: str) -> tuple[str, str]:
    """Spalte SECURITY aus der Suche: (Art, Anzeige). Art: open, psk (WPA/WPA2), sae (nur WPA3), enterprise, unsupported."""
    tokens = security.split()
    if not tokens or tokens == ["--"]:
        return "open", "offen"
    if "802.1X" in tokens:
        return "enterprise", security
    if "WEP" in tokens:
        return "unsupported", security
    if {"WPA1", "WPA2", "WPA"} & set(tokens):
        return "psk", security
    if "WPA3" in tokens:
        return "sae", security
    return "unsupported", security


def parse_networks(output: str, known: set[str]) -> list[dict]:
    """Ergebnis von „nmcli -t -f IN-USE,SSID,SIGNAL,SECURITY device wifi list“: jedes Netz einmal (das stärkste),
    ohne versteckte (leerer Name), stärkste zuerst."""
    best: dict[str, dict] = {}
    for line in output.splitlines():
        in_use, ssid, signal, security = fields_of(line, 4)
        if not ssid:
            continue
        kind, label = wifi_security(security)
        entry = {"ssid": ssid, "signal": int(signal) if signal.isdigit() else 0, "security": kind,
                 "label": label, "in_use": in_use.strip() == "*", "known": ssid in known}
        old = best.get(ssid)
        if old is not None:
            entry["in_use"] = entry["in_use"] or old["in_use"]
            if old["signal"] >= entry["signal"]:
                old["in_use"] = entry["in_use"]
                continue
        best[ssid] = entry
    return sorted(best.values(), key=lambda n: (-n["in_use"], -n["signal"], n["ssid"].lower()))


def check_ssid(ssid) -> str:
    if not isinstance(ssid, str) or not ssid or len(ssid.encode()) > 32 or any(ord(c) < 32 or ord(c) == 127 for c in ssid):
        raise NetworkError("Netzname: 1 bis 32 Zeichen, ohne Steuerzeichen")
    return ssid


def check_password(password, kind: str) -> str:
    if not isinstance(password, str) or not password:
        raise NetworkError("Passwort fehlt")
    if kind == "psk":
        if not (8 <= len(password) <= 63 and all(32 <= ord(c) < 127 for c in password)) \
                and not re.fullmatch(r"[0-9a-fA-F]{64}", password):
            raise NetworkError("Passwort: 8 bis 63 Zeichen (oder 64 Hex-Zeichen), nur Buchstaben, Zahlen und Zeichen ohne Umlaute")
    elif len(password) > 128 or any(ord(c) < 32 or ord(c) == 127 for c in password):
        raise NetworkError("Passwort: höchstens 128 Zeichen, ohne Steuerzeichen")
    return password


class Network:
    """WLAN-Verwaltung. run: Programm ausführen (in Tests ersetzt, sie greifen nie auf das echte Netzwerk zu)."""

    def __init__(self, run=run_command):
        self.run = run
        self.lock = asyncio.Lock()      # Änderungen nacheinander

    async def nmcli(self, *args: str, timeout: float = config.NET_QUERY_S, secret: str | None = None) -> str:
        code, out, err = await self.run(NMCLI, *args, timeout=timeout)
        if code != 0:
            raise explain(err or out, secret)
        return out

    # --- Lesen -------------------------------------------------------------------

    async def devices(self) -> dict[str, dict]:
        out = await self.nmcli("-t", "-f", "DEVICE,TYPE,STATE,CONNECTION", "device", "status")
        found: dict[str, dict] = {}
        for line in out.splitlines():
            name, kind, state, connection = fields_of(line, 4)
            if kind in ("ethernet", "wifi") and kind not in found:
                found[kind] = {"name": name, "state": state, "connected": state == "connected",
                               "connection": connection or None, "ip": None}
        return found

    async def addresses(self) -> tuple[dict[str, str], str | None]:
        """IP-Adressen je Schnittstelle und die Schnittstelle mit der besten Standardroute (nur zur Anzeige)."""
        ips: dict[str, str] = {}
        preferred = None
        try:
            code, out, _ = await self.run("ip", "-j", "-4", "addr", "show", timeout=config.NET_QUERY_S)
            if code == 0:
                for item in json.loads(out):
                    for info in item.get("addr_info", []):
                        if info.get("family") == "inet":
                            ips.setdefault(item["ifname"], info["local"])
            code, out, _ = await self.run("ip", "-j", "route", "show", "default", timeout=config.NET_QUERY_S)
            routes = json.loads(out) if code == 0 else []
            if routes:
                preferred = min(routes, key=lambda r: r.get("metric", 0))["dev"]
        except (NetworkError, ValueError, KeyError, TypeError):
            pass
        return ips, preferred

    async def ssid_of(self, uuid: str) -> str:
        try:
            return (await self.nmcli("--escape", "no", "-g", "802-11-wireless.ssid", "connection", "show",
                                     "uuid", uuid)).rstrip("\n")
        except NetworkError:
            return ""

    async def saved(self) -> list[dict]:
        """Gespeicherte WLAN-Verbindungen. managed: von dieser Seite angelegt (nur die lassen sich löschen);
        die anderen (netplan) baut das System beim Neustart aus seiner Konfiguration neu."""
        out = await self.nmcli("-t", "-f", "UUID,NAME,TYPE,AUTOCONNECT,DEVICE", "connection", "show")
        rows = [r for r in (fields_of(line, 5) for line in out.splitlines()) if r[2] == "802-11-wireless"]
        ssids = await asyncio.gather(*(self.ssid_of(r[0]) for r in rows))
        return [{"uuid": uuid, "name": name, "ssid": ssid or name, "autoconnect": auto == "yes",
                 "device": device or None, "active": bool(device),
                 "managed": name.startswith(config.NET_WIFI_PREFIX)}
                for (uuid, name, _, auto, device), ssid in zip(rows, ssids)]

    async def can_change(self) -> bool:
        try:
            out = await self.nmcli("-t", "-f", "PERMISSION,VALUE", "general", "permissions")
        except NetworkError:
            return False
        values = dict(fields_of(line, 2) for line in out.splitlines())
        return all(values.get(p) == "yes" for p in NEEDED_PERMISSIONS)

    async def status(self) -> dict:
        devices = await self.devices()
        saved = await self.saved()
        ips, preferred_dev = await self.addresses()
        for device in devices.values():
            device["ip"] = ips.get(device["name"])
        wifi = devices.get("wifi")
        ssid = next((s["ssid"] for s in saved if wifi and s["device"] == wifi["name"]), None)
        return {"ethernet": devices.get("ethernet"), "wifi": wifi, "ssid": ssid,
                "preferred": next((k for k, d in devices.items() if d["name"] == preferred_dev), None),
                "saved": saved, "can_change": await self.can_change(), "setup_command": SETUP_COMMAND}

    async def scan(self, rescan: bool) -> dict:
        """Netze in der Nähe. Ohne rescan die Liste der letzten Suche (geht ohne Berechtigung, ist sofort da)."""
        known = {s["ssid"] for s in await self.saved()}
        args = ["-t", "-f", "IN-USE,SSID,SIGNAL,SECURITY", "device", "wifi", "list", "--rescan"]
        warning = None
        try:
            out = await self.nmcli(*args, "yes" if rescan else "no", timeout=config.NET_SCAN_S)
        except NetworkError as e:
            if not rescan:
                raise
            warning = f"{e} (angezeigt wird die Liste der letzten Suche)"
            out = await self.nmcli(*args, "no")
        return {"networks": parse_networks(out, known), "rescanned": rescan and warning is None, "warning": warning}

    # --- Ändern ------------------------------------------------------------------

    async def add_profile(self, wifi: str, ssid: str, kind: str, password: str | None, hidden: bool) -> str:
        args = ["connection", "add", "type", "wifi", "ifname", wifi, "con-name", config.NET_WIFI_PREFIX + ssid,
                "ssid", ssid, "connection.autoconnect", "yes",
                "ipv4.route-metric", str(config.NET_WIFI_METRIC), "ipv6.route-metric", str(config.NET_WIFI_METRIC),
                "802-11-wireless.powersave", str(config.NET_WIFI_POWERSAVE)]
        if hidden:
            args += ["802-11-wireless.hidden", "yes"]
        if kind in ("psk", "sae"):
            args += ["wifi-sec.key-mgmt", "wpa-psk" if kind == "psk" else "sae", "wifi-sec.psk", password,
                     "wifi-sec.psk-flags", "0"]          # Passwort für alle Benutzer, damit es auch ohne Anmeldung verbindet
        out = await self.nmcli(*args, secret=password)
        match = re.search(r"\(([0-9a-fA-F-]{36})\)", out)
        if match:
            return match.group(1)
        found = [s["uuid"] for s in await self.saved() if s["managed"] and s["ssid"] == ssid]
        if not found:
            raise NetworkError("Verbindung angelegt, aber nicht gefunden")
        return found[-1]

    async def connect(self, ssid, password=None, hidden: bool = False, security: str | None = None) -> dict:
        """Mit einem Netz verbinden. Gibt es für den Namen schon eine Verbindung, wird sie gestartet (mit neuem
        Passwort, wenn die Seite sie angelegt hat). Sonst wird eine angelegt, bei Misserfolg wieder gelöscht."""
        check_ssid(ssid)
        if security in ("enterprise", "unsupported"):
            raise NetworkError("Dieses Netz (Unternehmens-WLAN oder WEP) wird nicht unterstützt")
        kind = security if security in ("open", "psk", "sae") else ("psk" if password else "open")
        async with self.lock:
            wifi = (await self.devices()).get("wifi")
            if wifi is None:
                raise NetworkError("Kein WLAN-Adapter gefunden", 503)
            existing = next((s for s in await self.saved() if s["ssid"] == ssid), None)
            created = existing is None
            if existing is None:
                if kind != "open":
                    check_password(password, kind)
                uuid = await self.add_profile(wifi["name"], ssid, kind, password, hidden)
            else:
                uuid = existing["uuid"]
                if password and existing["managed"]:
                    check_password(password, kind if kind != "open" else "psk")
                    await self.nmcli("connection", "modify", "uuid", uuid, "wifi-sec.psk", password,
                                     "wifi-sec.psk-flags", "0", secret=password)
            try:
                await self.nmcli("-w", str(config.NET_CONNECT_S), "connection", "up", "uuid", uuid, "ifname",
                                 wifi["name"], timeout=config.NET_CONNECT_S + 10, secret=password)
            except NetworkError:
                if created:         # falsches Passwort soll keine Verbindung zurücklassen, die ständig neu versucht
                    try:
                        await self.nmcli("connection", "delete", "uuid", uuid)
                    except NetworkError:
                        pass
                raise
        return await self.status()

    async def forget(self, uuid) -> dict:
        """Eine von der Seite angelegte, gerade nicht aktive WLAN-Verbindung löschen."""
        if not isinstance(uuid, str) or not UUID.match(uuid):
            raise NetworkError("Ungültige Verbindung")
        async with self.lock:
            entry = next((s for s in await self.saved() if s["uuid"] == uuid), None)
            if entry is None:
                raise NetworkError("Verbindung nicht gefunden", 404)
            if not entry["managed"]:
                raise NetworkError("Systemverbindung (netplan): lässt sich nur in der Systemkonfiguration ändern")
            if entry["active"]:
                raise NetworkError("Gerade verbunden: erst mit einem anderen Netz verbinden oder das Kabel einstecken")
            await self.nmcli("connection", "delete", "uuid", uuid)
        return await self.status()


async def _network(request: web.Request, call) -> web.Response:
    try:
        return _json(await call(request.app["network"]))
    except NetworkError as e:
        return _json({"error": str(e), "setup": e.setup}, e.status)


async def _body(request: web.Request) -> dict:
    try:
        data = await request.json()
    except ValueError:
        raise web.HTTPBadRequest(text="JSON erwartet") from None
    if not isinstance(data, dict):
        raise web.HTTPBadRequest(text="JSON-Objekt erwartet")
    return data


async def network_status(request: web.Request) -> web.Response:
    return await _network(request, lambda net: net.status())


async def network_scan(request: web.Request) -> web.Response:
    return await _network(request, lambda net: net.scan(request.query.get("rescan") == "1"))


async def network_connect(request: web.Request) -> web.Response:
    data = await _body(request)
    return await _network(request, lambda net: net.connect(
        data.get("ssid"), data.get("password") or None, data.get("hidden") is True, data.get("security")))


async def network_forget(request: web.Request) -> web.Response:
    data = await _body(request)
    return await _network(request, lambda net: net.forget(data.get("uuid")))


def add_network_routes(app: web.Application):
    app.router.add_get("/api/network", network_status)
    app.router.add_get("/api/network/scan", network_scan)
    app.router.add_post("/api/network/connect", network_connect)
    app.router.add_post("/api/network/forget", network_forget)


def static(name: str):
    async def handler(_request):
        return web.FileResponse(WEB_DIR / name)
    return handler


async def on_startup(app: web.Application):
    app["session"] = SimSession(asyncio.get_running_loop())
    app["robot"] = RobotRelay(app["session"])
    app["sockets"] = set()
    app["recordings"] = Recordings()
    app["network"] = Network()


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
    add_network_routes(app)
    app.on_startup.append(on_startup)
    app.on_shutdown.append(on_shutdown)

    print(f"Webseite läuft:  http://localhost:{args.port}   (im Netz: http://{local_ip()}:{args.port})",
          flush=True)
    web.run_app(app, host=args.host, port=args.port, print=None, shutdown_timeout=2)


if __name__ == "__main__":
    main()
