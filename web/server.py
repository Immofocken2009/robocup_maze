"""Webseite: Ansicht „Roboter“ (live vom echten Roboter) und „Simulation“.

Startet beim Booten (robocup-web.service), von Hand im Projektordner:  python3 -m web.server
Liefert die Seite aus und schickt jeden Schritt live per WebSocket (/ws) an die Browser.
Alle verbundenen Browser sehen dieselbe Simulation.
Das Roboterprogramm (main.py) verbindet sich über /robot (nur vom Pi selbst) und schickt seinen
Zustand, der unverändert an alle Browser weitergeht.
"""
import argparse
import asyncio
import json
import logging
import socket
import threading
from pathlib import Path

from aiohttp import WSCloseCode, WSMsgType, web

import config
from navigation.explorer import Explorer
from navigation.maze_map import Dir
from simulation.maze_generator import generate
from simulation.sim_robot import (MAZES_DIR, SimController, SimRobot, SimStopped, SimWorld,
                                  TimeUp, list_mazes)

WEB_DIR = Path(__file__).resolve().parent
log = logging.getLogger("web")


class SimSession:
    def __init__(self, loop: asyncio.AbstractEventLoop):
        self.loop = loop
        self.queues: set[asyncio.Queue] = set()   # eine Warteschlange pro Browser
        self.speed = 5.0
        self.red_mode = config.RED_MODE
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
        self.robot = SimRobot(self.world, self.controller, on_action=lambda: self._publish(gen))
        self.explorer = Explorer(self.robot, red_mode=self.red_mode,
                                 on_update=lambda: self._publish(gen))
        self._publish(gen)

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

    def _publish(self, gen: int):
        if gen != self.generation:
            return
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
                "finished": self.finished, "speed": self.speed, "red_mode": self.red_mode}

    def state_msg(self) -> dict:
        w, r, e = self.world, self.robot, self.explorer
        to_world = w.to_world
        tiles = [{"x": to_world(p)[0], "y": to_world(p)[1], "floor": kind.value,
                  "visited": p in e.maze.visited}
                 for p, kind in e.maze.tiles.items()]
        walls = [{"x": to_world(p)[0], "y": to_world(p)[1], "dir": w.to_world_dir(d).name,
                  "wall": present}
                 for p, d, present in e.maze.walls()]
        checkpoint = list(to_world(e.last_checkpoint)) if e.last_checkpoint else None
        return {
            "type": "state",
            "robot": {"x": r.pos[0], "y": r.pos[1], "dir": int(r.dir)},
            "mode": e.mode,
            "time": r.sim_time,
            "time_limit": r.time_limit_s,
            "known_tiles": tiles,
            "known_walls": walls,
            "frontier": [list(to_world(p)) for p in e.maze.frontier()],
            "planned": [list(to_world(p)) for p in e.planned],
            "checkpoint": checkpoint,
            "sensors": {**r.distances(), "walls": r.read_walls(),
                        "color": r.read_floor().value, "gyro": r.gyro_deg % 360},
            "last_action": r.last_action,
            "stats": {"visited": len(e.maze.visited), "reachable": self.reachable,
                      "actions": r.actions},
            "score": r.score(),
            "events": list(e.events)[-30:],
            "anim_ms": min(1500, r.last_duration / self.speed * 1000),
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


def static(name: str):
    async def handler(_request):
        return web.FileResponse(WEB_DIR / name)
    return handler


async def on_startup(app: web.Application):
    app["session"] = SimSession(asyncio.get_running_loop())
    app["robot"] = RobotRelay(app["session"])
    app["sockets"] = set()


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
    app.on_startup.append(on_startup)
    app.on_shutdown.append(on_shutdown)

    print(f"Webseite läuft:  http://localhost:{args.port}   (im Netz: http://{local_ip()}:{args.port})",
          flush=True)
    web.run_app(app, host=args.host, port=args.port, print=None, shutdown_timeout=2)


if __name__ == "__main__":
    main()
