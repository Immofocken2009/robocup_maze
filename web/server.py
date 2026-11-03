"""Webseite für die Simulation.

Starten (im Projektordner):  python3 -m web.server
Liefert die Seite aus und schickt jeden Schritt der Simulation live per WebSocket.
Alle verbundenen Browser sehen dieselbe Simulation.
"""
import argparse
import asyncio
import json
import logging
import socket
import threading
from pathlib import Path

from aiohttp import WSMsgType, web

import config
from navigation.explorer import Explorer
from navigation.maze_map import Dir
from simulation.sim_robot import SimController, SimRobot, SimStopped, SimWorld, list_mazes

WEB_DIR = Path(__file__).resolve().parent
log = logging.getLogger("web")


class SimSession:
    def __init__(self, loop: asyncio.AbstractEventLoop):
        self.loop = loop
        self.queues: set[asyncio.Queue] = set()   # eine Warteschlange pro Browser
        self.speed = 5.0
        self.thread: threading.Thread | None = None
        self.controller: SimController | None = None
        self.generation = 0      # verhindert, dass ein alter Sim-Thread noch Daten schickt
        self.last_state: dict | None = None
        self.finished = False
        self.load(list_mazes()[0])

    # --- Steuerung (läuft im Event-Loop) --------------------------------------

    def handle(self, msg: dict):
        cmd = msg.get("cmd")
        if cmd == "load" and msg.get("maze") in list_mazes():
            self.load(msg["maze"])
        elif cmd == "reset":
            self.load(self.name)
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

    def load(self, name: str):
        self._stop()
        self.generation += 1
        gen = self.generation
        self.name = name
        self.finished = False
        self.world = SimWorld.load(name)
        self.reachable = len(self.world.reachable_tiles())
        self.controller = SimController(self.speed)
        self.robot = SimRobot(self.world, self.controller, on_action=lambda: self._publish(gen))
        self.explorer = Explorer(self.robot, on_update=lambda: self._publish(gen))
        self.send_all(self.world_msg())
        self._publish(gen)
        self.send_all(self.control_msg())

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

    def world_msg(self) -> dict:
        w = self.world
        tiles = [{"x": x, "y": y, "floor": kind.value,
                  "walls": "".join(d.name for d in Dir if w.wall((x, y), d))}
                 for (x, y), kind in w.tiles.items()]
        return {"type": "world", "name": self.name, "width": w.width, "height": w.height,
                "start": list(w.start), "tiles": tiles}

    def control_msg(self) -> dict:
        c = self.controller
        return {"type": "control", "mazes": list_mazes(), "current": self.name,
                "running": c.running and not self.finished, "started": self.thread is not None,
                "finished": self.finished, "speed": self.speed}

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
            "time_limit": e.time_limit_s,
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
            "events": list(e.events)[-15:],
            "anim_ms": min(1500, r.last_duration / self.speed * 1000),
        }

    def send_all(self, msg: dict):
        data = json.dumps(msg)
        for q in self.queues:
            q.put_nowait(data)


# --- HTTP / WebSocket ---------------------------------------------------------

async def websocket(request: web.Request) -> web.WebSocketResponse:
    session: SimSession = request.app["session"]
    ws = web.WebSocketResponse(heartbeat=20)
    await ws.prepare(request)

    queue: asyncio.Queue = asyncio.Queue()
    for msg in (session.world_msg(), session.last_state, session.control_msg()):
        if msg:
            queue.put_nowait(json.dumps(msg))
    session.queues.add(queue)

    async def writer():
        while True:
            await ws.send_str(await queue.get())

    writer_task = asyncio.create_task(writer())
    try:
        async for msg in ws:
            if msg.type == WSMsgType.TEXT:
                try:
                    session.handle(json.loads(msg.data))
                except (ValueError, TypeError):
                    log.warning("Ungültige Nachricht: %s", msg.data)
    finally:
        session.queues.discard(queue)
        writer_task.cancel()
    return ws


def static(name: str):
    async def handler(_request):
        return web.FileResponse(WEB_DIR / name)
    return handler


async def on_startup(app: web.Application):
    app["session"] = SimSession(asyncio.get_running_loop())


async def on_shutdown(app: web.Application):
    app["session"]._stop()


def local_ip() -> str:
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(("10.255.255.255", 1))   # schickt nichts, ermittelt nur die eigene Adresse
            return s.getsockname()[0]
    except OSError:
        return "localhost"


def main():
    parser = argparse.ArgumentParser(description="Webseite für die Maze-Simulation")
    parser.add_argument("--host", default=config.WEB_HOST)
    parser.add_argument("--port", type=int, default=config.WEB_PORT)
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(name)s: %(message)s")

    app = web.Application()
    app.router.add_get("/", static("index.html"))
    app.router.add_get("/app.js", static("app.js"))
    app.router.add_get("/style.css", static("style.css"))
    app.router.add_get("/ws", websocket)
    app.on_startup.append(on_startup)
    app.on_shutdown.append(on_shutdown)

    print(f"Simulation läuft:  http://localhost:{args.port}   (im Netz: http://{local_ip()}:{args.port})")
    web.run_app(app, host=args.host, port=args.port, print=None)


if __name__ == "__main__":
    main()
