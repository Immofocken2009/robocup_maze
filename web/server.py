"""Webseite: Ansicht „Roboter“ (live vom echten Roboter), „Simulation“, „Aufzeichnung“ und „Netzwerk“.

Startet beim Booten (robocup-web.service), von Hand im Projektordner:  python3 -m web.server
Liefert die Seite aus und schickt jeden Schritt live per WebSocket (/ws) an die Browser.
Alle verbundenen Browser sehen dieselbe Simulation. Sie fährt mit Fahrphysik und dem echten
Fahrcode (simulation/sim_hardware.py, hardware/robot.py).
Das Roboterprogramm (main.py) verbindet sich über /robot (nur vom Pi selbst) und schickt seinen
Zustand, der unverändert an alle Browser weitergeht.
„Aufzeichnung“ zeigt die aufgezeichneten Läufe aus logs/ (hardware/run_log.py) über /api/runs.
„Netzwerk“ verwaltet WLAN-Verbindungen über NetworkManager (nmcli) über /api/network.
Aufgeteilt in: web/api.py (JSON-Helfer), web/sim_session.py (Simulation, Roboter-Verbindung), web/recordings.py
(Aufzeichnungen), web/network.py (WLAN), web/settings.py (Einstellungen); hier Seite, WebSocket und Start.
"""
import argparse
import asyncio
import json
import logging
import socket
from pathlib import Path

from aiohttp import WSCloseCode, WSMsgType, web

import config
# bleiben über web.server erreichbar:
from web.network import Network  # noqa: F401
from web.network import NetworkError, NEEDED_PERMISSIONS, SETUP_COMMAND, add_network_routes  # noqa: F401
from web.network import parse_networks, split_terse, wifi_security, run_command  # noqa: F401
from web.api import _finite  # noqa: F401
from web.recordings import Recording, Recordings, add_run_routes  # noqa: F401
from web.settings import settings_save, settings_get, settings_state, add_settings_routes  # noqa: F401
from web.sim_session import SimSession, RobotRelay  # noqa: F401

WEB_DIR = Path(__file__).resolve().parent
log = logging.getLogger("web")
STATIC_FILES = ("index.html", "style.css", "common.js", "view_robot.js", "view_sim.js", "view_log.js",
                "view_net.js", "view_settings.js", "app.js")   # Reihenfolge der Skripte wie in index.html


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
    relay.sync_viewers()

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
        relay.sync_viewers()
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
    relay.sync_viewers()
    log.info("Roboterprogramm verbunden")
    try:
        async for msg in ws:
            if msg.type == WSMsgType.TEXT:
                relay.last = msg.data
                relay.session.send_text(msg.data)
            elif msg.type == WSMsgType.BINARY:      # Kamerabild: flüchtig, wird nicht als Zustand gemerkt
                relay.session.send_cam(msg.data.decode())
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
    for name in STATIC_FILES:
        app.router.add_get(f"/{name}", static(name))
    app.router.add_get("/ws", websocket)
    app.router.add_get("/robot", robot_link)
    add_run_routes(app)
    add_network_routes(app)
    add_settings_routes(app)
    app.on_startup.append(on_startup)
    app.on_shutdown.append(on_shutdown)

    print(f"Webseite läuft:  http://localhost:{args.port}   (im Netz: http://{local_ip()}:{args.port})",
          flush=True)
    web.run_app(app, host=args.host, port=args.port, print=None, shutdown_timeout=2)


if __name__ == "__main__":
    main()
