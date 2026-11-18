"""Simulation und Roboter-Verbindung der Webseite: SimSession (Fahrphysik-Simulation, Zustand für alle Browser),
RobotRelay (Verbindung zum Roboterprogramm main.py)."""
import asyncio
import json
import logging
import threading
import time

from aiohttp import web

import config
from hardware.robot import TimeUp
from navigation.explorer import Explorer
from navigation.maze_map import Dir
from simulation.maze_generator import generate
from simulation.sim_hardware import SimDrivingRobot, SimHardware
from simulation.sim_robot import MAZES_DIR, SimController, SimStopped, SimWorld, list_mazes, score_2026

PUBLISH_S = 0.05    # Fahrphysik: höchstens so oft (Echtzeit) den Zustand an die Browser schicken

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

    def send_cam(self, data: str):
        """Kamerabild: nur an Browser, die nachkommen (sonst wächst die Warteschlange eines langsamen Browsers ohne Ende)."""
        for q in self.queues:
            if q.qsize() < 4:
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
