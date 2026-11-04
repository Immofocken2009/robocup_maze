"""Roboterprogramm für den Wettkampf.

Startet beim Booten (robocup-robot.service), von Hand im Projektordner:  python3 main.py

K1 = Start, K2 = Stopp (zurück in den Zustand wie nach dem Booten),
K3 = Lack of Progress (Roboter umsetzen, nochmal K3 = weiter).
Schickt seinen Zustand live an die Webseite (config.ROBOT_LINK_URL). Läuft die Webseite nicht,
fährt der Roboter trotzdem und verbindet sich später neu.
Jeder Lauf wird nach logs/ aufgezeichnet (hardware/run_log.py), nachrechnen mit tools/replay_run.py.
"""
import asyncio
import json
import logging
import queue
import signal
import threading
import time

import aiohttp

import config
from hardware.buttons import Buttons
from hardware.color_sensor import ColorSensor
from hardware.fan import Fan
from hardware.motors import Motors
from hardware.robot import LackOfProgress, RealRobot, RunStopped, TimeUp
from hardware.run_log import Clock, Recorder
from hardware.ultrasonic import Ultrasonic
from navigation.explorer import Explorer

log = logging.getLogger("robot")

STATUS_TEXT = {
    "start": "Wartet auf K1",
    "running": "Läuft",
    "lop": "Lack of Progress: umsetzen, dann K3",
    "finished": "Fertig: K1 startet neu, K2 setzt zurück",
    "error": "Hardware antwortet nicht",
}


def competition(robot: RealRobot, explorer: Explorer, wait_lop, on_status=lambda status: None) -> bool:
    """Ein Wettkampflauf bis zum Ende. True = mit K2 abgebrochen.
    wait_lop(): nach Lack of Progress auf 'lop' (K3, weiter) oder 'stop' (K2) warten.
    Wird auch zum Nachrechnen einer Aufzeichnung benutzt (tools/replay_run.py)."""
    robot.reset()
    robot.start_clock()
    on_status("running")
    resume = stopped = False
    while True:
        try:
            if resume:
                explorer.resume()
            else:
                explorer.run()
            break
        except LackOfProgress:
            robot.motors.float()
            explorer.mode = "Lack of Progress"
            robot.set_action("wartet auf K3", "Roboter auf den letzten Checkpoint setzen")
            on_status("lop")
            if wait_lop() == "stop":
                stopped = True
                break
            robot.clear_interrupt()
            explorer.lack_of_progress()
            on_status("running")
            resume = True
        except RunStopped:
            stopped = True
            break
        except TimeUp:
            explorer.mode = "Zeit abgelaufen"
            break
        except Exception:
            log.exception("Fehler im Lauf")
            explorer.mode = "Fehler, siehe Aufzeichnung oder journalctl -u robocup-robot"
            break
    robot.motors.float()
    robot.clear_interrupt()
    return stopped


class RobotProgram:
    def __init__(self, loop: asyncio.AbstractEventLoop):
        self.loop = loop
        self.presses: queue.Queue[str] = queue.Queue()   # gedrückte Taster ('start', 'stop', 'lop')
        self.status = "start"
        self.error: str | None = None
        self.red_mode = config.RED_MODE
        self.robot: RealRobot | None = None
        self.explorer: Explorer | None = None
        self.buttons: Buttons | None = None
        self._parts: list = []
        self.battery: float | None = None
        self._battery_t = 0.0
        self._publish_t = 0.0
        self.latest: str | None = None              # letzter Zustand (für neue Verbindungen)
        self.ws: aiohttp.ClientWebSocketResponse | None = None
        self.closing = False
        self.fan = Fan()
        self.rec = Recorder()       # zeichnet jeden Lauf auf (logs/)

    # --- Taster (laufen im gpiozero-Thread) -------------------------------------

    def on_button(self, function: str):
        if self.robot:
            if function == "stop":
                self.robot.interrupt("stop")
            elif function == "lop" and self.status == "running":
                self.robot.interrupt("lop")
        self.presses.put(function)

    def wait_for(self, *functions: str) -> str:
        """Auf einen der Taster warten, solange die Sensoren für die Anzeige lesen."""
        while not self.presses.empty():         # alte Tastendrücke verwerfen
            self.presses.get_nowait()
        while True:
            try:
                function = self.presses.get(timeout=config.IDLE_SENSOR_S)
                if function in functions:
                    return function
            except queue.Empty:
                pass
            if not self.closing:
                with self.rec.paused():         # nur Anzeige, gehört nicht zum Lauf
                    self.robot.poll_sensors()
                self.publish()

    # --- Ablauf (Arbeits-Thread) ----------------------------------------------------

    def worker(self):
        while not self._init_hardware():
            time.sleep(config.HARDWARE_RETRY_S)
        new_run = False
        while True:
            if not new_run:
                self.status = "start"
                self.explorer = None
                self.robot.reset()
                self.robot.set_action("wartet auf K1")
                self.publish(force=True)
                self.wait_for("start")
            if self._run():        # K2: zurück in den Zustand wie nach dem Booten
                new_run = False
                continue
            self.robot.set_action("steht")
            self.publish(force=True)
            new_run = self.wait_for("start", "stop") == "start"

    def _init_hardware(self) -> bool:
        self.status = "error"
        try:
            for part in (Motors, Ultrasonic, ColorSensor):
                self._parts.append(part())
            motors, us, color = self._parts
            self.robot = RealRobot(motors, us, color, on_change=self.publish, clock=Clock())
            self.rec.attach(self.robot)
            self.buttons = Buttons(self.on_button)
        except Exception as exc:
            self.error = f"{type(exc).__name__}: {exc}"
            log.error("Hardware: %s", self.error)
            self.robot = None
            self._close_parts()
            self.publish(force=True)
            return False
        self.error = None
        log.info("Hardware bereit")
        return True

    def _run(self) -> bool:
        """Ein Wettkampflauf mit Aufzeichnung. True = mit K2 abgebrochen."""
        robot = self.robot
        self.explorer = Explorer(robot, red_mode=self.red_mode, on_update=self._explorer_update,
                                 trace=self.rec.calc)
        self.rec.start_run(robot, self.red_mode)
        stopped = False
        try:
            stopped = competition(robot, self.explorer, self._wait_lop, self._set_status)
        finally:
            self.status = "finished"
            self.rec.stop(mode=self.explorer.mode, stopped=stopped, visited=len(self.explorer.maze.visited))
        return stopped

    def _wait_lop(self) -> str:
        return self.rec.flow("lop_wait", self.wait_for("lop", "stop"))

    def _set_status(self, status: str):
        self.status = status
        self.publish(force=True)

    def _explorer_update(self):
        # Der Explorer meldet sich nur zwischen zwei Aktionen: Bewegungsanzeige ist übernommen.
        self.robot.motion = None
        self.publish(force=True)

    def fan_loop(self):
        """Eigener Thread: Lüfter regeln, unabhängig davon, was der Roboter gerade tut."""
        while not self.closing:
            try:
                self.fan.update()
            except Exception:
                log.exception("Lüfter")
            time.sleep(config.FAN_INTERVAL_S)

    def shutdown(self):
        self.closing = True
        if self.robot:
            self.robot.interrupt("stop")
            self.rec.closed.wait(1.0)       # laufende Aufzeichnung sauber abschließen
            time.sleep(0.1)
        self._close_parts()
        if self.buttons:
            self.buttons.close()

    def _close_parts(self):
        for part in reversed(self._parts):
            try:
                part.close()
            except Exception:
                log.exception("Beim Schließen von %s", type(part).__name__)
        self._parts = []

    # --- Zustand an die Webseite ---------------------------------------------------

    def publish(self, force: bool = False):
        """Zustand an die Webseite schicken (gedrosselt). Läuft im Arbeits-Thread."""
        now = time.monotonic()
        if not force and now - self._publish_t < config.ROBOT_PUBLISH_S:
            return
        self._publish_t = now
        with self.rec.paused():             # Akku, Motorleistung, Uhr für die Anzeige: nicht Teil des Laufs
            msg = self.state_msg()
        self.rec.state(msg)
        data = json.dumps(msg)
        self.loop.call_soon_threadsafe(self._send, data)

    def state_msg(self) -> dict:
        r, e = self.robot, self.explorer
        msg = {
            "type": "robot",
            "status": self.status,
            "status_text": STATUS_TEXT[self.status],
            "error": self.error,
            "red_mode": self.red_mode,
            "time": r.time() if r and r.started_at is not None else None,
            "time_limit": config.TIME_LIMIT_S,
            "wall_threshold": config.WALL_THRESHOLD_CM,
            "warnings": [],
            "pose": {"x": 0, "y": 0, "dir": 0},
            "map": {"tiles": [], "walls": [], "frontier": [], "planned": [], "checkpoint": None},
            "events": [],
            "mode": "bereit",
        }
        fan = self.fan
        msg["system"] = {"cpu_temp": fan.temp, "fan_rpm": fan.rpm, "fan_level": fan.level,
                         "fan_levels": len(config.FAN_STEPS),
                         "fan_percent": round(fan.pwm / 2.55) if fan.pwm is not None else None,
                         "fan_control": "Python" if fan.python_controls else "Kernel",
                         "cpu_hot": fan.temp is not None and fan.temp >= config.CPU_HOT_C}
        if fan.warning:
            msg["warnings"].append(fan.warning)
        if msg["system"]["cpu_hot"]:
            msg["warnings"].append(f"CPU heiß ({fan.temp:.0f} °C)")
        if r:
            if time.monotonic() - self._battery_t > config.BATTERY_READ_S:
                self._battery_t = time.monotonic()
                self.battery = r.motors.battery()
            msg.update(action=r.action, detail=r.detail, motion=r.motion,
                       detected={"walls": r.walls, "floor": r.floor.value if r.floor else None},
                       sensors={**r.sensors(), "battery": round(self.battery, 2),
                                "motor_power": list(r.motors.power())},
                       battery_low=self.battery < config.BATTERY_LOW_V)
            if r.color.warning:
                msg["warnings"].append(r.color.warning)
            if self.battery < config.BATTERY_LOW_V:
                msg["warnings"].append(f"Akku schwach ({self.battery:.1f} V)")
        if e:
            maze = e.maze
            msg.update(
                mode=e.mode,
                pose={"x": e.pos[0], "y": e.pos[1], "dir": int(e.heading)},
                map={
                    "tiles": [{"x": p[0], "y": p[1], "floor": kind.value, "visited": p in maze.visited}
                              for p, kind in maze.tiles.items()],
                    "walls": [{"x": p[0], "y": p[1], "dir": d.name, "wall": present}
                              for p, d, present in maze.walls()],
                    "frontier": [list(p) for p in maze.frontier()],
                    "planned": [list(p) for p in e.planned],
                    "checkpoint": list(e.last_checkpoint) if e.last_checkpoint else None,
                },
                events=list(e.events)[-30:],
                stats={"visited": len(maze.visited), "frontier": len(maze.frontier())},
            )
        return msg

    def _send(self, data: str):
        """Im Event-Loop: an die Webseite schicken, falls verbunden."""
        self.latest = data
        if self.ws is not None and not self.ws.closed:
            asyncio.ensure_future(self.ws.send_str(data))

    async def link(self):
        """Verbindung zur Webseite halten und Befehle von dort annehmen."""
        async with aiohttp.ClientSession() as session:
            while True:
                try:
                    async with session.ws_connect(config.ROBOT_LINK_URL, heartbeat=10) as ws:
                        self.ws = ws
                        log.info("Mit der Webseite verbunden")
                        if self.latest:
                            await ws.send_str(self.latest)
                        async for msg in ws:
                            if msg.type == aiohttp.WSMsgType.TEXT:
                                self.handle(json.loads(msg.data))
                except (aiohttp.ClientError, OSError):
                    pass
                self.ws = None
                await asyncio.sleep(config.ROBOT_LINK_RETRY_S)

    def handle(self, msg: dict):
        if (msg.get("cmd") == "red_mode" and msg.get("value") in config.RED_MODES
                and self.status in ("start", "finished")):
            self.red_mode = msg["value"]


def main():
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(name)s: %(message)s")
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    program = RobotProgram(loop)
    threading.Thread(target=program.worker, daemon=True).start()
    threading.Thread(target=program.fan_loop, daemon=True).start()
    link = loop.create_task(program.link())
    for sig in (signal.SIGTERM, signal.SIGINT):
        loop.add_signal_handler(sig, link.cancel)
    print(f"Roboterprogramm läuft. K1 = Start, K2 = Stopp, K3 = Lack of Progress. "
          f"Webseite: {config.ROBOT_LINK_URL}")
    try:
        loop.run_until_complete(link)
    except asyncio.CancelledError:
        pass
    finally:
        program.shutdown()
        log.info("Beendet, Motoren aus")


if __name__ == "__main__":
    main()
