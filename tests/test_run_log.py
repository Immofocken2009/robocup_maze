"""Tests für die Aufzeichnung (hardware/run_log.py) und das Nachrechnen (tools/replay_run.py).

Aufgezeichnet wird ein Lauf in der Fahrphysik (echter Fahrcode, nachgebildete Hardware)."""
import gzip
import os
import random
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

import config
import main
from hardware.run_log import Recorder, read_run
from navigation.explorer import Explorer
from simulation.sim_hardware import SimDrivingRobot, SimHardware
from simulation.sim_robot import SimWorld
from tools.replay_run import replay_run


def record(path: Path, lop_at_s: float | None = None, seed: int = 1, settings_file: Path | None = None,
           calibrate: bool = True) -> Explorer:
    """Einen Lauf mit Rauschen und Schlupf aufzeichnen; lop_at_s: dann K3 drücken (Lack of Progress)."""
    world = SimWorld.load("sonderboeden")
    hw = SimHardware(world, noise=True, slip=True)
    hw.body.rng = random.Random(seed)
    hw.ultrasonic.rng = random.Random(seed + 1000)
    robot = SimDrivingRobot(hw)
    robot.settings_file = settings_file
    rec = Recorder()
    rec.attach(robot)
    if lop_at_s is not None:
        pressed = []

        def tick():
            if not pressed and hw.clock.t >= lop_at_s:
                pressed.append(True)
                robot.interrupt("lop")
        hw.clock.on_tick = tick
    explorer = Explorer(robot, red_mode="zuletzt", trace=rec.calc,
                        on_update=lambda: rec.state({"action": robot.action, "detail": robot.detail}))
    rec.start_run(robot, "zuletzt", path)
    calibrate = rec.flow("calibrate", calibrate)
    stopped = main.competition(robot, explorer, lambda: rec.flow("lop_wait", "lop"), calibrate=calibrate)
    rec.stop(mode=explorer.mode, stopped=stopped)
    return explorer


class RunLogTest(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.TemporaryDirectory()
        self.path = Path(self.dir.name) / "lauf.jsonl.gz"

    def tearDown(self):
        self.dir.cleanup()

    def test_contents(self):
        record(self.path)
        records, complete = read_run(self.path)
        self.assertTrue(complete)
        kinds = {r["k"] for r in records}
        self.assertTrue({"head", "io", "calc", "state", "end"} <= kinds)
        head = records[0]
        self.assertEqual(head["config"]["TILE_SIZE_CM"], config.TILE_SIZE_CM)
        sources = {r["src"] for r in records if r["k"] == "io"}
        self.assertTrue({"clock.monotonic", "clock.sleep", "motors.positions", "motors.set_dps",
                         "us.measure", "us.ready", "color.read_raw"} <= sources)
        calcs = {r["what"] for r in records if r["k"] == "calc"}
        self.assertTrue({"seite", "fahrt", "vorne", "drehen", "schräglage", "plan", "zeit", "wände"} <= calcs)
        self.assertIn("rückwärts", calcs)       # der Lauf enthält eine Rückwärtsfahrt (Aktion "B")

    def test_instrumented_call_passes_keyword_arguments(self):
        """Recorder._wrap reicht Schlüsselwortargumente durch und schreibt sie mit (Feld kw)."""
        from hardware.run_log import Recorder

        class Thing:
            def measure(self, side, n=1):
                return f"{side}:{n}"

        thing, rec = Thing(), Recorder()
        rec.start(self.path)
        try:
            rec.instrument(thing, "thing", ["measure"])
            self.assertEqual(thing.measure("front", n=3), "front:3")
            self.assertEqual(thing.measure("left"), "left:1")
        finally:
            rec.stop()
        ios = [r for r in read_run(self.path)[0] if r["k"] == "io"]
        self.assertEqual(ios[0], {**ios[0], "src": "thing.measure", "a": ["front"], "kw": {"n": 3}, "r": "front:3"})
        self.assertNotIn("kw", ios[1])

    def test_replay_is_identical(self):
        record(self.path)
        same, report = replay_run(read_run(self.path)[0])
        self.assertTrue(same, report)

    def test_replay_of_backward_drive_is_identical(self):
        """Eine Aufzeichnung mit Rückwärtsfahrten (Trace `rückwärts`, `fahrt` mit richtung) rechnet sich 1:1 nach."""
        record(self.path, seed=2)
        records = read_run(self.path)[0]
        backs = [r for r in records if r["k"] == "calc" and r.get("what") == "fahrt" and r.get("richtung") == -1]
        self.assertTrue(backs)
        same, report = replay_run(records)
        self.assertTrue(same, report)

    def test_replay_with_web_settings(self):
        """Einstellungen der Webseite werden mitgeschrieben und beim Nachrechnen wieder eingesetzt."""
        settings = Path(self.dir.name) / "settings.json"
        config.write_settings({"STEER_KP": 3.0, "STEER_MAX_DPS": 40.0}, settings)
        with mock.patch.object(config, "STEER_KP", config.STEER_KP), \
                mock.patch.object(config, "STEER_KD", config.STEER_KD), \
                mock.patch.object(config, "STEER_MAX_DPS", config.STEER_MAX_DPS), \
                mock.patch.object(config, "STEER_SLEW_DPS_PER_CM", config.STEER_SLEW_DPS_PER_CM):
            record(self.path, settings_file=settings)
            records = read_run(self.path)[0]
            self.assertTrue(any(r.get("src") == "robot.load_settings" for r in records))
            self.assertEqual(config.STEER_KP, 3.0)
            config.write_settings({}, settings)             # Datei danach geändert: Nachrechnen nimmt die Aufzeichnung
            same, report = replay_run(records)
            self.assertTrue(same, report)

    def test_replay_without_calibration_time(self):
        """Einmal K1 = ohne die 40 s Kalibrierzeit; die Entscheidung steht in der Aufzeichnung."""
        record(self.path, calibrate=False)
        records = read_run(self.path)[0]
        self.assertEqual(next(r["r"] for r in records if r.get("src") == "calibrate"), False)
        same, report = replay_run(records)
        self.assertTrue(same, report)

    def test_replay_of_old_recording_without_calibrate_entry(self):
        record(self.path)
        records = [r for r in read_run(self.path)[0] if r.get("src") != "calibrate"]
        same, report = replay_run(records)
        self.assertTrue(same, report)

    def test_replay_with_lack_of_progress(self):
        record(self.path, lop_at_s=90.0)
        records, _ = read_run(self.path)
        self.assertTrue(any(r["k"] == "io" and r["src"] == "robot._read_interrupt" for r in records))
        self.assertTrue(any(r["k"] == "flow" for r in records))
        same, report = replay_run(records)
        self.assertTrue(same, report)

    def test_replay_in_new_process(self):
        """Anderer Prozess mit anderem Hash-Seed (Mengen-Reihenfolge) rechnet trotzdem identisch."""
        record(self.path)
        env = dict(os.environ, PYTHONHASHSEED="12345")
        result = subprocess.run([sys.executable, "-m", "tools.replay_run", str(self.path)],
                                cwd=config.PROJECT_DIR, env=env, capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_tampered_value_is_found(self):
        record(self.path)
        records, _ = read_run(self.path)
        # eine Seitenmessung beim Fahren (geht direkt in die Lenkung, nicht in einen Median)
        sides = [i for i, r in enumerate(records) if r["k"] == "calc" and r["what"] == "seite"
                 and r["cm"] is not None and r["gefahren"] > 0]
        i = sides[len(sides) // 2]
        target = next(r for r in reversed(records[:i]) if r["k"] == "io" and r["src"] == "us.measure")
        self.assertEqual(target["r"], records[i]["cm"])
        target["r"] += 5.0
        same, report = replay_run(records)
        self.assertFalse(same)
        self.assertIn(f"t = {target['t']:.4f}", report)

    def test_cut_off_recording(self):
        """Strom weg mitten im Lauf: alles bis zum letzten Schreiben ist lesbar und rechnet nach."""
        record(self.path)
        lines = gzip.decompress(self.path.read_bytes()).decode().splitlines()
        cut = Path(self.dir.name) / "abgebrochen.jsonl.gz"
        data = gzip.compress("\n".join(lines[: len(lines) // 2]).encode() + b"\n")
        cut.write_bytes(data[: len(data) - 40])     # auch das Dateiende fehlt
        records, complete = read_run(cut)
        self.assertFalse(complete)
        self.assertGreater(len(records), len(lines) // 4)
        same, report = replay_run(records)
        self.assertTrue(same, report)
        self.assertIn("bricht", report)

