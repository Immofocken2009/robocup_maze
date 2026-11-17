"""Tests der Aufbereitung für die Webseite (Ansicht „Aufzeichnung“, web/server.py)."""
import json
import tempfile
import unittest
from pathlib import Path

from hardware.run_log import Recorder, read_run
from tests.test_run_log import record


class ViewerTest(unittest.TestCase):
    """Aufbereitung für die Webseite (web/server.py, Ansicht „Aufzeichnung“)."""

    def setUp(self):
        self.dir = tempfile.TemporaryDirectory()
        self.path = Path(self.dir.name) / "lauf.jsonl.gz"

    def tearDown(self):
        self.dir.cleanup()

    def test_unchanged_map_is_stored_once(self):
        from web.server import Recording
        tiles = {"tiles": [{"x": 1, "y": 2, "floor": "normal", "visited": True}], "walls": [], "frontier": [],
                 "planned": [], "checkpoint": None}
        rec = Recorder()
        rec.start(self.path)
        rec.state({"action": "a", "pose": {"x": 0, "y": 0, "dir": 0}, "map": tiles, "events": ["e1"]})
        rec.state({"action": "b", "pose": {"x": 1, "y": 0, "dir": 0}, "map": tiles, "events": ["e1"]})
        rec.stop()
        states = [r["msg"] for r in read_run(self.path)[0] if r["k"] == "state"]
        self.assertIn("map", states[0])
        self.assertNotIn("map", states[1])
        view = Recording(self.path)
        _, frame = view.frame_at(view.frame_t[-1])
        self.assertEqual((frame["action"], frame["map"], frame["events"]), ("b", tiles, ["e1"]))

    def test_values_at_time(self):
        from web.server import Recording, _finite
        record(self.path)
        view = Recording(self.path)
        summary = view.summary()
        json.dumps(_finite(summary), allow_nan=False)
        t = summary["duration"] / 2
        at = view.at(t)
        json.dumps(_finite(at), allow_nan=False)
        self.assertTrue({"seite", "plan"} <= set(at["calc"]))
        self.assertTrue({"motors.positions", "us.measure:front", "clock.monotonic"} <= set(at["io"]))
        raw, i = at["raw"], at["cursor"]
        self.assertLessEqual(raw[i]["t"], t)
        self.assertGreater(raw[i + 1]["t"], t)


if __name__ == "__main__":
    unittest.main()

