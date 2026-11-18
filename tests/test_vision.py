"""Tests der Kameraauswertung (vision/analysis.py) und der Bildschleife (vision/camera.py), ohne Kamera."""
import threading
import unittest

try:
    import numpy as np
except ImportError:             # ohne numpy (z.B. Rechner ohne die Kamerapakete) gibt es nichts zu testen
    np = None

import config

if np is not None:
    from vision.analysis import Analyzer
    from vision.camera import CameraFeed

W, H = 320, 240


def image(fn) -> "np.ndarray":
    """RGB-Bild aus einer Helligkeitsfunktion fn(y, x) -> 0..255 (Felder)."""
    y, x = np.mgrid[0:H, 0:W]
    g = fn(y, x).astype(np.uint8)
    return np.stack([g, g, g], axis=-1)


@unittest.skipIf(np is None, "numpy fehlt")
class AnalysisTest(unittest.TestCase):
    def test_dark_image_skips_edges(self):
        det = Analyzer().analyse(image(lambda y, x: np.full_like(x, 5)))
        self.assertTrue(det["dark"])
        self.assertEqual(det["vertical_edges"], [])
        self.assertEqual(det["lines"], [])

    def test_flat_image_has_no_edges(self):
        det = Analyzer().analyse(image(lambda y, x: np.full_like(x, 150)))
        self.assertFalse(det["dark"])
        self.assertEqual(det["vertical_edges"], [])
        self.assertEqual(det["lines"], [])

    def test_vertical_edge_found_with_bearing(self):
        det = Analyzer().analyse(image(lambda y, x: np.where(x < 240, 200, 90)))   # Kante rechts von der Mitte
        edges = det["vertical_edges"]
        self.assertEqual(len(edges), 1)
        self.assertAlmostEqual(edges[0]["x"], 240, delta=3)
        self.assertGreater(edges[0]["bearing"], 10)          # rechts der Fahrtrichtung
        self.assertEqual(det["lines"], [])                   # keine waagerechte Kante

    def test_horizontal_boundary_is_a_flat_line(self):
        det = Analyzer().analyse(image(lambda y, x: np.where(y < 150, 210, 90)))
        self.assertEqual(det["vertical_edges"], [])
        self.assertEqual(len(det["lines"]), 1)
        line = det["lines"][0]
        self.assertAlmostEqual(line["angle"], 0.0, delta=1.0)
        self.assertAlmostEqual(line["y0"], 150, delta=3)

    def test_slanted_boundary_angle(self):
        det = Analyzer().analyse(image(lambda y, x: np.where(y < 100 + 0.3 * x, 210, 90)))
        self.assertEqual(len(det["lines"]), 1)
        self.assertAlmostEqual(det["lines"][0]["angle"], 16.7, delta=2.0)     # atan(0.3)

    def test_motion_between_frames(self):
        analyzer = Analyzer()
        base = image(lambda y, x: (x * 3 + y) % 200 + 20)
        self.assertEqual(analyzer.analyse(base)["motion"], 0.0)
        same = analyzer.analyse(base)
        self.assertEqual(same["motion"], 0.0)
        self.assertFalse(same["moving"])
        moved = analyzer.analyse(np.roll(base, 12, axis=1))
        self.assertTrue(moved["moving"])

    def test_patch_color(self):
        rgb = np.zeros((H, W, 3), np.uint8)
        rgb[..., 0], rgb[..., 1], rgb[..., 2] = 200, 100, 50
        det = Analyzer().analyse(rgb)
        self.assertEqual(det["patch"]["rgb"], [200, 100, 50])
        self.assertTrue(0 <= det["patch"]["x0"] < det["patch"]["x1"] <= W)


class FakeSource:
    def __init__(self, frames):
        self.frames = list(frames)
        self.closed = False

    def capture(self, full=False):
        if not self.frames:
            raise RuntimeError("keine Bilder mehr")
        frame = self.frames.pop(0)
        return frame, (frame if full else None)

    def close(self):
        self.closed = True


@unittest.skipIf(np is None, "numpy fehlt")
class CameraFeedTest(unittest.TestCase):
    def run_feed(self, source_factory, wanted=lambda: True, wait=0.5):
        messages = []
        got = threading.Event()
        feed = CameraFeed(lambda m: (messages.append(m), got.set()), wanted=wanted, source_factory=source_factory)
        feed.start()
        got.wait(wait)
        feed.close()
        return feed, messages

    def test_sends_message_with_detections(self):
        try:
            import PIL  # noqa: F401
        except ImportError:
            self.skipTest("PIL fehlt")
        frame = image(lambda y, x: np.where(x < 240, 200, 90))
        feed, messages = self.run_feed(lambda: FakeSource([frame] * 50))
        self.assertTrue(messages)
        m = messages[0]
        self.assertEqual(m["type"], "cam")
        self.assertTrue(m["jpeg"])
        self.assertEqual(len(m["det"]["vertical_edges"]), 1)
        self.assertIsNone(feed.warning)

    def test_no_message_when_nobody_listens(self):
        frame = image(lambda y, x: np.full_like(x, 150))
        feed, messages = self.run_feed(lambda: FakeSource([frame] * 20), wanted=lambda: False, wait=0.3)
        self.assertEqual(messages, [])
        self.assertIsNotNone(feed.detections)       # ausgewertet wird trotzdem (für die Fahrsteuerung später)

    def test_missing_camera_gives_warning_not_crash(self):
        def broken():
            raise ImportError("No module named 'picamera2'")
        feed, messages = self.run_feed(broken, wait=0.3)
        self.assertEqual(messages, [])
        self.assertIn("Kamera nicht verfügbar", feed.warning)
        self.assertIn("picamera2", feed.warning)


class FakeRec:
    """Nachbau von hardware.run_log.Recorder für die Kamera-Aufzeichnung."""

    def __init__(self, path):
        from types import SimpleNamespace
        self.log = SimpleNamespace(path=path)
        self.records = []

    def write(self, kind, **values):
        self.records.append({"k": kind, **values})

    def now(self):
        import time
        return round(time.monotonic(), 4)


@unittest.skipIf(np is None, "numpy fehlt")
class RecordingTest(unittest.TestCase):
    def setUp(self):
        import tempfile
        from pathlib import Path
        self.dir = tempfile.TemporaryDirectory()
        self.addCleanup(self.dir.cleanup)
        self.path = Path(self.dir.name) / "2026-01-01_10-00-00.jsonl.gz"
        self.saved = (config.CAM_RECORD_MODE, config.CAM_DEBUG_FPS, config.CAM_REC_FPS)
        self.addCleanup(lambda: (setattr(config, "CAM_RECORD_MODE", self.saved[0]), setattr(config, "CAM_DEBUG_FPS", self.saved[1]),
                                 setattr(config, "CAM_REC_FPS", self.saved[2])))

    def record(self, mode, fps, frames=200, wait=0.6):
        config.CAM_RECORD_MODE, config.CAM_DEBUG_FPS, config.CAM_REC_FPS = mode, fps, fps
        frame = image(lambda y, x: np.where(x < 240, 200, 90))
        feed = CameraFeed(lambda m: None, wanted=lambda: False, source_factory=lambda: FakeSource([frame] * frames),
                          encode=lambda rgb, q: b"JPEG%d" % rgb.shape[1])     # kein echtes JPEG nötig: Bytes zählen
        rec = FakeRec(self.path)
        feed.start()
        feed.start_recording(rec)
        import time
        time.sleep(wait)
        info = feed.recording_info()
        feed.close()
        return feed, rec, info

    def test_cam_path(self):
        from vision.recording import cam_path
        self.assertEqual(cam_path(self.path).name, "2026-01-01_10-00-00.cam")

    def test_records_index_lines_and_file(self):
        feed, rec, info = self.record("normal", 20.0)
        self.assertTrue(info["on"])
        cams = [r for r in rec.records if r["k"] == "cam"]
        self.assertGreaterEqual(len(cams), 3)
        data = (self.path.parent / "2026-01-01_10-00-00.cam").read_bytes()
        offset = 0
        for r in cams:
            self.assertEqual(r["off"], offset)
            self.assertEqual(data[r["off"]:r["off"] + r["len"]], b"JPEG320")
            self.assertEqual((r["w"], r["h"]), (W, H))
            self.assertIn("vertical_edges", r["det"])
            offset += r["len"]
        self.assertEqual(len(data), offset)
        self.assertIsNone(feed.recorder)            # nach close() beendet

    def test_rate_is_limited(self):
        _, rec, _ = self.record("normal", 5.0, wait=1.1)
        cams = [r for r in rec.records if r["k"] == "cam"]
        self.assertTrue(3 <= len(cams) <= 8, len(cams))

    def test_mode_off_records_nothing(self):
        feed, rec, info = self.record("aus", 5.0, wait=0.3)
        self.assertFalse(info["on"])
        self.assertEqual(rec.records, [])
        self.assertFalse(list(self.path.parent.glob("*.cam")))

    def test_low_disk_space_disables_recording(self):
        import vision.camera as cam
        old = cam.free_mb
        cam.free_mb = lambda path: 10.0
        self.addCleanup(lambda: setattr(cam, "free_mb", old))
        feed, rec, info = self.record("normal", 5.0, wait=0.3)
        self.assertFalse(info["on"])
        self.assertIn("MB frei", feed.rec_warning)


@unittest.skipIf(np is None, "numpy fehlt")
class CamReviewTest(unittest.TestCase):
    """tools/cam_review.py auf einer kleinen künstlichen Aufzeichnung."""

    def setUp(self):
        import gzip
        import json
        import tempfile
        from pathlib import Path
        try:
            from PIL import Image
        except ImportError:
            self.skipTest("PIL fehlt")
        self.dir = tempfile.TemporaryDirectory()
        self.addCleanup(self.dir.cleanup)
        self.path = Path(self.dir.name) / "2026-01-01_10-00-00.jsonl.gz"
        det = {"w": W, "h": H, "brightness": 120.0, "dark": False, "motion": 1.0, "moving": False,
               "vertical_edges": [{"x": 200, "y0": 40, "y1": 200, "share": .5, "bearing": 8.0}],
               "lines": [{"x0": 20, "y0": 150, "x1": 300, "y1": 160, "n": 30, "angle": 2.0}], "patch": {}}
        records = [{"k": "head", "t": 0.0}]
        blob = b""
        import io
        for i in range(6):
            buf = io.BytesIO()
            Image.new("RGB", (640, 480), (40 * i, 100, 100)).save(buf, "JPEG")
            jpeg = buf.getvalue()
            records.append({"k": "cam", "t": i + .1, "ct": float(i), "off": len(blob), "len": len(jpeg), "w": 640, "h": 480, "det": det})
            blob += jpeg
        records.append({"k": "state", "t": 0.5, "msg": {"action": "fährt"}})
        records.append({"k": "calc", "t": 3.2, "what": "rutscht"})
        records.append({"k": "end", "t": 6.0})
        with gzip.open(self.path, "wt", encoding="utf-8") as f:
            f.write("".join(json.dumps(r) + "\n" for r in records))
        (Path(self.dir.name) / "2026-01-01_10-00-00.cam").write_bytes(blob)

    def test_stats_lists_frames_and_events(self):
        from tools.cam_review import Run, stats
        text = stats(Run(self.path))
        self.assertEqual(text.count("fährt"), 5)       # das erste Bild kommt vor dem ersten Zustand
        self.assertIn("3.2s rutscht", text)

    def test_sheet_with_events_contains_event_frame(self):
        from PIL import Image
        from tools.cam_review import Run, pick, sheet
        from pathlib import Path
        run = Run(self.path)
        picked = pick(run, 4, events=True)
        self.assertEqual([round(f["ct"]) for f, _ in picked], [3])
        out = sheet(run, 4, 2, False, Path(self.dir.name) / "s.jpg")
        self.assertEqual(Image.open(out).size, (800, 600))      # 4 Bilder, 2 Spalten, 400 x 300

    def test_frame_extracts_jpeg(self):
        from PIL import Image
        from tools.cam_review import Run
        run = Run(self.path)
        self.assertEqual(run.image(run.frame_near(2.2)).size, (640, 480))


if __name__ == "__main__":
    unittest.main()
