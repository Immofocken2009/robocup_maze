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

    def capture(self):
        if not self.frames:
            raise RuntimeError("keine Bilder mehr")
        return self.frames.pop(0)

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


if __name__ == "__main__":
    unittest.main()
