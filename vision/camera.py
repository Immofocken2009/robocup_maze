"""Kamera vorne: nimmt Bilder auf (eigener Thread), wertet sie aus und gibt dem Roboterprogramm Nachrichten für die Webseite.

Die Quelle (`PiCameraSource`, Bibliothek picamera2) ist austauschbar, damit Tests ohne Kamera laufen. Ohne picamera2 oder ohne Kamera
läuft der Roboter normal weiter (Warnung in `warning`).
"""
import base64
import io
import logging
import threading
import time

import config
from vision.analysis import Analyzer

log = logging.getLogger("vision")
RETRY_S = 2.0       # nach einem Fehler so lange warten


class PiCameraSource:
    """Bilder von der Kamera am CSI-Anschluss (picamera2). Liefert RGB-Felder (Höhe x Breite x 3, uint8)."""

    def __init__(self, size=None, fps=None):
        from picamera2 import Picamera2         # fehlt die Bibliothek, schlägt es hier fehl (ImportError)
        size = size or config.CAM_SIZE
        self.cam = Picamera2()
        # "BGR888" legt die Bytes je Pixel als [R, G, B] ab (picamera2 benennt die Formate nach der Speicherordnung)
        self.cam.configure(self.cam.create_video_configuration(
            main={"size": tuple(size), "format": "BGR888"}, controls={"FrameRate": float(fps or config.CAM_FPS)}))
        self.cam.start()

    def capture(self):
        return self.cam.capture_array("main")

    def close(self):
        try:
            self.cam.stop()
        finally:
            self.cam.close()


def encode_jpeg(rgb, quality: int) -> bytes:
    from PIL import Image
    buf = io.BytesIO()
    Image.fromarray(rgb).save(buf, "JPEG", quality=quality)
    return buf.getvalue()


class CameraFeed:
    """Hält die Kamera offen, wertet jedes Bild aus und meldet Bild und Auswertung an `on_message` (Dict, Typ „cam“).
    Bild kodieren und senden kostet Rechenzeit: nur, wenn `wanted()` wahr ist (Browser verbunden)."""

    def __init__(self, on_message, wanted=lambda: True, source_factory=PiCameraSource):
        self.on_message = on_message
        self.wanted = wanted
        self.source_factory = source_factory
        self.warning: str | None = None
        self.detections: dict | None = None      # letzte Auswertung (später für die Fahrsteuerung)
        self.frames = 0
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self):
        self._thread = threading.Thread(target=self._run, name="camera", daemon=True)
        self._thread.start()

    def close(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=3)

    def _run(self):
        analyzer = Analyzer()
        last_sent = 0.0
        fps_t, fps_n, fps = time.monotonic(), 0, 0.0
        source = None
        while not self._stop.is_set():
            try:
                if source is None:
                    source = self.source_factory()
                    self.warning = None
                    log.info("Kamera bereit")
                rgb = source.capture()
                t0 = time.monotonic()
                det = analyzer.analyse(rgb)
                load_ms = (time.monotonic() - t0) * 1000
                self.detections = det
                self.frames += 1
                fps_n += 1
                if t0 - fps_t >= 2.0:
                    fps, fps_t, fps_n = fps_n / (t0 - fps_t), t0, 0
                if t0 - last_sent >= config.CAM_PUBLISH_S and self.wanted():
                    last_sent = t0
                    jpeg = encode_jpeg(rgb, config.CAM_JPEG_QUALITY)
                    self.on_message({"type": "cam", "id": self.frames, "fps": round(fps, 1), "load_ms": round(load_ms, 1),
                                     "jpeg": base64.b64encode(jpeg).decode("ascii"), "det": det})
            except Exception as exc:
                text = f"{type(exc).__name__}: {exc}"
                if self.warning is None:
                    log.warning("Kamera: %s", text)
                self.warning = f"Kamera nicht verfügbar ({text})"
                self._release(source)
                source = None
                self._stop.wait(RETRY_S)
        self._release(source)

    @staticmethod
    def _release(source):
        if source is not None:
            try:
                source.close()
            except Exception:
                log.exception("Kamera schließen")
