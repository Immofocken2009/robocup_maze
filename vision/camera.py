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
from vision.recording import CamRecorder, cam_path, free_mb

log = logging.getLogger("vision")
RETRY_S = 2.0       # nach einem Fehler so lange warten


def record_settings():
    """(Größe, Bilder/s, JPEG-Qualität) der Aufzeichnung laut config.CAM_RECORD_MODE, None bei „aus“."""
    mode = config.CAM_RECORD_MODE
    if mode == "debug":
        return tuple(config.CAM_DEBUG_SIZE), config.CAM_DEBUG_FPS, config.CAM_DEBUG_QUALITY
    if mode == "normal":
        return tuple(config.CAM_SIZE), config.CAM_REC_FPS, config.CAM_REC_QUALITY
    return None


class PiCameraSource:
    """Bilder von der Kamera am CSI-Anschluss (picamera2). `capture()` liefert (klein, groß): klein = RGB-Feld in CAM_SIZE für die
    Auswertung, groß = RGB-Feld in Aufzeichnungsgröße, nur wenn verlangt (sonst None)."""

    def __init__(self, size=None, fps=None):
        from picamera2 import Picamera2         # fehlt die Bibliothek, schlägt es hier fehl (ImportError)
        size = tuple(size or config.CAM_SIZE)
        rec = record_settings()
        big = rec[0] if rec and tuple(rec[0]) != size else None
        self.cam = Picamera2()
        controls = {"FrameRate": float(fps or config.CAM_FPS)}
        # "BGR888" legt die Bytes je Pixel als [R, G, B] ab (picamera2 benennt die Formate nach der Speicherordnung)
        if big:     # zwei Ströme: klein für die Auswertung (vom ISP geliefert, fast gratis), groß nur für die Aufzeichnung
            cfg = self.cam.create_video_configuration(main={"size": big, "format": "BGR888"},
                                                      lores={"size": size, "format": "BGR888"}, controls=controls)
        else:
            cfg = self.cam.create_video_configuration(main={"size": size, "format": "BGR888"}, controls=controls)
        self.two_streams = bool(big)
        self.cam.configure(cfg)
        self.cam.start()

    def capture(self, full: bool = False):
        if not self.two_streams:
            small = self.cam.capture_array("main")
            return small, (small if full else None)
        request = self.cam.capture_request()
        try:
            small = request.make_array("lores")
            return small, (request.make_array("main") if full else None)
        finally:
            request.release()

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
    Bild kodieren und senden kostet Rechenzeit: nur, wenn `wanted()` wahr ist (Browser verbunden).
    Im Lauf (`start_recording`) werden zusätzlich Rohbilder aufgezeichnet (vision/recording.py, config.CAM_RECORD_MODE)."""

    def __init__(self, on_message, wanted=lambda: True, source_factory=PiCameraSource, encode=None):
        self.on_message = on_message
        self.wanted = wanted
        self.source_factory = source_factory
        self.encode = encode or encode_jpeg
        self.warning: str | None = None
        self.detections: dict | None = None      # letzte Auswertung (später für die Fahrsteuerung)
        self.frames = 0
        self.recorder: CamRecorder | None = None
        self.rec_warning: str | None = None
        self._rec_interval = 0.0
        self._rec_due = 0.0
        self._rec_clock = None
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self):
        self._thread = threading.Thread(target=self._run, name="camera", daemon=True)
        self._thread.start()

    def close(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=3)
        self.stop_recording()

    # --- Aufzeichnung (nur im Lauf) -------------------------------------------------

    def start_recording(self, rec):
        """Mit dem Lauf-Log `rec` (hardware.run_log.Recorder, nach `start_run`) mitschreiben."""
        self.stop_recording()
        settings = record_settings()
        self.rec_warning = None
        if settings is None or rec.log is None:
            return
        size, fps, quality = settings
        path = cam_path(rec.log.path)
        try:
            if free_mb(path) < config.CAM_REC_MIN_FREE_MB:
                self.rec_warning = f"Kamera-Aufzeichnung aus: nur {free_mb(path):.0f} MB frei"
                log.warning(self.rec_warning)
                return
            self.recorder = CamRecorder(path, rec, quality, self.encode)
        except OSError as exc:
            self.rec_warning = f"Kamera-Aufzeichnung aus ({exc})"
            log.warning(self.rec_warning)
            return
        self._rec_interval = 1.0 / fps
        self._rec_due = 0.0
        self._rec_clock = rec.now
        log.info("Kamera-Aufzeichnung: %s (%dx%d, %.1f Bilder/s)", path.name, size[0], size[1], fps)

    def stop_recording(self):
        recorder, self.recorder = self.recorder, None
        if recorder is not None:
            recorder.close()
            log.info("Kamera-Aufzeichnung beendet: %d Bilder, %d verworfen, %.0f MB", recorder.frames, recorder.dropped,
                     recorder.bytes / 1e6)

    def recording_info(self) -> dict:
        r = self.recorder
        return {"on": r is not None, "frames": r.frames if r else 0, "dropped": r.dropped if r else 0,
                "mode": config.CAM_RECORD_MODE}

    # --- Schleife -------------------------------------------------------------------

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
                recorder = self.recorder        # (kann zwischendurch None werden: lokal merken)
                now = time.monotonic()
                want_full = recorder is not None and now >= self._rec_due
                small, full = source.capture(want_full)
                t0 = time.monotonic()
                det = analyzer.analyse(small)
                load_ms = (time.monotonic() - t0) * 1000
                self.detections = det
                self.frames += 1
                fps_n += 1
                if want_full and full is not None:
                    self._rec_due = now + self._rec_interval
                    recorder.submit(self._rec_clock(), full, det)
                if t0 - fps_t >= 2.0:
                    fps, fps_t, fps_n = fps_n / (t0 - fps_t), t0, 0
                if t0 - last_sent >= config.CAM_PUBLISH_S and self.wanted():
                    last_sent = t0
                    jpeg = self.encode(small, config.CAM_JPEG_QUALITY)
                    self.on_message({"type": "cam", "id": self.frames, "fps": round(fps, 1), "load_ms": round(load_ms, 1),
                                     "jpeg": base64.b64encode(jpeg).decode("ascii"), "det": det,
                                     "rec": self.recording_info()})
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
