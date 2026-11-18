"""Kamerabilder eines Laufs aufzeichnen: logs/<Lauf>.cam (JPEG-Bilder hintereinander) und je Bild eine Zeile `cam` im Lauf-Log.

Die Zeile hat: t (Zeit des Schreibens wie alle Einträge), ct (Aufnahmezeit), off und len (Lage in der .cam-Datei), w, h, det (Auswertung
des kleinen Bilds, Koordinaten dort). Rohbilder ohne Einzeichnung: so lässt sich die Auswertung später auf denselben Bildern mit anderen
Schwellen wiederholen. Kodieren und Schreiben laufen in einem eigenen Thread, die Bildschleife wartet nie darauf.
"""
import logging
import os
import queue
import shutil
import threading
from pathlib import Path

import config
from hardware.run_log import SUFFIX

log = logging.getLogger("vision")


def cam_path(log_path: Path) -> Path:
    """logs/2026-11-18_12-00-00.jsonl.gz -> logs/2026-11-18_12-00-00.cam"""
    name = log_path.name
    return log_path.with_name((name[:-len(SUFFIX)] if name.endswith(SUFFIX) else log_path.stem) + ".cam")


def free_mb(path: Path) -> float:
    return shutil.disk_usage(path.parent).free / 1e6


class CamRecorder:
    def __init__(self, path: Path, rec, quality: int, encode):
        self.path = path
        self.rec = rec
        self.quality = quality
        self.encode = encode
        self.frames = 0
        self.dropped = 0
        self.bytes = 0
        self.warning: str | None = None
        self._queue: queue.Queue = queue.Queue(maxsize=config.CAM_REC_QUEUE_MAX)
        self._file = open(path, "wb")
        self._thread = threading.Thread(target=self._loop, name="cam_record", daemon=True)
        self._thread.start()

    def submit(self, capture_t: float, rgb, det: dict) -> bool:
        try:
            self._queue.put_nowait((capture_t, rgb, det))
            return True
        except queue.Full:
            self.dropped += 1
            return False

    def _loop(self):
        while True:
            item = self._queue.get()
            if item is None:
                break
            capture_t, rgb, det = item
            try:
                if self.warning is None:
                    jpeg = self.encode(rgb, self.quality)
                    self._file.write(jpeg)
                    self.rec.write("cam", ct=capture_t, off=self.bytes, len=len(jpeg), w=int(rgb.shape[1]), h=int(rgb.shape[0]),
                                   det=det)
                    self.bytes += len(jpeg)
                    self.frames += 1
                    if self.frames % config.CAM_REC_CHECK_EVERY == 0 and free_mb(self.path) < config.CAM_REC_MIN_FREE_MB:
                        self.warning = f"Kamera-Aufzeichnung gestoppt: nur noch {free_mb(self.path):.0f} MB frei"
                        log.warning(self.warning)
            except Exception as exc:
                self.warning = f"Kamera-Aufzeichnung gestoppt ({type(exc).__name__}: {exc})"
                log.exception("Kamera-Aufzeichnung")

    def close(self):
        self._queue.put(None)
        self._thread.join(timeout=15)
        try:
            self._file.flush()
            os.fsync(self._file.fileno())
        finally:
            self._file.close()
