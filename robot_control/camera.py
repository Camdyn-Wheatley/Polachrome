"""
camera.py — Threaded camera capture for the combat robot controller.

Decouples USB frame delivery from the main loop to prevent stalls.
"""

from __future__ import annotations

import logging
import threading
import time
from typing import Optional

import cv2
import numpy as np

from robot_control.config import Config

logger = logging.getLogger(__name__)


class CameraStream:
    """
    Producer thread that continuously reads frames from a V4L2 camera and
    stores the latest frame in a thread-safe buffer (size 1).
    """

    def __init__(self, cfg: Config) -> None:
        self._cfg = cfg

        self.cap = cv2.VideoCapture(cfg.camera_device, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self.cap = cv2.VideoCapture(cfg.camera_device)
        if not self.cap.isOpened():
            raise RuntimeError(f"Cannot open camera: {cfg.camera_device}")

        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, cfg.frame_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cfg.frame_height)
        self.cap.set(cv2.CAP_PROP_FPS, 60)
        self._configure_exposure()

        self._frame: Optional[np.ndarray] = None
        self._lock = threading.Lock()
        self._stop = False
        self._last_frame_time: float = time.time()
        self._frame_time_lock = threading.Lock()

        self._thread = threading.Thread(target=self._grab, daemon=True)
        self._thread.start()

        deadline = time.time() + 5.0
        while self._frame is None:
            if time.time() > deadline:
                self._stop = True
                self.cap.release()
                raise RuntimeError("Camera timed out — no frames received.")
            time.sleep(0.02)

        w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = self.cap.get(cv2.CAP_PROP_FPS)
        logger.info("Camera ready  %dx%d  @ %.0ffps", w, h, fps)

    def _configure_exposure(self) -> None:
        if self._cfg.exposure is None:
            self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3)
        else:
            self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
            self.cap.set(cv2.CAP_PROP_EXPOSURE, self._cfg.exposure)

    def _grab(self) -> None:
        while not self._stop:
            ret, frame = self.cap.read()
            if ret:
                with self._lock:
                    self._frame = frame
                with self._frame_time_lock:
                    self._last_frame_time = time.time()

    def read(self) -> Optional[np.ndarray]:
        """Return a copy of the latest frame, or ``None`` if unavailable."""
        with self._lock:
            return self._frame.copy() if self._frame is not None else None

    @property
    def last_frame_age_ms(self) -> float:
        """Milliseconds since the last frame was received."""
        with self._frame_time_lock:
            return (time.time() - self._last_frame_time) * 1000.0

    def get(self, prop: int) -> float:
        return self.cap.get(prop)

    def set(self, prop: int, val: float) -> None:
        self.cap.set(prop, val)

    def release(self) -> None:
        """Stop the capture thread and release the camera."""
        self._stop = True
        self._thread.join(timeout=1.0)
        self.cap.release()
        logger.info("Camera released")
