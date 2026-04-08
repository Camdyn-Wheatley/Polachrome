"""
filters.py — Kalman tracker for the combat robot controller.

6-state constant-acceleration Kalman filter (x, y, vx, vy, ax, ay)
with variable dt and coast-through-occlusion support.
"""

from __future__ import annotations

import logging
from typing import Optional, Tuple

import cv2
import numpy as np

logger = logging.getLogger(__name__)


class KalmanTracker:
    """
    Wraps an OpenCV KalmanFilter with:
      - Variable-dt transition matrix updates.
      - Coast counter for dead-reckoning through occlusions.
      - Correct predict→correct ordering (Action 8).
    """

    def __init__(
        self,
        max_coast_frames: int = 30,
        proc_noise: float = 0.5,
        meas_noise: float = 4.0,
    ) -> None:
        self._max_coast = max_coast_frames
        self.kf = cv2.KalmanFilter(6, 2)
        self.kf.measurementMatrix = np.zeros((2, 6), np.float32)
        self.kf.measurementMatrix[0, 0] = 1.0
        self.kf.measurementMatrix[1, 1] = 1.0
        self.kf.processNoiseCov = np.eye(6, dtype=np.float32) * proc_noise
        self.kf.measurementNoiseCov = np.eye(2, dtype=np.float32) * meas_noise
        self.kf.errorCovPost = np.eye(6, dtype=np.float32) * 500.0
        self._init = False
        self.coast = 0

    def _set_dt(self, dt: float) -> None:
        d2 = 0.5 * dt * dt
        self.kf.transitionMatrix = np.array(
            [
                [1, 0, dt, 0, d2, 0],
                [0, 1, 0, dt, 0, d2],
                [0, 0, 1, 0, dt, 0],
                [0, 0, 0, 1, 0, dt],
                [0, 0, 0, 0, 1, 0],
                [0, 0, 0, 0, 0, 1],
            ],
            np.float32,
        )

    def update(self, x: float, y: float, dt: float) -> Tuple[int, int]:
        """
        Incorporate a new measurement.

        Follows the correct OpenCV Kalman sequence: predict → correct.
        Returns the corrected position estimate.
        """
        self._set_dt(dt)
        if not self._init:
            self.kf.statePost = np.array(
                [x, y, 0, 0, 0, 0], np.float32
            ).reshape(6, 1)
            self._init = True
        # Action 8: predict BEFORE correct
        self.kf.predict()
        self.kf.correct(np.array([[x], [y]], np.float32))
        self.coast = 0
        s = self.kf.statePost
        return (int(s[0, 0]), int(s[1, 0]))

    def predict(self, dt: float) -> Optional[Tuple[int, int]]:
        """Dead-reckon one step without a measurement."""
        if not self._init:
            return None
        self.coast += 1
        if self.coast > self._max_coast:
            return None
        self._set_dt(dt)
        p = self.kf.predict()
        return (int(p[0, 0]), int(p[1, 0]))

    @property
    def coasting(self) -> bool:
        return self._init and 0 < self.coast <= self._max_coast

    @property
    def coast_ratio(self) -> float:
        return min(self.coast / self._max_coast, 1.0)

    def velocity(self) -> Tuple[float, float]:
        """Return the current velocity estimate ``(vx, vy)``."""
        s = self.kf.statePost
        return float(s[2, 0]), float(s[3, 0])

    def position_float(self) -> Optional[Tuple[float, float]]:
        """Return the current position as floats, or ``None`` if uninitialised."""
        if not self._init:
            return None
        s = self.kf.statePost
        return float(s[0, 0]), float(s[1, 0])
