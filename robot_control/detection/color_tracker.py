"""
color_tracker.py — HSV color-based tracking for Robot 2 and Robot 1 fallback.

Robot 2: User-pickable HSV segmentation with trackbar tuning.
Robot 1: Adaptive color tracker with periodic re-sampling.
"""

from __future__ import annotations

import json
import logging
import os
from typing import Optional, Tuple

import cv2
import numpy as np

from robot_control.config import Config

logger = logging.getLogger(__name__)


# ── Robot 2 color detection (Action 6: accepts hsv_frame directly) ────────

def detect_r2_color(
    hsv_frame: np.ndarray,
    hsv_lower: np.ndarray,
    hsv_upper: np.ndarray,
    min_contour_area: int,
) -> Tuple[Optional[Tuple[int, int, float, np.ndarray]], np.ndarray]:
    """
    Detect Robot 2 by HSV color segmentation.

    Returns ``(result, mask)`` where result is ``(cx, cy, area, contour)``
    or ``None`` if nothing detected.
    """
    if hsv_lower[0] > hsv_upper[0]:
        # Hue wrap-around (e.g., red spanning 0°/360°)
        mask = cv2.bitwise_or(
            cv2.inRange(
                hsv_frame, np.array([0, hsv_lower[1], hsv_lower[2]]), hsv_upper
            ),
            cv2.inRange(
                hsv_frame, hsv_lower, np.array([179, hsv_upper[1], hsv_upper[2]])
            ),
        )
    else:
        mask = cv2.inRange(hsv_frame, hsv_lower, hsv_upper)

    k = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
    mask = cv2.morphologyEx(mask, cv2.MORPH_DILATE, k)

    ctrs, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not ctrs:
        return None, mask
    best = max(ctrs, key=cv2.contourArea)
    if cv2.contourArea(best) < min_contour_area:
        return None, mask
    M = cv2.moments(best)
    if M["m00"] == 0:
        return None, mask
    return (
        int(M["m10"] / M["m00"]),
        int(M["m01"] / M["m00"]),
        cv2.contourArea(best),
        best,
    ), mask


# ── Robot 1 adaptive color tracker ───────────────────────────────────────────

class Robot1ColorTracker:
    """Color-based fallback tracker for Robot 1 with adaptive re-sampling."""

    def __init__(self, cfg: Config) -> None:
        self._cfg = cfg
        self.hsv_center: Optional[Tuple[int, int, int]] = None
        self.tol = {"h": 18, "s": 70, "v": 70}
        self._cnt = 0
        self._profile = "robot1_color.json"
        self._load()

    def _load(self) -> None:
        if os.path.exists(self._profile):
            with open(self._profile) as f:
                d = json.load(f)
            self.hsv_center = tuple(d["center"])
            self.tol.update(d.get("tol", {}))
            logger.info("R1 color loaded: %s", self.hsv_center)

    def save(self) -> None:
        if self.hsv_center is None:
            return
        with open(self._profile, "w") as f:
            json.dump(
                {"center": list(self.hsv_center), "tol": self.tol}, f, indent=2
            )

    def maybe_resample(
        self, hsv_frame: np.ndarray, r1_pos: Optional[Tuple[int, int]]
    ) -> None:
        """Periodically re-sample R1's color from the Kalman estimate."""
        self._cnt += 1
        if r1_pos is None or self._cnt % self._cfg.color_resample_interval != 0:
            return
        x, y = r1_pos
        r = self._cfg.color_sample_radius
        fh, fw = hsv_frame.shape[:2]
        patch = hsv_frame[max(0, y - r) : min(fh, y + r), max(0, x - r) : min(fw, x + r)]
        if patch.size == 0:
            return
        h, s, v = (int(np.median(patch[:, :, c])) for c in range(3))
        if s > 40:
            self.hsv_center = (h, s, v)
            self.save()
            logger.info("R1 color resampled → HSV %s", self.hsv_center)

    def detect(
        self, frame: np.ndarray, hsv_frame: np.ndarray
    ) -> Optional[Tuple[int, int]]:
        """Detect Robot 1 by color. Returns ``(cx, cy)`` or ``None``."""
        if self.hsv_center is None:
            return None
        h, s, v = self.hsv_center
        lo = np.array(
            [max(0, h - self.tol["h"]), max(0, s - self.tol["s"]), max(0, v - self.tol["v"])]
        )
        hi = np.array(
            [
                min(179, h + self.tol["h"]),
                min(255, s + self.tol["s"]),
                min(255, v + self.tol["v"]),
            ]
        )
        if lo[0] > hi[0]:
            mask = cv2.bitwise_or(
                cv2.inRange(hsv_frame, np.array([0, lo[1], lo[2]]), hi),
                cv2.inRange(hsv_frame, lo, np.array([179, hi[1], hi[2]])),
            )
        else:
            mask = cv2.inRange(hsv_frame, lo, hi)
        k = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
        ctrs, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not ctrs:
            return None
        best = max(ctrs, key=cv2.contourArea)
        if cv2.contourArea(best) < self._cfg.min_contour_area:
            return None
        M = cv2.moments(best)
        if M["m00"] == 0:
            return None
        return (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
