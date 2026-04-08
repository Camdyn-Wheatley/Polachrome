"""
arena.py — Arena bounds definition and safe-zone enforcement.
"""

from __future__ import annotations

import logging
import math
from typing import Tuple

import cv2
import numpy as np

from robot_control.config import Config

logger = logging.getLogger(__name__)


class ArenaBounds:
    """Rectangular arena with a configurable robot-size safety inset."""

    def __init__(self, cfg: Config) -> None:
        self.tl_raw: Tuple[int, int] = cfg.arena_tl
        self.br_raw: Tuple[int, int] = cfg.arena_br
        self.offset: int = cfg.robot_offset
        self._recompute()

    def _recompute(self) -> None:
        self.safe_tl = (
            self.tl_raw[0] + self.offset,
            self.tl_raw[1] + self.offset,
        )
        self.safe_br = (
            self.br_raw[0] - self.offset,
            self.br_raw[1] - self.offset,
        )

    def clamp(self, pt: Tuple[float, float]) -> Tuple[float, float]:
        """Force a point into the safe zone."""
        x = max(self.safe_tl[0], min(self.safe_br[0], pt[0]))
        y = max(self.safe_tl[1], min(self.safe_br[1], pt[1]))
        return (x, y)

    def inside(self, pt: Tuple[float, float]) -> bool:
        """Test whether a point is inside the safe zone."""
        return (
            self.safe_tl[0] <= pt[0] <= self.safe_br[0]
            and self.safe_tl[1] <= pt[1] <= self.safe_br[1]
        )

    def change_offset(self, delta: int) -> None:
        """Adjust the safety offset at runtime."""
        self.offset = max(0, self.offset + delta)
        self._recompute()
        logger.info("Robot offset = %dpx", self.offset)

    def draw(self, frame: np.ndarray) -> None:
        """Draw both the raw arena boundary and the dashed safe zone."""
        cv2.rectangle(frame, self.tl_raw, self.br_raw, (255, 255, 255), 2)
        _draw_dashed_rect(frame, self.safe_tl, self.safe_br, (255, 220, 0), 1)
        cv2.putText(
            frame,
            f"safe zone  offset={self.offset}px",
            (self.safe_tl[0] + 4, self.safe_tl[1] + 16),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.4,
            (255, 220, 0),
            1,
        )


def _draw_dashed_rect(
    frame: np.ndarray,
    tl: Tuple[int, int],
    br: Tuple[int, int],
    color: Tuple[int, int, int],
    thickness: int,
    dash: int = 12,
) -> None:
    """Draw a dashed rectangle outline."""
    pts = [
        (tl[0], tl[1]),
        (br[0], tl[1]),
        (br[0], br[1]),
        (tl[0], br[1]),
        (tl[0], tl[1]),
    ]
    for i in range(len(pts) - 1):
        x0, y0 = pts[i]
        x1, y1 = pts[i + 1]
        dx, dy = x1 - x0, y1 - y0
        length = math.hypot(dx, dy)
        if length < 1:
            continue
        steps = int(length / (dash * 2))
        for s in range(steps):
            t0 = (2 * s) * dash / length
            t1 = (2 * s + 1) * dash / length
            cv2.line(
                frame,
                (int(x0 + dx * t0), int(y0 + dy * t0)),
                (int(x0 + dx * t1), int(y0 + dy * t1)),
                color,
                thickness,
            )
