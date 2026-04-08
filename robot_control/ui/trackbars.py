"""
trackbars.py — HSV tuning trackbar management for Robot 2 color picking.
"""

from __future__ import annotations

import logging
from typing import Optional, Tuple

import cv2
import numpy as np

from robot_control.camera import CameraStream
from robot_control.state import TrackingState

logger = logging.getLogger(__name__)

TRACKBAR_WIN = "Color Tuning"


def _clamp_v(v: int, lo: int, hi: int) -> int:
    return max(lo, min(hi, v))


def apply_tolerance_r2(
    state: TrackingState,
    h: Optional[int] = None,
    s: Optional[int] = None,
    v: Optional[int] = None,
) -> None:
    """Recompute R2 HSV bounds from centre + tolerance."""
    if h is None:
        if state.picked_hsv is None:
            return
        h, s, v = state.picked_hsv
    state.hsv_lower = np.array(
        [
            _clamp_v(h - state.tolerance_r2["h"], 0, 179),
            _clamp_v(s - state.tolerance_r2["s"], 0, 255),
            _clamp_v(v - state.tolerance_r2["v"], 0, 255),
        ]
    )
    state.hsv_upper = np.array(
        [
            _clamp_v(h + state.tolerance_r2["h"], 0, 179),
            _clamp_v(s + state.tolerance_r2["s"], 0, 255),
            _clamp_v(v + state.tolerance_r2["v"], 0, 255),
        ]
    )


def create_trackbars(state: TrackingState, exposure: Optional[int]) -> None:
    """Create the HSV tuning trackbar window."""
    cv2.namedWindow(TRACKBAR_WIN, cv2.WINDOW_NORMAL)
    h = int(state.picked_hsv[0]) if state.picked_hsv else 90
    s = int(state.picked_hsv[1]) if state.picked_hsv else 150
    v = int(state.picked_hsv[2]) if state.picked_hsv else 150
    noop = lambda _: None
    cv2.createTrackbar("H Center", TRACKBAR_WIN, h, 179, noop)
    cv2.createTrackbar("H Tolerance", TRACKBAR_WIN, state.tolerance_r2["h"], 90, noop)
    cv2.createTrackbar("S Center", TRACKBAR_WIN, s, 255, noop)
    cv2.createTrackbar("S Tolerance", TRACKBAR_WIN, state.tolerance_r2["s"], 127, noop)
    cv2.createTrackbar("V Center", TRACKBAR_WIN, v, 255, noop)
    cv2.createTrackbar("V Tolerance", TRACKBAR_WIN, state.tolerance_r2["v"], 127, noop)
    cv2.createTrackbar(
        "Exposure (0=auto)",
        TRACKBAR_WIN,
        0 if exposure is None else int(exposure),
        500,
        noop,
    )
    cv2.resizeWindow(TRACKBAR_WIN, 520, 340)


def _sync_trackbars(state: TrackingState) -> None:
    """Push current state values back into the trackbar UI."""
    if state.picked_hsv:
        cv2.setTrackbarPos("H Center", TRACKBAR_WIN, int(state.picked_hsv[0]))
        cv2.setTrackbarPos("S Center", TRACKBAR_WIN, int(state.picked_hsv[1]))
        cv2.setTrackbarPos("V Center", TRACKBAR_WIN, int(state.picked_hsv[2]))
    cv2.setTrackbarPos("H Tolerance", TRACKBAR_WIN, state.tolerance_r2["h"])
    cv2.setTrackbarPos("S Tolerance", TRACKBAR_WIN, state.tolerance_r2["s"])
    cv2.setTrackbarPos("V Tolerance", TRACKBAR_WIN, state.tolerance_r2["v"])


def read_trackbars(state: TrackingState, cam: Optional[CameraStream] = None) -> None:
    """Read trackbar positions and update state."""
    hc = cv2.getTrackbarPos("H Center", TRACKBAR_WIN)
    ht = cv2.getTrackbarPos("H Tolerance", TRACKBAR_WIN)
    sc = cv2.getTrackbarPos("S Center", TRACKBAR_WIN)
    st = cv2.getTrackbarPos("S Tolerance", TRACKBAR_WIN)
    vc = cv2.getTrackbarPos("V Center", TRACKBAR_WIN)
    vt = cv2.getTrackbarPos("V Tolerance", TRACKBAR_WIN)
    state.tolerance_r2["h"] = max(1, ht)
    state.tolerance_r2["s"] = max(1, st)
    state.tolerance_r2["v"] = max(1, vt)
    apply_tolerance_r2(state, hc, sc, vc)
    if cam is not None:
        exp = cv2.getTrackbarPos("Exposure (0=auto)", TRACKBAR_WIN)
        if exp == 0:
            cam.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3)
        else:
            cam.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
            cam.set(cv2.CAP_PROP_EXPOSURE, exp)


def make_mouse_callback(state: TrackingState):
    """Return a mouse callback closure for R2 color picking."""
    def mouse_pick(event: int, x: int, y: int, _flags: int, _param: object) -> None:
        if event == cv2.EVENT_LBUTTONDOWN and state.current_hsv_frame is not None:
            h, s, v = state.current_hsv_frame[y, x]
            state.picked_hsv = (int(h), int(s), int(v))
            cv2.setTrackbarPos("H Center", TRACKBAR_WIN, int(h))
            cv2.setTrackbarPos("S Center", TRACKBAR_WIN, int(s))
            cv2.setTrackbarPos("V Center", TRACKBAR_WIN, int(v))
            apply_tolerance_r2(state, int(h), int(s), int(v))
            logger.info("R2 pick HSV %s", state.picked_hsv)
    return mouse_pick
