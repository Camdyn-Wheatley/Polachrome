"""
profile.py — Color profile persistence (save/load) for R2 colour tracking.
"""

from __future__ import annotations

import json
import logging
import os
from typing import Optional

import numpy as np

from robot_control.state import TrackingState
from robot_control.ui.trackbars import _sync_trackbars, apply_tolerance_r2

logger = logging.getLogger(__name__)


def save_profile(state: TrackingState, path: str) -> None:
    """Save the R2 colour profile to JSON."""
    with open(path, "w") as f:
        json.dump(
            {
                "lower": state.hsv_lower.tolist(),
                "upper": state.hsv_upper.tolist(),
                "picked": list(state.picked_hsv) if state.picked_hsv else None,
                "tolerance": state.tolerance_r2,
            },
            f,
            indent=2,
        )
    logger.info("Saved R2 profile to %s", path)


def load_profile(state: TrackingState, path: str) -> None:
    """Load the R2 colour profile from JSON."""
    if not os.path.exists(path):
        return
    with open(path) as f:
        d = json.load(f)
    state.hsv_lower = np.array(d["lower"])
    state.hsv_upper = np.array(d["upper"])
    state.picked_hsv = tuple(d["picked"]) if d.get("picked") else None
    state.tolerance_r2.update(d.get("tolerance", {}))
    logger.info("Loaded R2 color profile from %s", path)
    try:
        _sync_trackbars(state)
    except Exception:
        pass  # trackbars may not exist yet
