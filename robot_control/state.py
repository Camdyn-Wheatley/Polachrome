"""
state.py — Runtime state containers for the combat robot controller.

Replaces all module-level mutable globals with explicit, passable state objects.
"""

from __future__ import annotations

import threading
from dataclasses import dataclass, field
from typing import Optional, Tuple

import numpy as np

from robot_control.filters import KalmanTracker
from robot_control.arena import ArenaBounds


# ── Shutdown event ───────────────────────────────────────────────────────────
shutdown_event = threading.Event()
"""Cross-thread shutdown signal. All loops should check ``not shutdown_event.is_set()``."""


@dataclass
class TrackingState:
    """Mutable state for the detection/tracking pipeline."""

    kf1: KalmanTracker
    kf2: KalmanTracker
    arena: ArenaBounds

    # Robot orientation (determined by which ArUco tag is visible)
    robot_upright: bool = True  # True = top tag visible, False = bottom tag

    # Source of the latest R1 / R2 positions (for logging/debug)
    r1_source: str = ""
    r2_source: str = ""


@dataclass
class AppState:
    """Mutable state for the main application loop."""

    auto_mode: bool = False
    show_bounds: bool = True
    current_path: Optional[np.ndarray] = None
    last_r1_path: Optional[Tuple[int, int]] = None
    last_r2_path: Optional[Tuple[int, int]] = None
    current_heading: float = 0.0
    ch1: int = 1500
    ch2: int = 1500
    ch3: int = 1000
