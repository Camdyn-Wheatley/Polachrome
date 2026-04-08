"""
state.py — Runtime state containers for the combat robot controller.

Replaces all module-level mutable globals with explicit, passable state objects.
"""

from __future__ import annotations

import threading
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np

from robot_control.filters import KalmanTracker
from robot_control.arena import ArenaBounds
from robot_control.config import Config


# ── Shutdown event (Action 9) ────────────────────────────────────────────────
shutdown_event = threading.Event()
"""Cross-thread shutdown signal. All loops should check ``not shutdown_event.is_set()``."""


@dataclass
class TrackingState:
    """Mutable state for the detection/tracking pipeline."""

    kf1: KalmanTracker
    kf2: KalmanTracker
    arena: ArenaBounds

    # R2 color tracking
    hsv_lower: np.ndarray = field(
        default_factory=lambda: np.array([0, 100, 100])
    )
    hsv_upper: np.ndarray = field(
        default_factory=lambda: np.array([10, 255, 255])
    )
    tolerance_r2: Dict[str, int] = field(default_factory=dict)
    picked_hsv: Optional[Tuple[int, int, int]] = None
    current_hsv_frame: Optional[np.ndarray] = None

    # AprilTag ROI state
    roi: Dict[int, Optional[Tuple[int, int, int, int]]] = field(
        default_factory=dict
    )
    roi_miss: Dict[int, int] = field(default_factory=dict)
    mid_source: str = ""
    tag_offsets: Dict[int, Tuple[float, float, Optional[float]]] = field(
        default_factory=dict
    )

    def __post_init__(self) -> None:
        if not self.tolerance_r2:
            self.tolerance_r2 = {"h": 15, "s": 60, "v": 60}


@dataclass
class AppState:
    """Mutable state for the main application loop."""

    auto_mode: bool = False
    show_bounds: bool = True
    target_id: int = 3
    current_path: Optional[np.ndarray] = None
    last_r1_path: Optional[Tuple[int, int]] = None
    last_r2_path: Optional[Tuple[int, int]] = None
    current_heading: float = 0.0
    ch1: int = 1500
    ch2: int = 1500
    ch3: int = 1000

    # Camera reference for trackbar exposure control
    cam_ref: object = None
