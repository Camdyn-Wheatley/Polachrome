"""
state.py — Runtime state containers for the combat robot controller.

Stores the latest unified vision state from the ArUco and Depth Segmenter.
"""

from __future__ import annotations

import threading
from dataclasses import dataclass
from typing import Optional, Tuple


# ── Shutdown event ───────────────────────────────────────────────────────────
shutdown_event = threading.Event()
"""Cross-thread shutdown signal. All loops should check ``not shutdown_event.is_set()``."""


@dataclass
class WorldState:
    """Snapshot of the tracked objects in the arena."""
    
    # Our robot
    robot_pos: Optional[Tuple[int, int]] = None
    robot_heading: Optional[float] = None       # angle in radians
    robot_upright: bool = True                  # True if top tag is visible
    
    # Opponent
    opponent_pos: Optional[Tuple[int, int]] = None
    opponent_area: int = 0
    opponent_height: float = 0.0

    # System status
    auto_mode: bool = False
