"""
hud.py — Heads-up display overlay for the main tracker window.
"""

from __future__ import annotations

from typing import Tuple

import cv2
import numpy as np


def draw_auto_hud(
    frame: np.ndarray,
    auto_mode: bool,
    target_id: int,
    ch1: int,
    ch2: int,
    ch3: int,
    fps: float,
) -> None:
    """Render the mode / target / channel / FPS overlay."""
    fh, fw = frame.shape[:2]
    mode_str = "AUTO" if auto_mode else "MANUAL"
    mode_col = (0, 255, 80) if auto_mode else (0, 140, 255)

    cv2.rectangle(frame, (fw - 200, 0), (fw, 90), (30, 30, 30), -1)
    cv2.putText(
        frame, mode_str, (fw - 190, 28),
        cv2.FONT_HERSHEY_SIMPLEX, 0.8, mode_col, 2,
    )
    cv2.putText(
        frame, f"TGT: TAG {target_id}", (fw - 190, 54),
        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1,
    )
    cv2.putText(
        frame, f"CH1:{ch1} CH2:{ch2} CH3:{ch3}", (fw - 190, 76),
        cv2.FONT_HERSHEY_SIMPLEX, 0.38, (160, 160, 160), 1,
    )
    cv2.putText(
        frame, f"FPS:{int(fps)}", (10, 28),
        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2,
    )
    cv2.putText(
        frame, "A=auto  T=target  B=bounds  +/-=offset  Q=quit",
        (10, fh - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (160, 160, 160), 1,
    )
