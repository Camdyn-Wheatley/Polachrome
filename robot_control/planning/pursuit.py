"""
pursuit.py — Pure-pursuit path-following controller.

Converts a planned path and current heading into drive/steer PWM values.
"""

from __future__ import annotations

import math
from typing import Optional, Tuple

import numpy as np

from robot_control.config import Config


def pure_pursuit(
    r1: Tuple[float, float],
    heading_deg: float,
    path: Optional[np.ndarray],
    cfg: Config,
) -> Tuple[int, int]:
    """
    Pure-pursuit controller.

    Returns ``(drive_pwm, steer_pwm)`` each in [1000, 2000].
    """
    if path is None or len(path) < 2:
        return cfg.pwm_neutral, cfg.pwm_neutral

    r1_f = np.array(r1, dtype=float)

    lookahead_pt = None
    for pt in path:
        if np.linalg.norm(pt - r1_f) >= cfg.lookahead_dist:
            lookahead_pt = pt
            break
    if lookahead_pt is None:
        lookahead_pt = path[-1]

    diff = lookahead_pt - r1_f
    dist = np.linalg.norm(diff)
    if dist < 5:
        return cfg.pwm_neutral, cfg.pwm_neutral

    target_angle = math.degrees(math.atan2(diff[1], diff[0]))
    error_angle = (target_angle - heading_deg + 360) % 360
    if error_angle > 180:
        error_angle -= 360

    turn = int(error_angle / 180.0 * cfg.max_turn_speed)
    steer = cfg.pwm_neutral + max(
        -cfg.max_turn_speed, min(cfg.max_turn_speed, turn)
    )

    speed_factor = max(0.2, 1.0 - abs(error_angle) / 120.0)
    drive = cfg.pwm_neutral + max(
        -cfg.max_drive_speed,
        min(cfg.max_drive_speed, int(cfg.max_drive_speed * speed_factor)),
    )
    return drive, steer
