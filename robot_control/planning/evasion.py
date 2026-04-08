"""
evasion.py — Threat detection and evasion waypoint computation.

Detects when Robot 2 is charging at Robot 1 and computes a
perpendicular dodge point.
"""

from __future__ import annotations

import math
from typing import Optional, Tuple

from robot_control.arena import ArenaBounds
from robot_control.config import Config


def evasion_offset(
    r1: Tuple[float, float],
    r2: Tuple[float, float],
    r2_vel: Tuple[float, float],
    bounds: ArenaBounds,
    cfg: Config,
) -> Optional[Tuple[float, float]]:
    """
    Return a perpendicular dodge point if R2 is heading toward R1.

    Triggers when R2's speed > 1.5 px/frame AND velocity-to-R1 angle
    is within the threat cone.
    """
    vx, vy = r2_vel
    speed = math.hypot(vx, vy)
    if speed < 1.5:
        return None

    dx, dy = r1[0] - r2[0], r1[1] - r2[1]
    angle_to_r1 = math.degrees(math.atan2(dy, dx))
    vel_angle = math.degrees(math.atan2(vy, vx))
    diff = abs((angle_to_r1 - vel_angle + 360) % 360)
    if diff > 180:
        diff = 360 - diff
    if diff > cfg.threat_cone_deg:
        return None

    perp_angle = math.radians(vel_angle + 90)
    return bounds.clamp(
        (
            r1[0] + math.cos(perp_angle) * cfg.evasion_offset,
            r1[1] + math.sin(perp_angle) * cfg.evasion_offset,
        )
    )
