"""
path_planner.py — Bézier path planning for the knock-through maneuver.

Generates smooth approach trajectories from Robot 1 to Robot 2,
aligned so that R2 is knocked toward the target tag.
"""

from __future__ import annotations

import math
from typing import Tuple

import numpy as np

from robot_control.arena import ArenaBounds
from robot_control.config import Config


def _cubic_bezier(
    p0: np.ndarray,
    p1: np.ndarray,
    p2: np.ndarray,
    p3: np.ndarray,
    n: int = 60,
) -> np.ndarray:
    """Evaluate a cubic Bézier curve at ``n`` evenly spaced parameter values."""
    t = np.linspace(0, 1, n).reshape(-1, 1)
    mt = 1 - t
    return (
        mt**3 * np.array(p0)
        + 3 * mt**2 * t * np.array(p1)
        + 3 * mt * t**2 * np.array(p2)
        + t**3 * np.array(p3)
    )


def build_path(
    r1: Tuple[float, float],
    r2: Tuple[float, float],
    target: Tuple[float, float],
    bounds: ArenaBounds,
    cfg: Config,
    r1_heading_deg: float = 0.0,
) -> np.ndarray:
    """
    Build a knock-through path from R1 to R2 via an entry point.

    Geometry:
      push_unit  = normalise(target − R2)
      entry_pt   = R2 − push_unit × STANDOFF
      Bézier departs R1 along its heading, arrives at entry_pt
      aimed along −push_unit (straight at R2).
      A short straight segment carries R1 through R2.
    """
    r1_f = np.array(r1, dtype=float)
    r2_f = np.array(r2, dtype=float)
    tgt_f = np.array(target, dtype=float)

    push_vec = tgt_f - r2_f
    push_len = np.linalg.norm(push_vec)
    push_unit = push_vec / push_len if push_len > 1e-3 else np.array([1.0, 0.0])

    entry_pt = r2_f - push_unit * cfg.approach_standoff

    r1_f = np.array(bounds.clamp(tuple(r1_f)))
    entry_pt = np.array(bounds.clamp(tuple(entry_pt)))
    r2_clamped = np.array(bounds.clamp(tuple(r2_f)))

    h_rad = math.radians(r1_heading_deg)
    fwd_unit = np.array([math.cos(h_rad), math.sin(h_rad)])

    chord = entry_pt - r1_f
    length = np.linalg.norm(chord)

    if length > 1e-3:
        c1 = r1_f + fwd_unit * min(length * 0.45, 220.0)
        c2 = entry_pt - push_unit * min(length * 0.35, 180.0)
        c1 = np.array(bounds.clamp(tuple(c1)))
        c2 = np.array(bounds.clamp(tuple(c2)))
    else:
        c1 = r1_f.copy()
        c2 = entry_pt.copy()

    curve = _cubic_bezier(r1_f, c1, c2, entry_pt, n=55)
    straight = np.linspace(entry_pt, r2_clamped, 10)[1:]
    return np.vstack([curve, straight])


def build_evasion_path(
    r1: Tuple[float, float],
    evade_pt: Tuple[float, float],
    bounds: ArenaBounds,
    r1_heading_deg: float = 0.0,
) -> np.ndarray:
    """Simple single-segment Bézier from R1 to the evasion waypoint."""
    r1_f = np.array(r1, dtype=float)
    tgt_f = np.array(evade_pt, dtype=float)

    r1_f = np.array(bounds.clamp(tuple(r1_f)))
    tgt_f = np.array(bounds.clamp(tuple(tgt_f)))

    chord = tgt_f - r1_f
    length = np.linalg.norm(chord)

    h_rad = math.radians(r1_heading_deg)
    fwd_unit = np.array([math.cos(h_rad), math.sin(h_rad)])

    if length > 1e-3:
        c1 = r1_f + fwd_unit * min(length * 0.4, 160.0)
        c2 = tgt_f - (chord / length) * min(length * 0.2, 80.0)
        c1 = np.array(bounds.clamp(tuple(c1)))
        c2 = np.array(bounds.clamp(tuple(c2)))
    else:
        c1 = r1_f.copy()
        c2 = tgt_f.copy()

    return _cubic_bezier(r1_f, c1, c2, tgt_f, n=30)
