"""
apriltag_tracker.py — AprilTag detection, ROI management, midpoint, and heading.

Handles Robot 1 tracking via two AprilTags with ROI-based fast detection
and full-frame fallback.
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Set, Tuple

import cv2
import numpy as np

from robot_control.config import Config

logger = logging.getLogger(__name__)

try:
    import pupil_apriltags as _at

    def _parse_det(det: Any) -> Tuple[np.ndarray, np.ndarray, int]:
        return det.corners.astype(np.float64), det.center, det.tag_id

except ImportError:
    raise SystemExit("Install pupil-apriltags:  pip install pupil-apriltags")


# ── Data structures ──────────────────────────────────────────────────────────

@dataclass
class TagInfo:
    """Structured representation of a detected AprilTag."""

    center: Tuple[float, float]
    corners_2d: np.ndarray
    corners_int: np.ndarray
    rvec: Optional[np.ndarray] = None
    tvec: Optional[np.ndarray] = None
    _pose_done: bool = False


@dataclass
class AprilTagState:
    """Mutable state for ROI-based tag tracking."""

    roi: Dict[int, Optional[Tuple[int, int, int, int]]] = field(
        default_factory=dict
    )
    roi_miss: Dict[int, int] = field(default_factory=dict)
    mid_source: str = ""
    tag_offsets: Dict[int, Tuple[float, float, Optional[float]]] = field(
        default_factory=dict
    )


# ── Detector factory ─────────────────────────────────────────────────────────

def make_detector(cfg: Config) -> Any:
    """Create a pupil_apriltags Detector from config."""
    return _at.Detector(
        families=cfg.tag_family,
        nthreads=cfg.detector_threads,
        quad_decimate=cfg.quad_decimate,
        quad_sigma=0.0,
        refine_edges=1,
        decode_sharpening=0.25,
    )


# ── Pose solving ─────────────────────────────────────────────────────────────

def solve_pose(info: TagInfo, cfg: Config) -> None:
    """Compute rvec/tvec for a tag if not already done."""
    if info._pose_done:
        return
    info._pose_done = True
    ok, rvec, tvec = cv2.solvePnP(
        cfg.tag_points_3d(),
        info.corners_2d,
        cfg.camera_matrix(),
        cfg.dist_coeffs(),
        flags=cv2.SOLVEPNP_IPPE_SQUARE,
    )
    if ok:
        info.rvec, info.tvec = rvec, tvec


def _tag_yaw(info: TagInfo, cfg: Config) -> Optional[float]:
    """Return the tag's yaw angle in radians."""
    solve_pose(info, cfg)
    if info.rvec is None:
        return None
    R, _ = cv2.Rodrigues(info.rvec)
    return math.atan2(R[1, 0], R[0, 0])


def _tag_forward_deg(info: TagInfo, cfg: Config) -> Optional[float]:
    """Return the tag's forward direction in degrees."""
    solve_pose(info, cfg)
    if info.rvec is None:
        return None
    R, _ = cv2.Rodrigues(info.rvec)
    fwd_x, fwd_y = R[0, 1], R[1, 1]
    if math.hypot(fwd_x, fwd_y) < 1e-6:
        return None
    return math.degrees(math.atan2(fwd_y, fwd_x))


# ── Detection helpers ────────────────────────────────────────────────────────

def _build_tag(
    corners: np.ndarray, center: np.ndarray, ox: int = 0, oy: int = 0
) -> TagInfo:
    """Build a TagInfo from raw detector output."""
    corners = corners + np.array([ox, oy])
    return TagInfo(
        center=(center[0] + ox, center[1] + oy),
        corners_2d=corners,
        corners_int=corners.astype(np.int32),
    )


def _detect_gray(detector: Any, gray: np.ndarray) -> List[Tuple[np.ndarray, np.ndarray, int]]:
    return [_parse_det(d) for d in detector.detect(gray)]


def _update_roi(
    state: AprilTagState,
    tid: int,
    corners_int: np.ndarray,
    fw: int,
    fh: int,
    padding: int,
) -> None:
    """Update the ROI cache for a given tag."""
    state.roi[tid] = (
        max(0, corners_int[:, 0].min() - padding),
        max(0, corners_int[:, 1].min() - padding),
        min(fw, corners_int[:, 0].max() + padding),
        min(fh, corners_int[:, 1].max() + padding),
    )
    state.roi_miss[tid] = 0


# ── Main detection ───────────────────────────────────────────────────────────

def detect_tags(
    frame: np.ndarray,
    detector: Any,
    state: AprilTagState,
    cfg: Config,
) -> Dict[int, TagInfo]:
    """
    Two-pass AprilTag detection:
    1. ROI crops around last known tag positions.
    2. Full-frame fallback when tags are missing.
    """
    fh, fw = frame.shape[:2]
    tags: Dict[int, TagInfo] = {}

    # Pass 1: ROI-based detection
    for tid, roi in list(state.roi.items()):
        if roi is None:
            continue
        x1, y1, x2, y2 = roi
        crop = frame[y1:y2, x1:x2]
        if crop.size == 0:
            state.roi_miss[tid] = state.roi_miss.get(tid, 0) + 1
            continue
        found = {
            fid: _build_tag(c, ctr, x1, y1)
            for c, ctr, fid in _detect_gray(
                detector, cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
            )
        }
        if tid in found:
            tags[tid] = found[tid]
            _update_roi(state, tid, tags[tid].corners_int, fw, fh, cfg.roi_padding)
        else:
            state.roi_miss[tid] = state.roi_miss.get(tid, 0) + 1
            if state.roi_miss[tid] >= cfg.roi_reacquire_frames:
                state.roi[tid] = None

    # Pass 2: Full-frame fallback
    r1_ids = cfg.robot1_tag_id_set
    need_full = not state.roi or (r1_ids and not r1_ids.issubset(tags))
    if need_full:
        gray_full = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        for c, ctr, tid in _detect_gray(detector, gray_full):
            if tid not in tags:
                tags[tid] = _build_tag(c, ctr)
                _update_roi(state, tid, tags[tid].corners_int, fw, fh, cfg.roi_padding)

    return tags


# ── Midpoint estimation ──────────────────────────────────────────────────────

def _rot2d(dx: float, dy: float, a: float) -> Tuple[float, float]:
    c, s = math.cos(a), math.sin(a)
    return c * dx - s * dy, s * dx + c * dy


def _learn_offset(
    state: AprilTagState,
    tid: int,
    info: TagInfo,
    mx: float,
    my: float,
    cfg: Config,
) -> None:
    cx, cy = info.center
    yaw = _tag_yaw(info, cfg)
    if yaw is None:
        state.tag_offsets[tid] = (mx - cx, my - cy, None)
    else:
        ldx, ldy = _rot2d(mx - cx, my - cy, -yaw)
        state.tag_offsets[tid] = (ldx, ldy, yaw)


def _apply_offset(
    state: AprilTagState, tid: int, info: TagInfo, cfg: Config
) -> Optional[Tuple[int, int]]:
    if tid not in state.tag_offsets:
        return None
    ldx, ldy, _ = state.tag_offsets[tid]
    cx, cy = info.center
    yaw = _tag_yaw(info, cfg)
    if yaw is None:
        return (int(cx + ldx), int(cy + ldy))
    wdx, wdy = _rot2d(ldx, ldy, yaw)
    return (int(cx + wdx), int(cy + wdy))


def robot1_midpoint(
    tags: Dict[int, TagInfo],
    state: AprilTagState,
    cfg: Config,
) -> Optional[Tuple[int, int]]:
    """Compute Robot 1's midpoint from visible tags."""
    r1_ids = [
        i
        for i in (cfg.robot1_tag_ids if cfg.robot1_tag_ids else tags.keys())
        if i in tags and i not in (cfg.target_tag_a, cfg.target_tag_b)
    ]
    if len(r1_ids) >= 2:
        a, b = tags[r1_ids[0]].center, tags[r1_ids[1]].center
        mx, my = (a[0] + b[0]) / 2, (a[1] + b[1]) / 2
        _learn_offset(state, r1_ids[0], tags[r1_ids[0]], mx, my, cfg)
        _learn_offset(state, r1_ids[1], tags[r1_ids[1]], mx, my, cfg)
        state.mid_source = "both"
        return (int(mx), int(my))
    if len(r1_ids) == 1:
        pt = _apply_offset(state, r1_ids[0], tags[r1_ids[0]], cfg)
        state.mid_source = (
            f"tag {r1_ids[0]} +offset" if pt else f"tag {r1_ids[0]} no offset"
        )
        return pt
    state.mid_source = ""
    return None


# ── Heading estimation ────────────────────────────────────────────────────────

def estimate_heading(
    tags: Dict[int, TagInfo],
    kf: Any,
    cfg: Config,
) -> float:
    """Estimate Robot 1's heading from tag poses or Kalman velocity."""
    r1_tag_ids = [
        i
        for i in (cfg.robot1_tag_ids if cfg.robot1_tag_ids else tags.keys())
        if i in tags and i not in (cfg.target_tag_a, cfg.target_tag_b)
    ]

    fwd_vectors: List[Tuple[float, float]] = []
    for tid in r1_tag_ids:
        deg = _tag_forward_deg(tags[tid], cfg)
        if deg is not None:
            fwd_vectors.append(
                (math.cos(math.radians(deg)), math.sin(math.radians(deg)))
            )
    if fwd_vectors:
        mx = sum(v[0] for v in fwd_vectors) / len(fwd_vectors)
        my = sum(v[1] for v in fwd_vectors) / len(fwd_vectors)
        return math.degrees(math.atan2(my, mx))

    vx, vy = kf.velocity()
    if math.hypot(vx, vy) > 0.5:
        return math.degrees(math.atan2(vy, vx))
    return 0.0
