"""
drawing.py — OpenCV drawing utilities for path, axis, and robot markers.
"""

from __future__ import annotations

import math
from typing import Optional, Tuple

import cv2
import numpy as np

from robot_control.config import Config


def draw_path(
    frame: np.ndarray,
    path: Optional[np.ndarray],
    r2: Optional[Tuple[int, int]],
    target: Optional[Tuple[int, int]],
    target_id: int,
    cfg: Config,
) -> None:
    """Draw the planned path, knock-through axis, and target marker."""
    if path is None or len(path) < 2:
        return

    pts = path.astype(np.int32)
    for i in range(len(pts) - 1):
        frac = i / max(len(pts) - 2, 1)
        color = (0, int(200 * (1 - frac) + 100 * frac), int(255 * frac))
        cv2.line(frame, tuple(pts[i]), tuple(pts[i + 1]), color, 3)

    if len(pts) >= 4:
        cv2.arrowedLine(
            frame, tuple(pts[-4]), tuple(pts[-1]), (0, 180, 255), 2, tipLength=0.4
        )

    if r2 and target:
        r2_f = np.array(r2, dtype=float)
        tgt_f = np.array(target, dtype=float)
        push = tgt_f - r2_f
        pl = np.linalg.norm(push)
        if pl > 1e-3:
            unit = push / pl
            behind = r2_f - unit * cfg.approach_standoff * 1.2
            _draw_dashed_line(
                frame,
                (int(behind[0]), int(behind[1])),
                (int(tgt_f[0]), int(tgt_f[1])),
                (0, 0, 255),
                thickness=2,
                dash=14,
            )
            cv2.arrowedLine(
                frame,
                (int(r2_f[0]), int(r2_f[1])),
                (int(tgt_f[0]), int(tgt_f[1])),
                (0, 0, 255),
                2,
                tipLength=0.18,
            )

        cv2.circle(frame, (int(r2[0]), int(r2[1])), 12, (0, 100, 255), 2)
        cv2.putText(
            frame,
            "hit here",
            (int(r2[0]) + 14, int(r2[1]) - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (0, 100, 255),
            1,
        )

    if target:
        cv2.drawMarker(
            frame,
            (int(target[0]), int(target[1])),
            (0, 0, 255),
            cv2.MARKER_STAR,
            24,
            2,
        )
        cv2.putText(
            frame,
            f"TAG {target_id} (target)",
            (int(target[0]) + 14, int(target[1]) - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 0, 255),
            2,
        )


def draw_lookahead(
    frame: np.ndarray,
    r1: Optional[Tuple[int, int]],
    path: Optional[np.ndarray],
    lookahead: int = 80,
) -> None:
    """Draw the lookahead point on the path."""
    if r1 is None or path is None:
        return
    r1_f = np.array(r1, dtype=float)
    for pt in path:
        if np.linalg.norm(pt - r1_f) >= lookahead:
            cv2.circle(frame, (int(pt[0]), int(pt[1])), 7, (255, 0, 255), 2)
            cv2.line(frame, r1, (int(pt[0]), int(pt[1])), (255, 0, 255), 1)
            break


def draw_heading_arrow(
    frame: np.ndarray,
    r1: Optional[Tuple[int, int]],
    heading_deg: float,
    length: int = 60,
) -> None:
    """Draw R1's heading direction arrow."""
    if r1 is None:
        return
    h_rad = math.radians(heading_deg)
    tip = (
        int(r1[0] + math.cos(h_rad) * length),
        int(r1[1] + math.sin(h_rad) * length),
    )
    cv2.arrowedLine(frame, r1, tip, (255, 255, 255), 2, tipLength=0.3)


def draw_threat_indicator(
    frame: np.ndarray,
    r2: Optional[Tuple[int, int]],
    evade_pt: Optional[Tuple[float, float]],
) -> None:
    """Draw EVADE warning and arrow when threat evasion is active."""
    if evade_pt is None:
        return
    cv2.putText(
        frame, "EVADE", (20, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2
    )
    if r2:
        cv2.arrowedLine(
            frame,
            (int(r2[0]), int(r2[1])),
            (int(evade_pt[0]), int(evade_pt[1])),
            (0, 0, 255),
            2,
            tipLength=0.2,
        )


def draw_tags(
    frame: np.ndarray,
    tags: dict,
    target_tag_a: int,
    target_tag_b: int,
) -> None:
    """Draw detected tag outlines, centres, and ID labels."""
    for tid_draw, info in tags.items():
        ci = info.corners_int
        cx_d = int(info.center[0])
        cy_d = int(info.center[1])
        c_tag = (
            (0, 255, 0)
            if tid_draw not in (target_tag_a, target_tag_b)
            else (0, 80, 255)
        )
        cv2.polylines(frame, [ci.reshape(-1, 1, 2)], True, c_tag, 2)
        cv2.circle(frame, (cx_d, cy_d), 5, (0, 200, 255), -1)
        cv2.putText(
            frame,
            f"T{tid_draw}",
            (cx_d + 8, cy_d - 8),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 255),
            2,
        )


def draw_robot_markers(
    frame: np.ndarray,
    r1_pos: Optional[Tuple[int, int]],
    r1_src: str,
    r2_pos: Optional[Tuple[int, int]],
    r2_src: str,
    kf1_vel: Tuple[float, float],
    r2_vel: Tuple[float, float],
    heading_deg: float,
) -> None:
    """Draw R1 and R2 position markers, velocity arrows, and heading."""
    if r1_pos:
        col = (255, 200, 0) if r1_src != "coast" else (100, 100, 50)
        cv2.drawMarker(frame, r1_pos, col, cv2.MARKER_CROSS, 26, 3)
        cv2.putText(
            frame,
            f"R1 [{r1_src}]",
            (r1_pos[0] + 14, r1_pos[1] - 8),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            col,
            2,
        )
        vx1, vy1 = kf1_vel
        if math.hypot(vx1, vy1) > 0.5:
            cv2.arrowedLine(
                frame,
                r1_pos,
                (int(r1_pos[0] + vx1 * 3), int(r1_pos[1] + vy1 * 3)),
                col,
                2,
                tipLength=0.3,
            )
        draw_heading_arrow(frame, r1_pos, heading_deg, length=55)

    if r2_pos:
        cv2.circle(frame, r2_pos, 8, (0, 140, 255), -1)
        cv2.putText(
            frame,
            f"R2 [{r2_src}]",
            (r2_pos[0] + 12, r2_pos[1] - 8),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (0, 140, 255),
            2,
        )
        vx2, vy2 = r2_vel
        if math.hypot(vx2, vy2) > 0.5:
            cv2.arrowedLine(
                frame,
                r2_pos,
                (int(r2_pos[0] + vx2 * 3), int(r2_pos[1] + vy2 * 3)),
                (0, 0, 255),
                2,
                tipLength=0.3,
            )


def _draw_dashed_line(
    frame: np.ndarray,
    p0: Tuple[int, int],
    p1: Tuple[int, int],
    color: Tuple[int, int, int],
    thickness: int = 1,
    dash: int = 12,
) -> None:
    """Draw a dashed line between two points."""
    dx, dy = p1[0] - p0[0], p1[1] - p0[1]
    length = math.hypot(dx, dy)
    if length < 1:
        return
    steps = int(length / (dash * 2))
    for s in range(steps + 1):
        t0 = min((2 * s) * dash / length, 1.0)
        t1 = min((2 * s + 1) * dash / length, 1.0)
        cv2.line(
            frame,
            (int(p0[0] + dx * t0), int(p0[1] + dy * t0)),
            (int(p0[0] + dx * t1), int(p0[1] + dy * t1)),
            color,
            thickness,
        )
