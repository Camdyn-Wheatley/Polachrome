"""
kinect_viewer.py — Diagnostic viewer for Kinect V2 streams with ArUco detection.

Displays four windows showing the raw sensor data from a connected Kinect V2:

  - **Color**:       1920×1080 colour image
  - **Depth**:       512×424 depth map (colourised with TURBO palette)
  - **IR**:          512×424 infrared image — ArUco boxes overlaid when detect_on_ir=True
  - **Registered**:  512×424 colour image warped into depth-camera coordinates.
                     Every pixel here has a matching depth value from the Depth
                     window (same resolution, same view). ArUco boxes overlaid
                     when detect_on_ir=True (shares coordinate space with IR).

ArUco detection source (set by config detect_on_ir):
  - detect_on_ir=False  → detect on the 1920×1080 Color frame; boxes on Color window.
  - detect_on_ir=True   → detect on the 512×424 IR frame; boxes on IR + Registered.

What is the Registered frame?
  The colour (RGB) and depth/IR cameras are physically offset by ~25 mm on the
  Kinect body, so the same point in the scene appears at slightly different pixel
  positions in each sensor. "Registration" uses the Kinect's factory-calibrated
  intrinsics to re-project the 1920×1080 colour pixels so they align with the
  512×424 depth pixels — every (x, y) in the Registered image corresponds to the
  same (x, y) in the Depth image. This lets you ask "what colour is the object
  at distance D mm?" without any extra coordinate math. The downside is that the
  colour is now at the lower depth resolution (512×424) and there are small border
  regions where one camera sees something the other cannot.

Run as::

    python -m robot_control.vision.kinect_viewer [--pipeline opengl]

Controls:
    Q   quit
"""

from __future__ import annotations

import argparse
import logging
import signal
import sys
import time
from typing import Any, List, Optional, Tuple

import cv2
import numpy as np

from robot_control.config import load_config
from robot_control.vision.kinect_stream import KinectStream
from robot_control.vision.segmentation import DepthSegmenter

logger = logging.getLogger(__name__)

_running = True


def _signal_handler(sig: int, _frame: object) -> None:
    global _running
    logger.warning("Signal %s — shutting down viewer...", signal.Signals(sig).name)
    _running = False

_tracked_centroid: Optional[Tuple[int, int]] = None
_clicked_point: Optional[Tuple[int, int]] = None
_calibration_points: List[Tuple[int, int]] = []
_excluded_points: List[Tuple[int, int]] = []

def _on_mouse_click(event: int, x: int, y: int, flags: int, param: Any) -> None:
    global _clicked_point, _calibration_points, _excluded_points
    if event == cv2.EVENT_LBUTTONDOWN:
        if len(_calibration_points) < 4:
            _calibration_points.append((x, y))
        else:
            _clicked_point = (x, y)
    elif event == cv2.EVENT_MBUTTONDOWN:
        _excluded_points.append((x, y))


# ── Image helpers ────────────────────────────────────────────────────────────

def _depth_to_colormap(depth: np.ndarray, max_depth_mm: float = 4500.0) -> np.ndarray:
    """Convert a float32 depth frame (mm) to a colourised BGR image for display."""
    valid = depth > 0
    normalised = np.zeros_like(depth, dtype=np.uint8)
    normalised[valid] = np.clip(depth[valid] / max_depth_mm * 255, 0, 255).astype(np.uint8)
    coloured = cv2.applyColorMap(normalised, cv2.COLORMAP_TURBO)
    coloured[~valid] = 0
    return coloured


def _ir_to_uint8(ir: np.ndarray) -> np.ndarray:
    """Normalise float32 IR to uint8 grey. Clamps at 10 000 (typical indoor range)."""
    clamped = np.clip(ir / 10000.0, 0.0, 1.0)
    return (clamped * 255).astype(np.uint8)


def _ir_to_display(ir: np.ndarray) -> np.ndarray:
    """Return a BGR version of the IR frame suitable for display."""
    return cv2.cvtColor(_ir_to_uint8(ir), cv2.COLOR_GRAY2BGR)


# ── ArUco detection ──────────────────────────────────────────────────────────

def _build_detector(dict_name: str) -> cv2.aruco.ArucoDetector:
    """Create a tuned ArUco detector for the named dictionary.

    Parameter choices:
    - minMarkerPerimeterRate=0.01  default 0.03 — allows smaller tags in the frame
    - cornerRefinementMethod=CORNER_REFINE_SUBPIX — improves accuracy under blur/angle
    """
    dict_id = getattr(cv2.aruco, dict_name, None)
    if dict_id is None:
        raise ValueError(
            f"Unknown ArUco dictionary: {dict_name!r}. "
            f"Available examples: DICT_4X4_50, DICT_APRILTAG_36H11, DICT_6X6_250"
        )
    aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)
    params = cv2.aruco.DetectorParameters()
    # Allow tags that appear smaller in the frame (e.g. at distance or wide-angle view).
    params.minMarkerPerimeterRate = 0.01
    # Subpixel corner refinement — helps with motion blur and perspective skew.
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    return cv2.aruco.ArucoDetector(aruco_dict, params)


def _detect_aruco(
    detector: cv2.aruco.ArucoDetector,
    grey: np.ndarray,
) -> Tuple[List, Optional[np.ndarray], List]:
    """Run ArUco detection on a uint8 greyscale image.

    Returns (corners, ids, rejected) in the same form as detectMarkers.
    ids is a flat list of ints (or empty list if none found).
    """
    corners, ids, rejected = detector.detectMarkers(grey)
    flat_ids: List[int] = ids.flatten().tolist() if ids is not None else []
    return corners, flat_ids, rejected


def _draw_detections(
    image: np.ndarray,
    corners: List,
    ids: List[int],
    tag_top: int,
    tag_bottom: int,
) -> np.ndarray:
    """Draw ArUco detection boxes and labels onto *image* (in-place, returns image).

    Colour coding:
      - Green  = top tag (robot right-side up)
      - Blue   = bottom tag (robot upside down)
      - Yellow = any other detected tag
    """
    if not corners:
        return image

    for corner, tag_id in zip(corners, ids):
        pts = corner[0].astype(int)  # shape (4, 2)

        # Pick colour by tag role.
        if tag_id == tag_top:
            colour = (0, 255, 0)      # green
            label = f"TOP id={tag_id}"
        elif tag_id == tag_bottom:
            colour = (255, 80, 0)     # blue
            label = f"BOT id={tag_id}"
        else:
            colour = (0, 220, 255)    # yellow
            label = f"id={tag_id}"

        # Draw quadrilateral outline.
        cv2.polylines(image, [pts], isClosed=True, color=colour, thickness=2)

        # Draw corner dots.
        for pt in pts:
            cv2.circle(image, tuple(pt), 4, colour, -1)

        # Label above the top-left corner.
        cx = int(pts[:, 0].mean())
        cy = int(pts[:, 1].min()) - 8
        cy = max(cy, 15)
        cv2.putText(image, label, (cx - 30, cy),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, colour, 2)

    return image


# ── Main ─────────────────────────────────────────────────────────────────────

def main() -> None:
    global _running

    parser = argparse.ArgumentParser(description="Kinect V2 diagnostic viewer")
    parser.add_argument(
        "--pipeline",
        type=str,
        default="opengl",
        choices=["opengl", "opencl", "cpu"],
        help="Kinect processing pipeline backend (default: opengl)",
    )
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(name)s] %(levelname)s: %(message)s",
        datefmt="%H:%M:%S",
    )

    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    cfg = load_config()
    detector = _build_detector(cfg.aruco_dict)
    logger.info(
        "ArUco detector ready: dict=%s  top_tag=%d  bottom_tag=%d  detect_on_ir=%s",
        cfg.aruco_dict, cfg.robot_tag_top, cfg.robot_tag_bottom, cfg.detect_on_ir,
    )

    logger.info("Starting Kinect V2 viewer (pipeline=%s)...", args.pipeline)

    stream = KinectStream(
        pipeline=args.pipeline,
        enable_color=True,
        enable_depth=True,
        enable_registered=True,
        flip_horizontal=cfg.flip_horizontal,
    )

    segmenter = DepthSegmenter(
        max_depth_mm=cfg.arena_max_depth_mm,
        obstacle_min_height_mm=cfg.obstacle_min_height_mm,
        obstacle_min_area_px=cfg.obstacle_min_area_px,
    )
    calibration_frames: List[np.ndarray] = []

    try:
        stream.start()
    except RuntimeError as exc:
        logger.error("Failed to start Kinect: %s", exc)
        sys.exit(1)

    cv2.namedWindow("Color", cv2.WINDOW_NORMAL)
    cv2.namedWindow("Depth", cv2.WINDOW_NORMAL)
    cv2.namedWindow("IR + ArUco", cv2.WINDOW_NORMAL)
    cv2.namedWindow("Registered + ArUco", cv2.WINDOW_NORMAL)

    cv2.resizeWindow("Color", 960, 540)
    cv2.resizeWindow("Depth", 512, 424)
    cv2.resizeWindow("IR + ArUco", 512, 424)
    cv2.resizeWindow("Registered + ArUco", 512, 424)

    cv2.setMouseCallback("Depth", _on_mouse_click)
    cv2.setMouseCallback("Registered + ArUco", _on_mouse_click)

    fps_time = time.time()
    frame_count = 0
    fps = 0.0

    try:
        while _running:
            color = stream.read_color()
            depth = stream.read_depth()
            ir = stream.read_ir()
            registered = stream.read_registered()

            if color is None and depth is None:
                time.sleep(0.01)
                continue

            # ── Depth Calibration ─────────────────────────────────────────
            global _calibration_points
            if depth is not None and not segmenter.is_calibrated:
                if len(_calibration_points) == 4:
                    calibration_frames.append(depth)
                    if len(calibration_frames) >= 10:
                        logger.info("4 points selected! Calibrating ground plane...")
                        
                        tag_corners = np.array(_calibration_points, dtype=np.int32)
                        mask = np.zeros(depth.shape, dtype=np.uint8)
                        cv2.fillConvexPoly(mask, tag_corners, 255)
                        
                        if not segmenter.calibrate(calibration_frames, mask=mask):
                            logger.warning("Calibration failed! Retrying...")
                            calibration_frames.clear()
                            _calibration_points.clear()
                else:
                    calibration_frames.clear()
                
                # We skip object tracking while calibrating
                obstacles = []
            else:
                obstacles = []
                if depth is not None and segmenter.is_calibrated:
                    raw_obstacles = segmenter.find_obstacles(depth)
                    global _excluded_points
                    for obs in raw_obstacles:
                        ox, oy, ow, oh = obs["bbox"]
                        excluded = False
                        for ex, ey in _excluded_points:
                            if ox <= ex <= ox + ow and oy <= ey <= oy + oh:
                                excluded = True
                                break
                        if not excluded:
                            obstacles.append(obs)

            # ── Obstacle Selection & Tracking ─────────────────────────────
            global _clicked_point, _tracked_centroid
            if _clicked_point is not None:
                cx, cy = _clicked_point
                _clicked_point = None
                best_obs = None
                for obs in obstacles:
                    ox, oy, ow, oh = obs["bbox"]
                    if ox <= cx <= ox + ow and oy <= cy <= oy + oh:
                        best_obs = obs
                        break
                if best_obs:
                    _tracked_centroid = best_obs["centroid"]
                    logger.info("Tracking obstacle at %s", _tracked_centroid)
                else:
                    _tracked_centroid = None
                    logger.info("Cleared tracked obstacle")

            tracked_obs = None
            if _tracked_centroid is not None and obstacles:
                closest = min(obstacles, key=lambda o: (o["centroid"][0] - _tracked_centroid[0])**2 + (o["centroid"][1] - _tracked_centroid[1])**2)
                dist_sq = (closest["centroid"][0] - _tracked_centroid[0])**2 + (closest["centroid"][1] - _tracked_centroid[1])**2
                if dist_sq < 10000:  # within 100px
                    tracked_obs = closest
                    _tracked_centroid = closest["centroid"]
                else:
                    _tracked_centroid = None
                    logger.info("Lost track of obstacle")

            # FPS counter.
            frame_count += 1
            now = time.time()
            elapsed = now - fps_time
            if elapsed >= 1.0:
                fps = frame_count / elapsed
                frame_count = 0
                fps_time = now

            # ── ArUco detection ───────────────────────────────────────────
            # detect_on_ir=False → detect on the 1920×1080 colour frame.
            # detect_on_ir=True  → detect on the 512×424 IR frame.
            corners: List = []
            ids: List[int] = []
            if cfg.detect_on_ir:
                if ir is not None:
                    grey = _ir_to_uint8(ir)
                    corners, ids, _ = _detect_aruco(detector, grey)
            else:
                if color is not None:
                    grey_color = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)
                    corners, ids, _ = _detect_aruco(detector, grey_color)
                else:
                    logger.warning("Color frame is None — cannot run detection")

            # Periodic INFO-level status to the terminal (every 2 s).
            if elapsed >= 1.0:  # reuses the FPS elapsed window
                if ids:
                    logger.info(
                        "DETECTED %d tag(s): ids=%s  dict=%s",
                        len(ids), ids, cfg.aruco_dict,
                    )
                else:
                    src = "IR" if cfg.detect_on_ir else "Color"
                    logger.info(
                        "No tags detected  src=%s  dict=%s  color_avail=%s  ir_avail=%s",
                        src, cfg.aruco_dict,
                        color is not None, ir is not None,
                    )

            # ── Color window ──────────────────────────────────────────────
            if color is not None:
                disp_color = color.copy()
                # Draw ArUco boxes on the colour window when detecting on colour.
                if not cfg.detect_on_ir:
                    _draw_detections(
                        disp_color, corners, ids,
                        cfg.robot_tag_top, cfg.robot_tag_bottom,
                    )
                tag_count = len(ids)
                status = f"{tag_count} tag(s)" if tag_count else "no tags"
                src_label = "IR" if cfg.detect_on_ir else "Color"
                cv2.putText(
                    disp_color,
                    f"Color 1920x1080  [{src_label} detection] {status}  FPS: {fps:.1f}",
                    (20, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 2,
                )
                cv2.imshow("Color", disp_color)

            # ── Depth window ──────────────────────────────────────────────
            if depth is not None:
                disp_depth = _depth_to_colormap(depth)
                if segmenter.is_calibrated:
                    mask = segmenter.get_ground_mask(depth)
                    disp_depth[mask] = (disp_depth[mask].astype(np.float32) * 0.5 + np.array([0, 200, 0]) * 0.5).astype(np.uint8)
                    for obs in obstacles:
                        x, y, bw, bh = obs["bbox"]
                        color = (0, 255, 0) if obs == tracked_obs else (0, 0, 255)
                        cv2.rectangle(disp_depth, (x, y), (x + bw, y + bh), color, 2)
                        cv2.putText(disp_depth, f"H={obs['mean_height_mm']:.0f}mm", (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

                cv2.putText(
                    disp_depth,
                    f"Depth 512x424  FPS: {fps:.1f}",
                    (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1,
                )
                cv2.imshow("Depth", disp_depth)

            # ── IR + ArUco window ─────────────────────────────────────────
            # Boxes drawn here only when detecting on IR (same coordinate space).
            if ir is not None:
                disp_ir = _ir_to_display(ir)
                if cfg.detect_on_ir:
                    _draw_detections(
                        disp_ir, corners, ids,
                        cfg.robot_tag_top, cfg.robot_tag_bottom,
                    )
                tag_count = len(ids) if cfg.detect_on_ir else 0
                status = f"{tag_count} tag(s)" if (cfg.detect_on_ir and tag_count) else (
                    "detection on Color" if not cfg.detect_on_ir else "no tags"
                )
                cv2.putText(
                    disp_ir,
                    f"IR 512x424  {status}  FPS: {fps:.1f}",
                    (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1,
                )
                cv2.imshow("IR + ArUco", disp_ir)

            # ── Registered + ArUco window ─────────────────────────────────
            # IR and Registered share the same 512×424 coordinate space, so
            # boxes drawn here only when detecting on IR.
            if registered is not None:
                disp_reg = registered.copy()
                if cfg.detect_on_ir:
                    _draw_detections(
                        disp_reg, corners, ids,
                        cfg.robot_tag_top, cfg.robot_tag_bottom,
                    )
                if segmenter.is_calibrated:
                    for obs in obstacles:
                        x, y, bw, bh = obs["bbox"]
                        color = (0, 255, 0) if obs == tracked_obs else (0, 0, 255)
                        cv2.rectangle(disp_reg, (x, y), (x + bw, y + bh), color, 2)
                        
                    for ex, ey in _excluded_points:
                        cv2.drawMarker(disp_reg, (ex, ey), (0, 0, 255), cv2.MARKER_CROSS, 10, 2)
                        
                    cv2.putText(
                        disp_reg,
                        f"Registered 512x424  FPS: {fps:.1f}",
                        (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1,
                    )
                else:
                    # Draw calibration UI
                    cv2.putText(disp_reg, f"Click 4 corners on the floor: {len(_calibration_points)}/4", (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 165, 255), 2)
                    for pt in _calibration_points:
                        cv2.circle(disp_reg, pt, 5, (0, 255, 0), -1)
                    if len(_calibration_points) > 1:
                        for i in range(len(_calibration_points) - 1):
                            cv2.line(disp_reg, _calibration_points[i], _calibration_points[i+1], (0, 255, 0), 2)
                    if len(_calibration_points) == 4:
                        cv2.line(disp_reg, _calibration_points[-1], _calibration_points[0], (0, 255, 0), 2)
                        
                cv2.imshow("Registered + ArUco", disp_reg)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                _running = False

    finally:
        stream.stop()
        cv2.destroyAllWindows()
        logger.info("Kinect viewer shut down.")


if __name__ == "__main__":
    main()
