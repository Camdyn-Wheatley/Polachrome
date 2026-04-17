"""
__main__.py — Entry point for the autonomous combat robot controller.

Connects the Kinect V2 vision pipeline, the pure pursuit path planner,
and the Arduino serial output into a single high-performance control loop.

Controls:
    SPACE   Toggle Autonomous Mode
    Click   Lock onto an opponent obstacle in the Registered/Depth window
    Q       Quit
"""

from __future__ import annotations

import argparse
import logging
import math
import signal
import sys
import time
from typing import Any, List, Optional, Tuple

import cv2
import numpy as np

from robot_control.config import load_config
from robot_control.serial_output import ArduinoSerial
from robot_control.state import WorldState, shutdown_event
from robot_control.vision.kinect_stream import KinectStream
from robot_control.vision.segmentation import DepthSegmenter
from robot_control.planning.planner import Planner

logger = logging.getLogger(__name__)

# Global state for UI interaction
world_state = WorldState()
_tracked_centroid: Optional[Tuple[int, int]] = None
_tracked_height_mm: Optional[float] = None
_clicked_point: Optional[Tuple[int, int]] = None


def _signal_handler(sig: int, _frame: object) -> None:
    logger.warning("Signal %s — shutting down...", signal.Signals(sig).name)
    shutdown_event.set()


def _on_mouse_click(event: int, x: int, y: int, flags: int, param: Any) -> None:
    global _clicked_point, _tracked_centroid, _tracked_height_mm, world_state
    if event == cv2.EVENT_LBUTTONDOWN:
        if len(world_state.calibration_points) < 4:
            world_state.calibration_points.append((x, y))
        else:
            _clicked_point = (x, y)
    elif event == cv2.EVENT_MBUTTONDOWN:
        world_state.excluded_points.append((x, y))
    elif event == cv2.EVENT_RBUTTONDOWN:
        _tracked_centroid = None
        _tracked_height_mm = None
        _clicked_point = None
        logger.info("Cleared target lock")


# ── Vision Helpers ───────────────────────────────────────────────────────────

def _depth_to_colormap(depth: np.ndarray, max_depth_mm: float = 4500.0) -> np.ndarray:
    valid = depth > 0
    normalised = np.zeros_like(depth, dtype=np.uint8)
    normalised[valid] = np.clip(depth[valid] / max_depth_mm * 255, 0, 255).astype(np.uint8)
    coloured = cv2.applyColorMap(normalised, cv2.COLORMAP_TURBO)
    coloured[~valid] = 0
    return coloured


def _ir_to_uint8(ir: np.ndarray) -> np.ndarray:
    clamped = np.clip(ir / 10000.0, 0.0, 1.0)
    return (clamped * 255).astype(np.uint8)


def _build_detector(dict_name: str) -> cv2.aruco.ArucoDetector:
    dict_id = getattr(cv2.aruco, dict_name, None)
    if dict_id is None:
        raise ValueError(f"Unknown ArUco dictionary: {dict_name!r}")
    aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)
    params = cv2.aruco.DetectorParameters()
    params.minMarkerPerimeterRate = 0.01
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    return cv2.aruco.ArucoDetector(aruco_dict, params)


def _detect_robot(corners: List, ids: List[int], cfg: Any, state: WorldState) -> None:
    """Find the robot tags in the ArUco detections and update the WorldState."""
    if not corners or not ids:
        return
        
    for corner, tag_id in zip(corners, ids):
        if tag_id == cfg.robot_tag_top or tag_id == cfg.robot_tag_bottom:
            pts = corner[0].astype(int)  # shape (4, 2)
            
            # Centroid
            cx = int(pts[:, 0].mean())
            cy = int(pts[:, 1].mean())
            state.robot_pos = (cx, cy)
            
            # Heading: vector from bottom edge center to top edge center.
            # Assuming standard ArUco corner ordering: top-left, top-right, bottom-right, bottom-left.
            tl, tr, br, bl = pts
            top_center = (tl + tr) / 2.0
            bot_center = (bl + br) / 2.0
            
            dx = top_center[0] - bot_center[0]
            dy = top_center[1] - bot_center[1]
            state.robot_heading = math.atan2(dy, dx)
            state.robot_upright = (tag_id == cfg.robot_tag_top)
            
            # We only track one robot, break on first matching tag
            break


def _draw_hud(img: np.ndarray, state: WorldState, ch1: int, ch2: int, ch3: int, fps: float) -> None:
    """Draw tracking overlay and text HUD on an image."""
    mode_text = "AUTO" if state.auto_mode else "MANUAL (Space to toggle)"
    mode_color = (0, 0, 255) if state.auto_mode else (0, 255, 0)
    
    cv2.putText(img, f"Mode: {mode_text}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, mode_color, 2)
    cv2.putText(img, f"PWM: CH1={ch1} CH2={ch2} CH3={ch3}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
    cv2.putText(img, f"FPS: {fps:.1f}", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
    
    # Draw Robot position and heading line
    if state.robot_pos and state.robot_heading is not None:
        rx, ry = state.robot_pos
        cv2.circle(img, (rx, ry), 8, (0, 255, 255), -1)
        
        # Heading line (length 30)
        hx = int(rx + 30 * math.cos(state.robot_heading))
        hy = int(ry + 30 * math.sin(state.robot_heading))
        cv2.line(img, (rx, ry), (hx, hy), (0, 255, 255), 3)
        cv2.putText(img, "RBT", (rx + 10, ry), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
        
    # Draw Opponent position target
    if state.opponent_pos:
        ox, oy = state.opponent_pos
        cv2.drawMarker(img, (ox, oy), (0, 0, 255), cv2.MARKER_CROSS, 20, 2)
        cv2.putText(img, "TGT", (ox + 10, oy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)


# ── Main Loop ────────────────────────────────────────────────────────────────

def main() -> None:
    global _clicked_point, _tracked_centroid, _tracked_height_mm, world_state

    parser = argparse.ArgumentParser(description="Autonomous Combat Robot Controller")
    parser.add_argument("--config", type=str, default=None, help="Path to config.yaml")
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(name)s] %(levelname)s: %(message)s", datefmt="%H:%M:%S")

    cfg = load_config(args.config)
    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    # Init Hardware
    arduino = ArduinoSerial(cfg)
    planner = Planner(cfg)
    
    stream = KinectStream(
        pipeline=cfg.kinect_pipeline,
        enable_color=True,
        enable_depth=True,
        enable_registered=True,
        flip_horizontal=cfg.flip_horizontal,
    )
    
    segmenter = DepthSegmenter(
        max_depth_mm=cfg.arena_max_depth_mm,
        obstacle_min_height_mm=cfg.obstacle_min_height_mm,
        obstacle_min_area_px=cfg.obstacle_min_area_px,
        obstacle_max_area_px=cfg.obstacle_max_area_px,
    )
    
    detector = _build_detector(cfg.aruco_dict)
    
    logger.info("Starting Autonomous Controller...")
    try:
        stream.start()
    except RuntimeError as exc:
        logger.error("Failed to start Kinect: %s", exc)
        sys.exit(1)

    cv2.namedWindow("Registered Vision", cv2.WINDOW_NORMAL)
    cv2.namedWindow("Depth Seg", cv2.WINDOW_NORMAL)
    cv2.setMouseCallback("Registered Vision", _on_mouse_click)
    cv2.setMouseCallback("Depth Seg", _on_mouse_click)

    calibration_frames: List[np.ndarray] = []
    fps_time = time.time()
    frame_count = 0
    fps = 0.0

    logger.info("Controls: SPACE to toggle AUTO, Click to lock target, Q to quit.")

    try:
        while not shutdown_event.is_set():
            color = stream.read_color()
            depth = stream.read_depth()
            registered = stream.read_registered()

            if depth is None or registered is None:
                time.sleep(0.01)
                continue

            frame_count += 1
            now = time.time()
            if now - fps_time >= 1.0:
                fps = frame_count / (now - fps_time)
                frame_count = 0
                fps_time = now

            # Pre-compute ArUco detections for both calibration and robot tracking
            grey_reg = cv2.cvtColor(registered, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = detector.detectMarkers(grey_reg)
            flat_ids = ids.flatten().tolist() if ids is not None else []

            # 1. Depth Calibration
            if not segmenter.is_calibrated:
                pts = world_state.calibration_points
                if len(pts) == 4:
                    calibration_frames.append(depth)
                    if len(calibration_frames) >= 10:
                        logger.info("4 points selected! Calibrating ground plane...")
                        
                        tag_corners = np.array(pts, dtype=np.int32)
                        mask = np.zeros(depth.shape, dtype=np.uint8)
                        cv2.fillConvexPoly(mask, tag_corners, 255)
                        
                        if not segmenter.calibrate(calibration_frames, mask=mask):
                            logger.warning("Calibration failed! Retrying...")
                            calibration_frames.clear()
                            world_state.calibration_points.clear()
                else:
                    calibration_frames.clear()
                
                # Show waiting screen with instructions
                wait_img = registered.copy()
                cv2.putText(wait_img, f"Click 4 corners on the floor: {len(pts)}/4", (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 165, 255), 2)
                
                for pt in pts:
                    cv2.circle(wait_img, pt, 5, (0, 255, 0), -1)
                if len(pts) > 1:
                    for i in range(len(pts) - 1):
                        cv2.line(wait_img, pts[i], pts[i+1], (0, 255, 0), 2)
                if len(pts) == 4:
                    cv2.line(wait_img, pts[-1], pts[0], (0, 255, 0), 2)
                    
                cv2.imshow("Registered Vision", wait_img)
                cv2.waitKey(1)
                continue

            # 2. Obstacle Segmentation
            obstacles = []
            if depth is not None and segmenter.is_calibrated:
                raw_obstacles = segmenter.find_obstacles(depth)
                # Filter out excluded obstacles
                for obs in raw_obstacles:
                    ox, oy, ow, oh = obs["bbox"]
                    excluded = False
                    for ex, ey in world_state.excluded_points:
                        if ox <= ex <= ox + ow and oy <= ey <= oy + oh:
                            excluded = True
                            break
                    if not excluded:
                        obstacles.append(obs)

            # 3. Handle Mouse Click (Target Lock)
            if _clicked_point is not None:
                cx, cy = _clicked_point
                for obs in obstacles:
                    ox, oy, ow, oh = obs["bbox"]
                    if ox <= cx <= ox + ow and oy <= cy <= oy + oh:
                        _tracked_centroid = (ox + ow // 2, oy + oh // 2)
                        _tracked_height_mm = obs["mean_height_mm"]
                        logger.info("Locked obstacle at %s (Height: %.1f mm)", _tracked_centroid, _tracked_height_mm)
                        break
                _clicked_point = None

            # 4. Height Filtering
            if _tracked_height_mm is not None:
                height_tolerance = 25.0
                filtered_obstacles = []
                for obs in obstacles:
                    if abs(obs["mean_height_mm"] - _tracked_height_mm) <= height_tolerance:
                        filtered_obstacles.append(obs)
                obstacles = filtered_obstacles

            tracked_obs = None
            if _tracked_centroid is not None and obstacles:
                # Find the closest obstacle to the last tracked centroid
                tx, ty = _tracked_centroid
                best_dist = float("inf")
                for obs in obstacles:
                    cx, cy = obs["centroid"]
                    dist = (cx - tx) ** 2 + (cy - ty) ** 2
                    if dist < best_dist and dist < 10000:  # 100px radius
                        best_dist = dist
                        tracked_obs = obs

                if tracked_obs is not None:
                    _tracked_centroid = tracked_obs["centroid"]
                    # Update height with EMA
                    _tracked_height_mm = 0.9 * _tracked_height_mm + 0.1 * tracked_obs["mean_height_mm"]
                else:
                    logger.info("Lost spatial track, falling back to global height search")
                    _tracked_centroid = None
            
            # 5. Global Reacquisition
            if _tracked_centroid is None and _tracked_height_mm is not None and obstacles:
                # We have a registered height, but no spatial lock. Search for best match.
                best_diff = float("inf")
                best_obs = None
                for obs in obstacles:
                    diff = abs(obs["mean_height_mm"] - _tracked_height_mm)
                    if diff < best_diff:
                        best_diff = diff
                        best_obs = obs
                
                if best_obs is not None:
                    tracked_obs = best_obs
                    _tracked_centroid = tracked_obs["centroid"]
                    _tracked_height_mm = 0.9 * _tracked_height_mm + 0.1 * tracked_obs["mean_height_mm"]
                    logger.info("Reacquired obstacle at %s (Height: %.1f mm)", _tracked_centroid, _tracked_height_mm)
            
            if tracked_obs is not None:
                world_state.opponent_pos = tracked_obs["centroid"]
                world_state.opponent_area = tracked_obs["area"]
                world_state.opponent_height = tracked_obs["mean_height_mm"]
            else:
                world_state.opponent_pos = None

            # 4. Robot ArUco Detection
            old_pos = world_state.robot_pos
            _detect_robot(corners, flat_ids, cfg, world_state)
            
            if world_state.robot_pos is None and old_pos is not None:
                # ArUco missing this frame, we could coast using Kalman but for now we'll just wait
                pass

            # 5. Planning & Control
            ch1, ch2, ch3 = planner.compute_commands(world_state)
            arduino.send(ch1, ch2, ch3)

            # 6. UI Drawing
            disp_depth = _depth_to_colormap(depth)
            mask = segmenter.get_ground_mask(depth)
            disp_depth[mask] = (disp_depth[mask].astype(np.float32) * 0.5 + np.array([0, 200, 0]) * 0.5).astype(np.uint8)

            disp_reg = registered.copy()

            for obs in obstacles:
                ox, oy, ow, oh = obs["bbox"]
                color = (0, 255, 0) if obs == tracked_obs else (0, 0, 255)
                cv2.rectangle(disp_depth, (ox, oy), (ox + ow, oy + oh), color, 2)
                cv2.rectangle(disp_reg, (ox, oy), (ox + ow, oy + oh), color, 2)

            _draw_hud(disp_reg, world_state, ch1, ch2, ch3, fps)
            _draw_hud(disp_depth, world_state, ch1, ch2, ch3, fps)

            cv2.imshow("Depth Seg", disp_depth)
            cv2.imshow("Registered Vision", disp_reg)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                shutdown_event.set()
            elif key == ord(" "):
                world_state.auto_mode = not world_state.auto_mode
                if world_state.auto_mode:
                    logger.warning("AUTO MODE ENGAGED!")
                else:
                    logger.info("Auto mode disabled. Motors neutral.")
                    arduino.send_neutral()

    finally:
        arduino.close()
        stream.stop()
        cv2.destroyAllWindows()
        logger.info("Controller shut down.")


if __name__ == "__main__":
    main()
