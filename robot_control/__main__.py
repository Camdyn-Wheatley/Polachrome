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
_clicked_point: Optional[Tuple[int, int]] = None


def _signal_handler(sig: int, _frame: object) -> None:
    logger.warning("Signal %s — shutting down...", signal.Signals(sig).name)
    shutdown_event.set()


def _on_mouse_click(event: int, x: int, y: int, flags: int, param: Any) -> None:
    global _clicked_point
    if event == cv2.EVENT_LBUTTONDOWN:
        _clicked_point = (x, y)


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
    global _clicked_point, world_state

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
                floor_tag_idx = flat_ids.index(cfg.arena_floor_tag) if cfg.arena_floor_tag in flat_ids else -1
                
                if floor_tag_idx >= 0:
                    calibration_frames.append(depth)
                    if len(calibration_frames) >= 10:
                        logger.info("Floor tag (ID=%d) found! Calibrating ground plane...", cfg.arena_floor_tag)
                        
                        tag_corners = corners[floor_tag_idx][0].astype(np.int32)
                        mask = np.zeros(depth.shape, dtype=np.uint8)
                        cv2.fillConvexPoly(mask, tag_corners, 255)
                        
                        if not segmenter.calibrate(calibration_frames, mask=mask):
                            logger.warning("Calibration failed! Retrying...")
                            calibration_frames.clear()
                else:
                    calibration_frames.clear()
                
                # Show waiting screen
                wait_img = registered.copy()
                msg = f"Waiting for floor tag (ID={cfg.arena_floor_tag})..."
                cv2.putText(wait_img, msg, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 165, 255), 2)
                cv2.imshow("Registered Vision", wait_img)
                cv2.waitKey(1)
                continue

            # 2. Obstacle Segmentation
            obstacles = segmenter.find_obstacles(depth)

            # 3. Handle Mouse Click (Target Lock)
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
                    world_state.opponent_pos = best_obs["centroid"]
                    world_state.opponent_area = best_obs["area"]
                    world_state.opponent_height = best_obs["mean_height_mm"]
                    logger.info("Locked on target at %s", world_state.opponent_pos)
                else:
                    world_state.opponent_pos = None
                    logger.info("Cleared target lock")

            # Update tracked target position
            tracked_obs = None
            if world_state.opponent_pos is not None and obstacles:
                # Find closest obstacle to last known position
                tx, ty = world_state.opponent_pos
                closest = min(obstacles, key=lambda o: (o["centroid"][0] - tx)**2 + (o["centroid"][1] - ty)**2)
                dist_sq = (closest["centroid"][0] - tx)**2 + (closest["centroid"][1] - ty)**2
                
                if dist_sq < 10000:  # within 100px
                    tracked_obs = closest
                    world_state.opponent_pos = closest["centroid"]
                    world_state.opponent_area = closest["area"]
                    world_state.opponent_height = closest["mean_height_mm"]
                else:
                    world_state.opponent_pos = None
                    logger.info("Lost track of opponent target")

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
