"""
__main__.py — Entry point for the combat robot controller.

Run with:  python -m robot_control [--config path/to/config.yaml]
"""

from __future__ import annotations

import argparse
import logging
import math
import signal
import sys
import threading
import time
from typing import Optional

import cv2

from robot_control.config import load_config
from robot_control.camera import CameraStream
from robot_control.filters import KalmanTracker
from robot_control.serial_output import ArduinoSerial
from robot_control.arena import ArenaBounds
from robot_control.detection.apriltag_tracker import (
    AprilTagState,
    detect_tags,
    estimate_heading,
    make_detector,
    robot1_midpoint,
)
from robot_control.detection.color_tracker import Robot1ColorTracker, detect_r2_color
from robot_control.planning.path_planner import build_evasion_path, build_path
from robot_control.planning.pursuit import pure_pursuit
from robot_control.planning.evasion import evasion_offset
from robot_control.ui.drawing import (
    draw_lookahead,
    draw_path,
    draw_robot_markers,
    draw_tags,
    draw_threat_indicator,
)
from robot_control.ui.hud import draw_auto_hud
from robot_control.ui.trackbars import (
    create_trackbars,
    make_mouse_callback,
    read_trackbars,
)
from robot_control.state import TrackingState, AppState, shutdown_event
from robot_control.profile import load_profile, save_profile
from robot_control.log import init_log, log_row
from robot_control.watchdog import Watchdog

logger = logging.getLogger(__name__)


# ── Signal handling (Action 9) ───────────────────────────────────────────────

def _signal_handler(sig: int, _frame: object) -> None:
    logger.warning("Signal %s — shutting down...", signal.Signals(sig).name)
    shutdown_event.set()


# ── Terminal loop ────────────────────────────────────────────────────────────

def _terminal_loop(tracking: TrackingState, profile_file: str) -> None:
    logger.info("Terminal commands: save | load | info | quit")
    while not shutdown_event.is_set():
        try:
            cmd = input("> ").strip().lower()
        except EOFError:
            break
        if cmd == "quit":
            shutdown_event.set()
        elif cmd == "save":
            save_profile(tracking, profile_file)
        elif cmd == "load":
            load_profile(tracking, profile_file)
        elif cmd == "info":
            print(
                f"  R2 HSV lower={tracking.hsv_lower.tolist()} "
                f"upper={tracking.hsv_upper.tolist()}"
            )
        else:
            print("  Commands: save  load  info  quit")


# ── Main ─────────────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(description="Combat Robot Controller")
    parser.add_argument(
        "--config", type=str, default=None, help="Path to config.yaml"
    )
    args = parser.parse_args()

    # Logging setup (Action 11)
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(name)s] %(levelname)s: %(message)s",
        datefmt="%H:%M:%S",
    )

    cfg = load_config(args.config)

    # Signal handlers (Action 9)
    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    # Instantiate components — no module-level globals (Action 3)
    detector = make_detector(cfg)
    try:
        cam = CameraStream(cfg)
    except RuntimeError as exc:
        logger.error("Camera failed: %s", exc)
        return

    arduino = ArduinoSerial(cfg)
    r1_color = Robot1ColorTracker(cfg)
    arena = ArenaBounds(cfg)
    kf1 = KalmanTracker(
        max_coast_frames=cfg.max_coast_frames, proc_noise=0.5, meas_noise=4.0
    )
    kf2 = KalmanTracker(
        max_coast_frames=cfg.max_coast_frames, proc_noise=1.0, meas_noise=8.0
    )

    tag_state = AprilTagState()
    tracking = TrackingState(kf1=kf1, kf2=kf2, arena=arena)
    app = AppState(target_id=cfg.target_tag_a, ch3=cfg.weapon_off_value)

    # Watchdogs (Action 4)
    frame_wd = Watchdog("frame", threshold_ms=200, failsafe=arduino.send_neutral)
    serial_wd = Watchdog("serial", threshold_ms=100, failsafe=arduino.send_neutral)
    frame_wd.start()
    serial_wd.start()

    # GUI setup
    cv2.namedWindow("Tracker", cv2.WINDOW_NORMAL)
    cv2.namedWindow("Mask", cv2.WINDOW_NORMAL)
    create_trackbars(tracking, cfg.exposure)
    cv2.setMouseCallback("Tracker", make_mouse_callback(tracking))
    load_profile(tracking, cfg.profile_file)

    log_file, log_writer = init_log(cfg.position_log)

    threading.Thread(
        target=_terminal_loop,
        args=(tracking, cfg.profile_file),
        daemon=True,
    ).start()

    last_t = time.time()

    while not shutdown_event.is_set():
        frame = cam.read()
        if frame is None:
            time.sleep(0.005)
            continue

        frame_wd.feed()

        now = time.time()
        dt = max(now - last_t, 1e-4)
        fps = 1.0 / dt
        last_t = now

        # Action 6: single HSV conversion, reused everywhere
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        tracking.current_hsv_frame = hsv_frame
        read_trackbars(tracking, cam)

        # ── Robot 1 ──────────────────────────────────────────────────────
        tags = detect_tags(frame, detector, tag_state, cfg)
        raw_mid = robot1_midpoint(tags, tag_state, cfg)

        if raw_mid:
            r1_pos = kf1.update(raw_mid[0], raw_mid[1], dt)
            r1_src = tag_state.mid_source
        else:
            color_pos = r1_color.detect(frame, hsv_frame)
            if color_pos:
                r1_pos = kf1.update(color_pos[0], color_pos[1], dt)
                r1_src = "color-fallback"
            else:
                r1_pos = kf1.predict(dt)
                r1_src = "coast" if r1_pos else "lost"

        r1_color.maybe_resample(hsv_frame, r1_pos)
        current_heading = estimate_heading(tags, kf1, cfg)

        # ── Robot 2 (Action 6: pass hsv_frame, not frame) ────────────────
        color_result, mask = detect_r2_color(
            hsv_frame, tracking.hsv_lower, tracking.hsv_upper, cfg.min_contour_area
        )
        if color_result:
            cx, cy = color_result[0], color_result[1]
            r2_pos = kf2.update(cx, cy, dt)
            r2_src = "color"
        else:
            r2_pos = kf2.predict(dt)
            r2_src = "coast" if r2_pos else "lost"

        r2_vel = kf2.velocity()

        # ── Target tag ───────────────────────────────────────────────────
        target_pos = None
        if app.target_id in tags:
            tc = tags[app.target_id].center
            target_pos = (int(tc[0]), int(tc[1]))

        # ── Path planning ────────────────────────────────────────────────
        needs_plan = app.current_path is None
        if r1_pos and app.last_r1_path:
            if (
                math.hypot(
                    r1_pos[0] - app.last_r1_path[0],
                    r1_pos[1] - app.last_r1_path[1],
                )
                > cfg.path_replan_dist
            ):
                needs_plan = True
        if r2_pos and app.last_r2_path:
            if (
                math.hypot(
                    r2_pos[0] - app.last_r2_path[0],
                    r2_pos[1] - app.last_r2_path[1],
                )
                > cfg.path_replan_dist
            ):
                needs_plan = True

        if needs_plan and r1_pos and r2_pos and target_pos:
            app.current_path = build_path(
                r1_pos, r2_pos, target_pos, arena, cfg, r1_heading_deg=current_heading
            )
            app.last_r1_path = r1_pos
            app.last_r2_path = r2_pos

        if target_pos is None:
            app.current_path = None

        # ── Threat evasion ───────────────────────────────────────────────
        threat_evade = None
        if r1_pos and r2_pos:
            threat_evade = evasion_offset(r1_pos, r2_pos, r2_vel, arena, cfg)

        # ── Control output ───────────────────────────────────────────────
        if app.auto_mode and r1_pos:
            if threat_evade:
                evade_path = build_evasion_path(
                    r1_pos, threat_evade, arena, r1_heading_deg=current_heading
                )
                drive_cmd, steer_cmd = pure_pursuit(
                    r1_pos, current_heading, evade_path, cfg
                )
                app.ch3 = cfg.weapon_off_value
            elif app.current_path is not None:
                drive_cmd, steer_cmd = pure_pursuit(
                    r1_pos, current_heading, app.current_path, cfg
                )
                app.ch3 = cfg.weapon_on_value
            else:
                drive_cmd, steer_cmd = cfg.pwm_neutral, cfg.pwm_neutral
                app.ch3 = cfg.weapon_off_value
            app.ch2 = drive_cmd
            app.ch1 = steer_cmd
            arduino.send(app.ch1, app.ch2, app.ch3)
            serial_wd.feed()
        else:
            app.ch1 = app.ch2 = cfg.pwm_neutral
            app.ch3 = cfg.weapon_off_value
            arduino.send_neutral()
            serial_wd.feed()

        # ── Draw ─────────────────────────────────────────────────────────
        draw_tags(frame, tags, cfg.target_tag_a, cfg.target_tag_b)
        draw_robot_markers(
            frame,
            r1_pos,
            r1_src,
            r2_pos,
            r2_src,
            kf1.velocity(),
            r2_vel,
            current_heading,
        )
        draw_path(frame, app.current_path, r2_pos, target_pos, app.target_id, cfg)
        draw_lookahead(frame, r1_pos, app.current_path, cfg.lookahead_dist)
        draw_threat_indicator(frame, r2_pos, threat_evade)
        if app.show_bounds:
            arena.draw(frame)
        draw_auto_hud(
            frame, app.auto_mode, app.target_id, app.ch1, app.ch2, app.ch3, fps
        )

        log_row(log_writer, r1_pos, r1_src, r2_pos, r2_src, app.ch1, app.ch2, app.ch3, app.auto_mode)
        log_file.flush()

        cv2.imshow("Tracker", frame)
        cv2.imshow("Mask", mask)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            shutdown_event.set()
        elif key == ord("a"):
            app.auto_mode = not app.auto_mode
            logger.info("Autonomous: %s", app.auto_mode)
            if not app.auto_mode:
                arduino.send_neutral()
        elif key == ord("t"):
            app.target_id = (
                cfg.target_tag_b
                if app.target_id == cfg.target_tag_a
                else cfg.target_tag_a
            )
            app.current_path = None
            logger.info("Target switched to TAG %d", app.target_id)
        elif key == ord("b"):
            app.show_bounds = not app.show_bounds
        elif key in (ord("+"), ord("=")):
            arena.change_offset(+5)
            app.current_path = None
        elif key == ord("-"):
            arena.change_offset(-5)
            app.current_path = None

    # ── Shutdown ─────────────────────────────────────────────────────────
    frame_wd.stop()
    serial_wd.stop()
    arduino.send_neutral()
    arduino.close()
    cam.release()
    log_file.close()
    cv2.destroyAllWindows()
    logger.info("Positions saved to %s", cfg.position_log)


if __name__ == "__main__":
    main()
