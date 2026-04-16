"""
kinect_viewer.py — Diagnostic viewer for Kinect V2 streams.

Displays up to four windows showing the raw sensor data from a connected
Kinect V2:

  - **Color**:       1920×1080 RGB image
  - **Depth**:       512×424 depth map (colourised for visibility)
  - **IR**:          512×424 infrared image (normalised for visibility)
  - **Registered**:  512×424 colour image mapped to depth coordinates

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

import cv2
import numpy as np

from robot_control.vision.kinect_stream import KinectStream

logger = logging.getLogger(__name__)

_running = True


def _signal_handler(sig: int, _frame: object) -> None:
    global _running
    logger.warning("Signal %s — shutting down viewer...", signal.Signals(sig).name)
    _running = False


def _depth_to_colormap(depth: np.ndarray, max_depth_mm: float = 4500.0) -> np.ndarray:
    """Convert a float32 depth frame (mm) to a colourised BGR image for display.

    Pixels with depth == 0 (invalid) are drawn black.
    """
    # Clamp and normalise to 0–255.
    valid = depth > 0
    normalised = np.zeros_like(depth, dtype=np.uint8)
    normalised[valid] = np.clip(depth[valid] / max_depth_mm * 255, 0, 255).astype(np.uint8)
    # Apply a colour map for easier visual interpretation.
    coloured = cv2.applyColorMap(normalised, cv2.COLORMAP_TURBO)
    # Black out invalid pixels.
    coloured[~valid] = 0
    return coloured


def _ir_to_display(ir: np.ndarray) -> np.ndarray:
    """Normalise a float32 IR frame to uint8 for display.

    The Kinect V2 IR sensor has a wide dynamic range; we clamp and scale
    so the image is visible without blowout.
    """
    # Typical useful range is roughly 0–65535, but most energy is < 10000.
    clamped = np.clip(ir / 10000.0, 0.0, 1.0)
    grey = (clamped * 255).astype(np.uint8)
    return cv2.cvtColor(grey, cv2.COLOR_GRAY2BGR)


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

    logger.info("Starting Kinect V2 viewer (pipeline=%s)...", args.pipeline)

    stream = KinectStream(
        pipeline=args.pipeline,
        enable_color=True,
        enable_depth=True,
        enable_registered=True,
    )

    try:
        stream.start()
    except RuntimeError as exc:
        logger.error("Failed to start Kinect: %s", exc)
        sys.exit(1)

    cv2.namedWindow("Color", cv2.WINDOW_NORMAL)
    cv2.namedWindow("Depth", cv2.WINDOW_NORMAL)
    cv2.namedWindow("IR", cv2.WINDOW_NORMAL)
    cv2.namedWindow("Registered", cv2.WINDOW_NORMAL)

    # Resize windows to reasonable defaults so they don't fill the screen.
    cv2.resizeWindow("Color", 960, 540)
    cv2.resizeWindow("Depth", 512, 424)
    cv2.resizeWindow("IR", 512, 424)
    cv2.resizeWindow("Registered", 512, 424)

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

            # FPS counter.
            frame_count += 1
            now = time.time()
            elapsed = now - fps_time
            if elapsed >= 1.0:
                fps = frame_count / elapsed
                frame_count = 0
                fps_time = now

            # --- Color window ---
            if color is not None:
                disp_color = color.copy()
                cv2.putText(
                    disp_color,
                    f"Color 1920x1080  FPS: {fps:.1f}",
                    (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1.0,
                    (0, 255, 0),
                    2,
                )
                cv2.imshow("Color", disp_color)

            # --- Depth window ---
            if depth is not None:
                disp_depth = _depth_to_colormap(depth)
                cv2.putText(
                    disp_depth,
                    f"Depth 512x424  FPS: {fps:.1f}",
                    (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (255, 255, 255),
                    1,
                )
                cv2.imshow("Depth", disp_depth)

            # --- IR window ---
            if ir is not None:
                disp_ir = _ir_to_display(ir)
                cv2.putText(
                    disp_ir,
                    f"IR 512x424  FPS: {fps:.1f}",
                    (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (255, 255, 255),
                    1,
                )
                cv2.imshow("IR", disp_ir)

            # --- Registered window ---
            if registered is not None:
                disp_reg = registered.copy()
                cv2.putText(
                    disp_reg,
                    f"Registered 512x424  FPS: {fps:.1f}",
                    (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 0),
                    1,
                )
                cv2.imshow("Registered", disp_reg)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                _running = False

    finally:
        stream.stop()
        cv2.destroyAllWindows()
        logger.info("Kinect viewer shut down.")


if __name__ == "__main__":
    main()
