"""
depth_analysis.py — Capture depth frame and analyze ground plane + obstacles.

Captures a single depth frame, fits a ground plane via RANSAC, identifies
objects above the plane, and saves annotated visualizations to disk.

Run with:
    python tools/depth_analysis.py
"""
import logging
import sys
import time
from pathlib import Path

import cv2
import numpy as np

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)s: %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("depth_analysis")

OUT_DIR = Path("tools/diag_frames")


def ransac_ground_plane(
    depth: np.ndarray,
    max_depth_mm: float = 4500.0,
    iterations: int = 200,
    inlier_threshold_mm: float = 20.0,
    min_inlier_ratio: float = 0.3,
) -> tuple:
    """Fit a ground plane to the depth image using RANSAC.

    The depth frame from the Kinect is a 2D array where each pixel (y, x)
    contains the distance in mm from the camera. We treat each valid pixel
    as a 3D point (x, y, depth_mm) in image-space coordinates. For a flat
    floor viewed at an angle, the depth values form a plane in this space:

        depth = a*x + b*y + c

    This avoids needing camera intrinsics — we fit a plane in (row, col, depth)
    space directly.

    Returns
    -------
    (a, b, c) : plane coefficients (depth = a*col + b*row + c)
    inlier_mask : bool array same shape as depth, True for ground pixels
    """
    h, w = depth.shape

    # Build coordinate arrays for valid pixels only.
    valid = (depth > 0) & (depth < max_depth_mm)
    rows, cols = np.where(valid)
    depths = depth[valid]

    n_valid = len(depths)
    if n_valid < 100:
        log.warning("Too few valid depth pixels (%d) for plane fitting", n_valid)
        return None, np.zeros_like(depth, dtype=bool)

    log.info("Valid depth pixels: %d / %d (%.1f%%)",
             n_valid, h * w, 100 * n_valid / (h * w))

    best_inliers = 0
    best_plane = None

    for _ in range(iterations):
        # Pick 3 random points.
        idx = np.random.choice(n_valid, 3, replace=False)
        r = rows[idx].astype(np.float64)
        c = cols[idx].astype(np.float64)
        d = depths[idx].astype(np.float64)

        # Solve: d = a*c + b*r + const  →  [c r 1] @ [a b const]^T = d
        A = np.column_stack([c, r, np.ones(3)])
        try:
            plane = np.linalg.solve(A, d)
        except np.linalg.LinAlgError:
            continue

        a, b, const = plane

        # Compute residuals for all valid pixels.
        predicted = a * cols.astype(np.float64) + b * rows.astype(np.float64) + const
        residuals = np.abs(depths.astype(np.float64) - predicted)
        inlier_count = np.sum(residuals < inlier_threshold_mm)

        if inlier_count > best_inliers:
            best_inliers = inlier_count
            best_plane = plane

    if best_plane is None:
        log.warning("RANSAC failed to find a plane")
        return None, np.zeros_like(depth, dtype=bool)

    a, b, c = best_plane
    log.info("Ground plane: depth = %.4f*col + %.4f*row + %.1f", a, b, c)
    log.info("RANSAC inliers: %d / %d (%.1f%%)",
             best_inliers, n_valid, 100 * best_inliers / n_valid)

    # Build full inlier mask.
    predicted_full = a * np.arange(w)[None, :] + b * np.arange(h)[:, None] + c
    residual_full = np.abs(depth.astype(np.float64) - predicted_full)
    inlier_mask = valid & (residual_full < inlier_threshold_mm)

    return (a, b, c), inlier_mask


def find_obstacles(
    depth: np.ndarray,
    plane_coeffs: tuple,
    inlier_mask: np.ndarray,
    max_depth_mm: float = 4500.0,
    above_threshold_mm: float = 30.0,
    min_obstacle_area_px: int = 200,
) -> list:
    """Find objects above the ground plane.

    An obstacle pixel satisfies:
        - depth is valid and within max_depth_mm
        - depth is MORE THAN above_threshold_mm CLOSER to the camera than the
          predicted ground plane at that pixel (i.e. sticking up from the floor)

    Returns list of dicts with keys: bbox, area, centroid, mean_height_mm.
    """
    if plane_coeffs is None:
        return []

    a, b, c = plane_coeffs
    h, w = depth.shape
    valid = (depth > 0) & (depth < max_depth_mm)

    # Predicted ground depth at each pixel.
    predicted = a * np.arange(w)[None, :] + b * np.arange(h)[:, None] + c

    # "Above the ground" means closer to the camera → depth < predicted - threshold.
    above_mask = valid & ~inlier_mask & (depth < predicted - above_threshold_mm)

    # Clean up with morphological ops.
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    above_uint8 = above_mask.astype(np.uint8) * 255
    above_uint8 = cv2.morphologyEx(above_uint8, cv2.MORPH_OPEN, kernel)
    above_uint8 = cv2.morphologyEx(above_uint8, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(above_uint8, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    obstacles = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < min_obstacle_area_px:
            continue
        x, y, bw, bh = cv2.boundingRect(cnt)
        mask_roi = above_uint8[y:y+bh, x:x+bw] > 0
        depth_roi = depth[y:y+bh, x:x+bw]
        pred_roi = predicted[y:y+bh, x:x+bw]
        heights = pred_roi[mask_roi] - depth_roi[mask_roi]
        mean_height = float(np.mean(heights)) if len(heights) > 0 else 0.0

        M = cv2.moments(cnt)
        cx = int(M["m10"] / M["m00"]) if M["m00"] > 0 else x + bw // 2
        cy = int(M["m01"] / M["m00"]) if M["m00"] > 0 else y + bh // 2

        obstacles.append({
            "bbox": (x, y, bw, bh),
            "area": int(area),
            "centroid": (cx, cy),
            "mean_height_mm": mean_height,
        })

    obstacles.sort(key=lambda o: o["area"], reverse=True)
    return obstacles


def depth_to_colormap(depth, max_depth_mm=4500.0):
    valid = depth > 0
    norm = np.zeros_like(depth, dtype=np.uint8)
    norm[valid] = np.clip(depth[valid] / max_depth_mm * 255, 0, 255).astype(np.uint8)
    coloured = cv2.applyColorMap(norm, cv2.COLORMAP_TURBO)
    coloured[~valid] = 0
    return coloured


def main():
    from pylibfreenect2 import (
        Freenect2, SyncMultiFrameListener, FrameType
    )
    from pylibfreenect2 import OpenGLPacketPipeline
    from robot_control.config import load_config

    cfg = load_config()
    OUT_DIR.mkdir(parents=True, exist_ok=True)

    log.info("=" * 70)
    log.info("Depth Ground Plane + Obstacle Analysis")
    log.info("=" * 70)

    fn = Freenect2()
    fn.enumerateDevices()
    serial = fn.getDeviceSerialNumber(0)
    pipeline = OpenGLPacketPipeline()
    device = fn.openDevice(serial, pipeline=pipeline)

    listener = SyncMultiFrameListener(FrameType.Color | FrameType.Depth | FrameType.Ir)
    device.setColorFrameListener(listener)
    device.setIrAndDepthFrameListener(listener)
    device.start()

    log.info("Warming up 3s...")
    time.sleep(3.0)

    # Collect several frames and average depth to reduce noise.
    NUM_FRAMES = 10
    log.info("Collecting %d depth frames for averaging...", NUM_FRAMES)
    depth_acc = None
    depth_count = None
    color_frame = None

    for i in range(NUM_FRAMES):
        frames = listener.waitForNewFrame(milliseconds=2000)
        if frames is None:
            continue
        try:
            d = frames[FrameType.Depth].asarray(np.float32).copy()
            c = frames[FrameType.Color].asarray(np.uint8)[:, :, :3].copy()
        finally:
            listener.release(frames)

        if cfg.flip_horizontal:
            d = cv2.flip(d, 1)
            c = cv2.flip(c, 1)

        if depth_acc is None:
            depth_acc = np.zeros_like(d, dtype=np.float64)
            depth_count = np.zeros_like(d, dtype=np.int32)

        valid = d > 0
        depth_acc[valid] += d[valid]
        depth_count[valid] += 1
        color_frame = c

    device.stop()
    device.close()

    # Average depth.
    with np.errstate(divide='ignore', invalid='ignore'):
        depth_avg = np.where(depth_count > 0, depth_acc / depth_count, 0).astype(np.float32)

    log.info("Averaged depth: valid pixels=%d  min=%.0f mm  max=%.0f mm  mean=%.0f mm",
             np.sum(depth_avg > 0),
             depth_avg[depth_avg > 0].min() if np.any(depth_avg > 0) else 0,
             depth_avg[depth_avg > 0].max() if np.any(depth_avg > 0) else 0,
             depth_avg[depth_avg > 0].mean() if np.any(depth_avg > 0) else 0)

    # Save raw depth vis.
    cv2.imwrite(str(OUT_DIR / "depth_raw.jpg"), depth_to_colormap(depth_avg))
    cv2.imwrite(str(OUT_DIR / "color_for_depth.jpg"), color_frame)
    log.info("Saved depth_raw.jpg and color_for_depth.jpg")

    # ── Fit ground plane ────────────────────────────────────────────────
    MAX_DEPTH = 3500.0  # mm — cutoff to ignore walls
    plane, inlier_mask = ransac_ground_plane(
        depth_avg,
        max_depth_mm=MAX_DEPTH,
        iterations=500,
        inlier_threshold_mm=15.0,
    )

    if plane is None:
        log.error("Ground plane fitting failed — aborting")
        sys.exit(1)

    # Visualize ground plane.
    vis_plane = depth_to_colormap(depth_avg)
    vis_plane[inlier_mask] = (
        vis_plane[inlier_mask].astype(np.float32) * 0.5
        + np.array([0, 200, 0], dtype=np.float32) * 0.5
    ).astype(np.uint8)
    cv2.imwrite(str(OUT_DIR / "ground_plane.jpg"), vis_plane)
    log.info("Saved ground_plane.jpg (green = ground inliers)")

    # ── Find obstacles ──────────────────────────────────────────────────
    obstacles = find_obstacles(
        depth_avg,
        plane,
        inlier_mask,
        max_depth_mm=MAX_DEPTH,
        above_threshold_mm=25.0,
        min_obstacle_area_px=100,
    )

    log.info("Found %d obstacle(s):", len(obstacles))
    for i, obs in enumerate(obstacles):
        log.info("  [%d] bbox=%s  area=%d px  centroid=%s  height=%.0f mm",
                 i, obs["bbox"], obs["area"], obs["centroid"], obs["mean_height_mm"])

    # Annotated visualization.
    vis_obs = depth_to_colormap(depth_avg)
    # Tint ground green.
    vis_obs[inlier_mask] = (
        vis_obs[inlier_mask].astype(np.float32) * 0.6
        + np.array([0, 180, 0], dtype=np.float32) * 0.4
    ).astype(np.uint8)

    for i, obs in enumerate(obstacles):
        x, y, bw, bh = obs["bbox"]
        cv2.rectangle(vis_obs, (x, y), (x + bw, y + bh), (0, 0, 255), 2)
        cx, cy = obs["centroid"]
        cv2.circle(vis_obs, (cx, cy), 5, (0, 0, 255), -1)
        label = f"H={obs['mean_height_mm']:.0f}mm A={obs['area']}px"
        cv2.putText(vis_obs, label, (x, y - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

    cv2.imwrite(str(OUT_DIR / "obstacles.jpg"), vis_obs)
    log.info("Saved obstacles.jpg (green=ground, red boxes=obstacles)")

    log.info("=" * 70)
    log.info("Analysis complete. Check: %s", OUT_DIR.resolve())


if __name__ == "__main__":
    main()
