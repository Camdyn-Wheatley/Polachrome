"""
kinect_detect_diag.py — Non-GUI detection diagnostic.

Captures N color frames from the Kinect, tries detection with multiple
ArUco dictionaries (including the user's 25H9 tags), saves the captured
image to disk, and prints a full report to the terminal.

Run with:
    python tools/kinect_detect_diag.py
"""
import logging
import sys
import time
from pathlib import Path

import cv2
import numpy as np

# ── Logging ─────────────────────────────────────────────────────────────────
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)s: %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("detect_diag")

# ── Dictionaries to try ──────────────────────────────────────────────────────
DICTS_TO_TRY = [
    "DICT_APRILTAG_25H9",
    "DICT_APRILTAG_36H11",
    "DICT_APRILTAG_36H10",
    "DICT_APRILTAG_16H5",
    "DICT_4X4_50",
    "DICT_5X5_50",
    "DICT_6X6_250",
]

# Very loose parameters to maximise chance of detection in diagnostics.
def _make_permissive_params() -> cv2.aruco.DetectorParameters:
    p = cv2.aruco.DetectorParameters()
    p.minMarkerPerimeterRate = 0.005   # very small markers allowed
    p.maxMarkerPerimeterRate = 4.0
    p.polygonalApproxAccuracyRate = 0.08
    p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    p.adaptiveThreshWinSizeMin = 3
    p.adaptiveThreshWinSizeMax = 53
    p.adaptiveThreshWinSizeStep = 4
    p.adaptiveThreshConstant = 7
    return p


def _try_dict(name: str, grey: np.ndarray, params: cv2.aruco.DetectorParameters):
    """Return (corners, ids) for this dictionary or ([], []) on error."""
    dict_id = getattr(cv2.aruco, name, None)
    if dict_id is None:
        return [], []
    try:
        d = cv2.aruco.getPredefinedDictionary(dict_id)
        det = cv2.aruco.ArucoDetector(d, params)
        corners, ids, rejected = det.detectMarkers(grey)
        flat_ids = ids.flatten().tolist() if ids is not None else []
        return corners, flat_ids
    except Exception as exc:
        log.warning("  %s: exception %s", name, exc)
        return [], []


def main() -> None:
    from pylibfreenect2 import (
        Freenect2, SyncMultiFrameListener, FrameType, Frame
    )
    from pylibfreenect2 import OpenGLPacketPipeline

    log.info("=" * 70)
    log.info("Kinect ArUco Detection Diagnostic")
    log.info("=" * 70)

    fn = Freenect2()
    n = fn.enumerateDevices()
    if n == 0:
        log.error("No Kinect found!")
        sys.exit(1)

    serial = fn.getDeviceSerialNumber(0)
    log.info("Kinect serial: %s", serial)

    pipeline = OpenGLPacketPipeline()
    device = fn.openDevice(serial, pipeline=pipeline)

    listener = SyncMultiFrameListener(FrameType.Color)
    device.setColorFrameListener(listener)
    device.start()
    log.info("Kinect started. Warming up 2s...")
    time.sleep(2.0)

    params = _make_permissive_params()
    out_dir = Path("tools/diag_frames")
    out_dir.mkdir(parents=True, exist_ok=True)

    best_total = 0
    best_frame_path = None
    all_results = {}   # dict_name -> list of id sets found across frames

    NUM_FRAMES = 30
    log.info("Capturing %d frames and running detection...", NUM_FRAMES)
    log.info("")

    for frame_idx in range(NUM_FRAMES):
        frames = listener.waitForNewFrame(milliseconds=2000)
        if frames is None:
            log.warning("Frame %d: timeout", frame_idx)
            continue

        try:
            color_frame = frames[FrameType.Color]
            bgr = color_frame.asarray(np.uint8)[:, :, :3].copy()
        finally:
            listener.release(frames)

        grey = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)

        frame_total = 0
        frame_summary = []
        for dict_name in DICTS_TO_TRY:
            corners, ids = _try_dict(dict_name, grey, params)
            if ids:
                frame_summary.append(f"{dict_name}→{ids}")
                frame_total += len(ids)
                all_results.setdefault(dict_name, []).extend(ids)

        if frame_summary:
            log.info("Frame %2d: DETECTED — %s", frame_idx, "  |  ".join(frame_summary))
        else:
            log.info("Frame %2d: no detections", frame_idx)

        # Save the frame with the most detections.
        if frame_total > best_total or (frame_total == best_total and best_frame_path is None):
            best_total = frame_total
            # Draw any detected markers on the saved frame.
            annotated = bgr.copy()
            for dict_name in DICTS_TO_TRY:
                corners, ids = _try_dict(dict_name, grey, params)
                if ids:
                    cv2.aruco.drawDetectedMarkers(annotated, corners)
                    for corner, tag_id in zip(corners, ids):
                        cx = int(corner[0, :, 0].mean())
                        cy = int(corner[0, :, 1].mean())
                        cv2.putText(annotated, f"{dict_name} id={tag_id}",
                                    (cx, cy - 10), cv2.FONT_HERSHEY_SIMPLEX,
                                    0.6, (0, 255, 0), 2)
            # Also save raw frame for manual inspection.
            raw_path = out_dir / f"frame_{frame_idx:02d}_raw.jpg"
            ann_path = out_dir / f"frame_{frame_idx:02d}_annotated.jpg"
            cv2.imwrite(str(raw_path), bgr)
            cv2.imwrite(str(ann_path), annotated)
            best_frame_path = raw_path
            log.info("  → Saved best frame: %s", raw_path)

        # Also save the first frame always so we can see what the camera sees.
        if frame_idx == 0:
            first_path = out_dir / "frame_00_first.jpg"
            cv2.imwrite(str(first_path), bgr)
            log.info("  → Saved first frame: %s", first_path)
            # Print image stats.
            log.info("  Image shape=%s dtype=%s  brightness_mean=%.1f  brightness_std=%.1f",
                     bgr.shape, bgr.dtype,
                     grey.mean(), grey.std())

    device.stop()
    device.close()

    log.info("")
    log.info("=" * 70)
    log.info("SUMMARY (%d frames)", NUM_FRAMES)
    log.info("=" * 70)
    if not all_results:
        log.warning("NO TAGS DETECTED IN ANY FRAME WITH ANY DICTIONARY")
        log.warning("Possible causes:")
        log.warning("  1. Tag is not facing the camera cleanly")
        log.warning("  2. Image is blurry or overexposed — check saved frame")
        log.warning("  3. Tag was printed with different parameters or wrong dict")
        log.warning("  4. White border ('quiet zone') missing around tag")
        log.warning("  5. Kinect color stream orientation — may need rotate/flip")
    else:
        for dict_name, found_ids in sorted(all_results.items()):
            unique = sorted(set(found_ids))
            log.info("  %-30s detected %d times  unique IDs: %s",
                     dict_name, len(found_ids), unique)
    log.info("")
    log.info("Saved frames to: %s", out_dir.resolve())
    if best_frame_path:
        log.info("Best frame (most detections): %s", best_frame_path.resolve())


if __name__ == "__main__":
    main()
