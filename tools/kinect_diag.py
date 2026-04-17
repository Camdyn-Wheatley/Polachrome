"""
Quick synchronous diagnostic — no threading, no viewer.
Tests whether waitForNewFrame receives any frames at all.
Run with: python tools/kinect_diag.py
"""
import sys
import time
import logging

logging.basicConfig(level=logging.DEBUG, format="%(asctime)s %(levelname)s: %(message)s")
log = logging.getLogger("kinect_diag")

from pylibfreenect2 import (
    Freenect2, SyncMultiFrameListener, FrameType, Registration, Frame
)

def try_pipeline(pipeline_name):
    log.info("=" * 60)
    log.info("Testing pipeline: %s", pipeline_name)

    if pipeline_name == "opengl":
        from pylibfreenect2 import OpenGLPacketPipeline
        pipeline = OpenGLPacketPipeline()
    elif pipeline_name == "cpu":
        from pylibfreenect2 import CpuPacketPipeline
        pipeline = CpuPacketPipeline()
    else:
        raise ValueError(pipeline_name)

    fn = Freenect2()
    n = fn.enumerateDevices()
    log.info("Devices found: %d", n)
    if n == 0:
        log.error("No Kinect V2 found!")
        return

    serial = fn.getDeviceSerialNumber(0)
    log.info("Serial: %s", serial)

    device = fn.openDevice(serial, pipeline=pipeline)

    # Try depth+IR only first (simpler — no color)
    listener = SyncMultiFrameListener(FrameType.Depth | FrameType.Ir)
    device.setIrAndDepthFrameListener(listener)
    device.start()

    log.info("Device started. Waiting for frames (5s timeout each)...")

    for attempt in range(3):
        log.info("  waitForNewFrame attempt %d ...", attempt + 1)
        t0 = time.time()
        frames = listener.waitForNewFrame(milliseconds=5000)
        elapsed = time.time() - t0
        log.info("  returned after %.2fs, frames=%r, type=%s", elapsed, frames, type(frames))

        if frames is None:
            log.warning("  TIMEOUT — no frames received")
            continue

        # Inspect what keys are available
        for key in ("color", "ir", "depth"):
            try:
                present = key in frames
                log.info("    '%s' in frames: %s", key, present)
            except Exception as e:
                log.info("    '%s' in frames: ERROR %s", key, e)

        try:
            depth = frames["depth"]
            arr = depth.asarray()
            log.info("  depth shape=%s dtype=%s min=%.1f max=%.1f",
                     arr.shape, arr.dtype, arr.min(), arr.max())
        except Exception as e:
            log.error("  depth access failed: %s", e)

        listener.release(frames)
        log.info("  Frame released OK")
        break

    device.stop()
    device.close()
    log.info("Device closed.")


if __name__ == "__main__":
    # Try CPU first (no OpenGL conflicts), then OpenGL
    for name in ("cpu", "opengl"):
        try:
            try_pipeline(name)
        except Exception as exc:
            log.error("Pipeline %s failed: %s", name, exc)
        time.sleep(2)
