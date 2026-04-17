"""
kinect_stream.py — Threaded Kinect V2 frame acquisition.

Wraps pylibfreenect2 to provide thread-safe access to color, depth, IR,
and registered (color-mapped-to-depth) frames as numpy arrays ready for
OpenCV consumption.

The Kinect V2 hardware delivers:
  - Color:  1920×1080 @ 30fps  (BGRX uint8)
  - Depth:  512×424   @ 30fps  (float32, millimetres)
  - IR:     512×424   @ 30fps  (float32, raw intensity)

Usage::

    stream = KinectStream(pipeline="opengl")
    stream.start()
    try:
        while True:
            color = stream.read_color()
            depth = stream.read_depth()
            ir    = stream.read_ir()
            reg   = stream.read_registered()
            if color is None:
                continue
            # ... process frames ...
    finally:
        stream.stop()
"""

from __future__ import annotations

import logging
import threading
import time
from typing import Optional

import numpy as np

logger = logging.getLogger(__name__)

# pylibfreenect2 is imported lazily so the module can be imported (and tested
# for import errors) even on machines without a Kinect attached.
try:
    from pylibfreenect2 import (
        Freenect2,
        SyncMultiFrameListener,
        FrameType,
        Registration,
        Frame,
    )
    # Pipeline backends — tried in order of preference.
    from pylibfreenect2 import OpenGLPacketPipeline  # noqa: F401
    _HAVE_OPENGL = True
except ImportError:
    _HAVE_OPENGL = False

try:
    from pylibfreenect2 import OpenCLPacketPipeline  # noqa: F401
    _HAVE_OPENCL = True
except ImportError:
    _HAVE_OPENCL = False

try:
    from pylibfreenect2 import CpuPacketPipeline  # noqa: F401
    _HAVE_CPU = True
except ImportError:
    _HAVE_CPU = False

try:
    from pylibfreenect2 import Freenect2 as _Fn2Check  # noqa: F811
    _HAVE_FREENECT2 = True
except ImportError:
    _HAVE_FREENECT2 = False


# Kinect V2 sensor resolutions (fixed by hardware).
DEPTH_WIDTH = 512
DEPTH_HEIGHT = 424
COLOR_WIDTH = 1920
COLOR_HEIGHT = 1080


def _select_pipeline(name: str):
    """Return a pipeline instance for the requested backend."""
    name = name.lower().strip()
    if name == "opengl":
        if not _HAVE_OPENGL:
            raise RuntimeError("OpenGL pipeline not available in pylibfreenect2")
        return OpenGLPacketPipeline()
    if name == "opencl":
        if not _HAVE_OPENCL:
            raise RuntimeError("OpenCL pipeline not available in pylibfreenect2")
        return OpenCLPacketPipeline()
    if name == "cpu":
        if not _HAVE_CPU:
            raise RuntimeError("CPU pipeline not available in pylibfreenect2")
        return CpuPacketPipeline()
    raise ValueError(f"Unknown pipeline: {name!r}. Use 'opengl', 'opencl', or 'cpu'.")


class KinectStream:
    """Thread-safe wrapper around a Kinect V2 device.

    Parameters
    ----------
    pipeline : str
        Processing backend — ``"opengl"`` (fastest), ``"opencl"``, or ``"cpu"``.
    enable_color : bool
        Whether to capture color frames.
    enable_depth : bool
        Whether to capture depth + IR frames.
    enable_registered : bool
        Whether to produce registered (color-mapped-to-depth) frames.
        Requires both color and depth to be enabled.
    """

    def __init__(
        self,
        pipeline: str = "opengl",
        enable_color: bool = True,
        enable_depth: bool = True,
        enable_registered: bool = True,
        flip_horizontal: bool = False,
    ) -> None:
        if not _HAVE_FREENECT2:
            raise RuntimeError(
                "pylibfreenect2 is not installed. "
                "Install libfreenect2 and then: pip install pylibfreenect2"
            )

        self._pipeline_name = pipeline
        self._enable_color = enable_color
        self._enable_depth = enable_depth
        self._enable_registered = enable_registered and enable_color and enable_depth

        self._fn: Optional[Freenect2] = None
        self._device = None
        self._listener = None
        self._registration = None
        self._undistorted: Optional[Frame] = None
        self._registered_frame: Optional[Frame] = None

        # Latest frames, protected by lock.
        self._lock = threading.Lock()
        self._color: Optional[np.ndarray] = None
        self._depth: Optional[np.ndarray] = None
        self._ir: Optional[np.ndarray] = None
        self._registered: Optional[np.ndarray] = None

        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._started = False
        self._flip_horizontal = flip_horizontal

    # ── Lifecycle ────────────────────────────────────────────────────────

    def start(self) -> None:
        """Open the Kinect device and begin capturing frames."""
        if self._started:
            logger.warning("KinectStream already started")
            return

        self._fn = Freenect2()
        num_devices = self._fn.enumerateDevices()
        if num_devices == 0:
            raise RuntimeError("No Kinect V2 devices found")

        serial = self._fn.getDeviceSerialNumber(0)
        logger.info("Opening Kinect V2  serial=%s  pipeline=%s", serial, self._pipeline_name)

        pipeline = _select_pipeline(self._pipeline_name)
        self._device = self._fn.openDevice(serial, pipeline=pipeline)

        # Build the frame type mask.
        frame_types = 0
        if self._enable_color:
            frame_types |= FrameType.Color
        if self._enable_depth:
            frame_types |= (FrameType.Depth | FrameType.Ir)
        if frame_types == 0:
            raise ValueError("At least one of enable_color or enable_depth must be True")

        self._listener = SyncMultiFrameListener(frame_types)
        if self._enable_color:
            self._device.setColorFrameListener(self._listener)
        if self._enable_depth:
            self._device.setIrAndDepthFrameListener(self._listener)

        self._device.start()

        # Registration for mapping color → depth coordinate space.
        if self._enable_registered:
            self._registration = Registration(
                self._device.getIrCameraParams(),
                self._device.getColorCameraParams(),
            )
            self._undistorted = Frame(DEPTH_WIDTH, DEPTH_HEIGHT, 4)
            self._registered_frame = Frame(DEPTH_WIDTH, DEPTH_HEIGHT, 4)

        self._stop_event.clear()
        self._thread = threading.Thread(target=self._capture_loop, daemon=True, name="kinect-capture")
        self._thread.start()

        # Wait for first frames.
        deadline = time.time() + 10.0
        while self._color is None and self._depth is None:
            if time.time() > deadline:
                self.stop()
                raise RuntimeError("Kinect V2 timed out — no frames received in 10 s")
            time.sleep(0.05)

        self._started = True
        logger.info(
            "Kinect V2 ready  color=%s  depth=%s  registered=%s",
            self._enable_color,
            self._enable_depth,
            self._enable_registered,
        )

    def stop(self) -> None:
        """Stop capturing and release the device."""
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=3.0)
            self._thread = None
        if self._device is not None:
            self._device.stop()
            self._device.close()
            self._device = None
        self._started = False
        logger.info("Kinect V2 released")

    # ── Frame accessors ─────────────────────────────────────────────────

    def read_color(self) -> Optional[np.ndarray]:
        """Return the latest color frame as a BGR ``(1080, 1920, 3)`` uint8 array.

        Returns ``None`` if no frame is available yet or color is disabled.
        """
        with self._lock:
            return self._color.copy() if self._color is not None else None

    def read_depth(self) -> Optional[np.ndarray]:
        """Return the latest depth frame as a ``(424, 512)`` float32 array (millimetres).

        Returns ``None`` if no frame is available yet or depth is disabled.
        """
        with self._lock:
            return self._depth.copy() if self._depth is not None else None

    def read_ir(self) -> Optional[np.ndarray]:
        """Return the latest IR frame as a ``(424, 512)`` float32 array.

        Returns ``None`` if no frame is available yet or depth is disabled.
        """
        with self._lock:
            return self._ir.copy() if self._ir is not None else None

    def read_registered(self) -> Optional[np.ndarray]:
        """Return the latest registered frame as a BGR ``(424, 512, 3)`` uint8 array.

        The registered frame is the color image mapped into the depth camera's
        coordinate space, allowing pixel-by-pixel colour-depth correspondence.

        Returns ``None`` if registration is disabled or no frame is available.
        """
        with self._lock:
            return self._registered.copy() if self._registered is not None else None

    @property
    def ir_camera_params(self):
        """Return the Kinect's IR camera intrinsic parameters.

        Useful for ArUco pose estimation (building a camera matrix).
        """
        if self._device is None:
            return None
        return self._device.getIrCameraParams()

    @property
    def color_camera_params(self):
        """Return the Kinect's color camera intrinsic parameters."""
        if self._device is None:
            return None
        return self._device.getColorCameraParams()

    # ── Internal capture loop ───────────────────────────────────────────

    def _capture_loop(self) -> None:
        """Continuously read frames from the Kinect and store the latest.

        pylibfreenect2 FrameMap API quirks (version 0.1.4):
        - waitForNewFrame(frame_map=None, milliseconds=-1)
          → first positional arg is frame_map, use milliseconds as keyword.
        - FrameMap does NOT support __contains__ ('x in frames' raises 'Not supported').
        - Access frames by FrameType enum: frames[FrameType.Color] etc.
        - String key access frames["color"] works for item get but not 'in' check.
        - On timeout, waitForNewFrame returns None (does not raise).
        """
        while not self._stop_event.is_set():
            try:
                # IMPORTANT: milliseconds is the SECOND parameter (after frame_map).
                # A bare positional int raises TypeError (expects FrameMap, got int).
                frames = self._listener.waitForNewFrame(milliseconds=1000)
            except Exception as exc:
                logger.debug("waitForNewFrame exception: %s", exc)
                continue

            if frames is None:
                continue

            color_arr = None
            depth_arr = None
            ir_arr = None
            reg_arr = None

            try:
                # FrameMap.__contains__ raises 'Not supported' — access by
                # FrameType enum key directly and catch KeyError if missing.
                if self._enable_color:
                    try:
                        raw = frames[FrameType.Color]
                        # Kinect color is BGRX (4-channel); drop the X channel.
                        color_arr = raw.asarray(np.uint8)[:, :, :3].copy()
                    except KeyError:
                        pass

                if self._enable_depth:
                    try:
                        depth_arr = frames[FrameType.Depth].asarray(np.float32).copy()
                    except KeyError:
                        pass
                    try:
                        ir_arr = frames[FrameType.Ir].asarray(np.float32).copy()
                    except KeyError:
                        pass

                if self._enable_registered and self._registration is not None:
                    if color_arr is not None and depth_arr is not None:
                        self._registration.apply(
                            frames[FrameType.Color],
                            frames[FrameType.Depth],
                            self._undistorted,
                            self._registered_frame,
                        )
                        reg_arr = self._registered_frame.asarray(np.uint8)[:, :, :3].copy()
            finally:
                self._listener.release(frames)

            with self._lock:
                if self._flip_horizontal:
                    import cv2  # local import — cv2 is a viewer dep, avoid top-level coupling
                    if color_arr is not None:
                        color_arr = cv2.flip(color_arr, 1)
                    if depth_arr is not None:
                        depth_arr = cv2.flip(depth_arr, 1)
                    if ir_arr is not None:
                        ir_arr = cv2.flip(ir_arr, 1)
                    if reg_arr is not None:
                        reg_arr = cv2.flip(reg_arr, 1)
                if color_arr is not None:
                    self._color = color_arr
                if depth_arr is not None:
                    self._depth = depth_arr
                if ir_arr is not None:
                    self._ir = ir_arr
                if reg_arr is not None:
                    self._registered = reg_arr
