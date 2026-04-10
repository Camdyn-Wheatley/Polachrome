"""
robot_brain.py  —  Autonomous Combat Robot Brain
==================================================
Extends unified_tracking.py with:
  • Bézier path planning: Robot1 lines up on knock axis, drives through Robot2 into Tag target
  • Bounds box with robot-size offset (keeps robot inside arena)
  • Evasion from Robot2's front (velocity-direction threat avoidance)
  • Target switching between AprilTag ID 3 and ID 4
  • Serial output to Arduino Nano (CH1=steer, CH2=drive, CH3=weapon)
  • Adaptive color re-sampling for Robot1 fallback tracking

Hardware
--------
  Arduino Nano connected via USB serial running arduino_trainer.ino (PPM output).
  FS-i6X trainer port wired to Nano D2 (PPM signal) and GND.

Controls (keyboard on Tracker window)
--------------------------------------
  Q          quit
  T          toggle target tag (3 ↔ 4)
  A          toggle autonomous mode on/off
  B          toggle arena bounds-box draw mode
  +/-        increase / decrease robot offset (collision box size)
  Click      pick Robot2 color

Serial protocol to Arduino (9600 baud, single-character commands):
  w/s = drive forward/back (CH2 ± STEP)
  a/d = steer left/right   (CH1 ± STEP)
  x/z = weapon on/off      (CH3 = 2000/1000)
  c   = center steer+drive
  ' ' = emergency stop (all neutral + weapon off)

PATH GEOMETRY
-------------
  push_unit  = normalise(Tag3 − R2)        ← direction FROM R2 TOWARD Tag3
  entry_pt   = R2 − push_unit * STANDOFF   ← R1 lines up here (opposite side from Tag3)
  R1 travels entry_pt → R2, which sends R2 in direction push_unit toward Tag3.
  The Bézier departs R1 along its current heading and arrives at entry_pt
  aimed along −push_unit (directly at R2).
"""

import csv
import glob
import json
import math
import os
os.environ.setdefault("QT_STYLE_OVERRIDE", "Fusion")
import signal
import threading
import time
from datetime import datetime
from collections import deque
from typing import Optional, Tuple

import cv2
import numpy as np

try:
    import serial
    import serial.tools.list_ports
    _HAVE_SERIAL = True
except ImportError:
    _HAVE_SERIAL = False
    print("[WARN] pyserial not installed — Arduino output disabled. pip install pyserial")

try:
    import pupil_apriltags as _at
    def _parse_det(det):
        return det.corners.astype(np.float64), det.center, det.tag_id
except ImportError:
    raise SystemExit("Install pupil-apriltags:  pip install pupil-apriltags")


# ──────────────────────────────────────────────────────────────────────────────
# CONFIG
# ──────────────────────────────────────────────────────────────────────────────
CAMERA_DEVICE        = "/dev/video1"
FRAME_WIDTH          = 1280
FRAME_HEIGHT         = 720

TAG_FAMILY           = "tag25h9"
TAG_SIZE             = 0.03302
ROBOT1_TAG_IDS       = {0, 1}           # T0 = right side, T1 = left side
QUAD_DECIMATE        = 2.0
DETECTOR_THREADS     = 2

ROI_PADDING          = 150
ROI_REACQUIRE_FRAMES = 6

PROFILE_FILE         = "color_profile.json"
MIN_CONTOUR_AREA     = 500
DEFAULT_TOL          = {"h": 15, "s": 60, "v": 60}

MAX_COAST_FRAMES     = 30
EXPOSURE             = 150
POSITION_LOG         = "positions.csv"

ARENA_TL             = (50,  50)
ARENA_BR             = (1230, 670)
ROBOT_OFFSET         = 40
PATH_REPLAN_DIST     = 20
LOOKAHEAD_DIST       = 80
MAX_DRIVE_SPEED      = 400
MAX_TURN_SPEED       = 400
WEAPON_ON_VALUE      = 2000
WEAPON_OFF_VALUE     = 1000

# How far behind R2 (away from Tag) to place the alignment point
APPROACH_STANDOFF    = 80              # pixels

THREAT_CONE_DEG      = 40.0
EVASION_OFFSET       = 120
TARGET_TAG_A         = 3
TARGET_TAG_B         = 4
ARDUINO_PORT         = None
ARDUINO_BAUD         = 9600
ARDUINO_STEP         = 5       # PWM µs per char command
CHANNEL_REVERSE_STEER  = False
CHANNEL_REVERSE_DRIVE  = True   # robot drives backwards by default
CHANNEL_REVERSE_WEAPON = False
COLOR_RESAMPLE_INTERVAL = 60
COLOR_SAMPLE_RADIUS     = 12

CAMERA_MATRIX = np.array([
    [FRAME_WIDTH,           0, FRAME_WIDTH  / 2.0],
    [          0, FRAME_WIDTH, FRAME_HEIGHT / 2.0],
    [          0,           0,               1.0 ],
], dtype=np.float64)
DIST_COEFFS = np.zeros((4, 1))

_h_tag = TAG_SIZE / 2.0
TAG_POINTS_3D = np.array([
    [-_h_tag,  _h_tag, 0], [ _h_tag,  _h_tag, 0],
    [ _h_tag, -_h_tag, 0], [-_h_tag, -_h_tag, 0],
], dtype=np.float64)


# ──────────────────────────────────────────────────────────────────────────────
# GLOBAL STATE
# ──────────────────────────────────────────────────────────────────────────────
_AVAILABLE_CAMERAS = []
_CURRENT_CAM_INDEX = 0
_CAM_SWITCH_PENDING = False

path_draw_mode = False
custom_drawn_path = []
is_drawing = False

def scan_cameras():
    global _AVAILABLE_CAMERAS, _CURRENT_CAM_INDEX
    _AVAILABLE_CAMERAS = []
    print("[CAM] Scanning for valid video devices...")
    for path in sorted(glob.glob("/dev/video*")):
        cap = cv2.VideoCapture(path, cv2.CAP_V4L2)
        if cap.isOpened():
            _AVAILABLE_CAMERAS.append(path)
            cap.release()
    if not _AVAILABLE_CAMERAS:
         _AVAILABLE_CAMERAS = [CAMERA_DEVICE]
    try:
        _CURRENT_CAM_INDEX = _AVAILABLE_CAMERAS.index(CAMERA_DEVICE)
    except ValueError:
        _CURRENT_CAM_INDEX = 0
    print(f"[CAM] Candidates: {_AVAILABLE_CAMERAS}")

# ──────────────────────────────────────────────────────────────────────────────
# THREADED CAMERA
# ──────────────────────────────────────────────────────────────────────────────
class CameraStream:
    def __init__(self, device, width, height):
        self.cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self.cap = cv2.VideoCapture(device)
        if not self.cap.isOpened():
            raise RuntimeError(f"Cannot open camera: {device}")
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE,   1)
        self.cap.set(cv2.CAP_PROP_FOURCC,       cv2.VideoWriter_fourcc(*"MJPG"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS,          60)
        self._configure_exposure()
        self._frame  = None
        self._lock   = threading.Lock()
        self._stop   = False
        self._thread = threading.Thread(target=self._grab, daemon=True)
        self._thread.start()
        deadline = time.time() + 5.0
        while self._frame is None:
            if time.time() > deadline:
                self._stop = True
                self.cap.release()
                raise RuntimeError("Camera timed out — no frames received.")
            time.sleep(0.02)
        w   = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h   = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = self.cap.get(cv2.CAP_PROP_FPS)
        print(f"[CAM] Ready  {w}x{h}  @ {fps:.0f}fps")

    def _configure_exposure(self):
        if EXPOSURE is None:
            self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3)
        else:
            self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
            self.cap.set(cv2.CAP_PROP_EXPOSURE, EXPOSURE)

    def _grab(self):
        while not self._stop:
            ret, frame = self.cap.read()
            if ret:
                with self._lock:
                    self._frame = frame

    def read(self):
        with self._lock:
            return self._frame.copy() if self._frame is not None else None

    def get(self, prop):      return self.cap.get(prop)
    def set(self, prop, val): self.cap.set(prop, val)

    def release(self):
        self._stop = True
        self._thread.join(timeout=1.0)
        self.cap.release()


# ──────────────────────────────────────────────────────────────────────────────
# KALMAN TRACKER  (state: x y vx vy ax ay)
# ──────────────────────────────────────────────────────────────────────────────
class KalmanTracker:
    def __init__(self, proc_noise=0.5, meas_noise=4.0):
        self.kf = cv2.KalmanFilter(6, 2)
        self.kf.measurementMatrix               = np.zeros((2, 6), np.float32)
        self.kf.measurementMatrix[0, 0]         = 1.0
        self.kf.measurementMatrix[1, 1]         = 1.0
        self.kf.processNoiseCov                 = np.eye(6, dtype=np.float32) * proc_noise
        self.kf.measurementNoiseCov             = np.eye(2, dtype=np.float32) * meas_noise
        self.kf.errorCovPost                    = np.eye(6, dtype=np.float32) * 500.0
        self._init = False
        self.coast = 0

    def _set_dt(self, dt):
        d2 = 0.5 * dt * dt
        self.kf.transitionMatrix = np.array([
            [1, 0, dt, 0,  d2, 0 ],
            [0, 1, 0,  dt, 0,  d2],
            [0, 0, 1,  0,  dt, 0 ],
            [0, 0, 0,  1,  0,  dt],
            [0, 0, 0,  0,  1,  0 ],
            [0, 0, 0,  0,  0,  1 ],
        ], np.float32)

    def update(self, x, y, dt):
        self._set_dt(dt)
        if not self._init:
            self.kf.statePost = np.array([x, y, 0, 0, 0, 0], np.float32).reshape(6, 1)
            self._init = True
        self.kf.correct(np.array([[x], [y]], np.float32))
        p = self.kf.predict()
        self.coast = 0
        return (int(p[0]), int(p[1]))

    def predict(self, dt):
        if not self._init:
            return None
        self.coast += 1
        if self.coast > MAX_COAST_FRAMES:
            return None
        self._set_dt(dt)
        p = self.kf.predict()
        return (int(p[0]), int(p[1]))

    @property
    def coasting(self):
        return self._init and 0 < self.coast <= MAX_COAST_FRAMES

    @property
    def coast_ratio(self):
        return min(self.coast / MAX_COAST_FRAMES, 1.0)

    def velocity(self) -> Tuple[float, float]:
        s = self.kf.statePost
        return float(s[2]), float(s[3])

    def position_float(self) -> Optional[Tuple[float, float]]:
        if not self._init:
            return None
        s = self.kf.statePost
        return float(s[0]), float(s[1])


kf1 = KalmanTracker(proc_noise=0.5, meas_noise=4.0)
kf2 = KalmanTracker(proc_noise=1.0, meas_noise=8.0)


# ──────────────────────────────────────────────────────────────────────────────
# ARDUINO SERIAL OUTPUT
# ──────────────────────────────────────────────────────────────────────────────
class ArduinoSerial:
    """Non-blocking serial writer using single-character protocol."""

    def __init__(self, port=None, baud=9600):
        self._ser   = None
        self._lock  = threading.Lock()
        self._queue = deque(maxlen=1)
        self._stop  = False
        self._neutral = 1486
        self._step  = ARDUINO_STEP
        self._ch1   = self._neutral
        self._ch2   = self._neutral
        self._ch3   = WEAPON_OFF_VALUE

        if not _HAVE_SERIAL:
            print("[ARDUINO] pyserial missing — serial output disabled.")
            return

        if port is None:
            ports = [p.device for p in serial.tools.list_ports.comports()]
            port  = next((p for p in ports
                          if any(x in p for x in ("USB", "ACM", "COM"))), None)

        if port is None:
            print("[ARDUINO] No port found — serial output disabled.")
            return

        try:
            self._ser = serial.Serial(port, baud, timeout=0.1)
            time.sleep(2.0)
            print(f"[ARDUINO] Connected on {port} @ {baud}")
        except serial.SerialException as e:
            print(f"[ARDUINO] Could not open {port}: {e}")
            self._ser = None
            return

        self._thread = threading.Thread(target=self._sender, daemon=True)
        self._thread.start()

    def _sender(self):
        while not self._stop:
            msg = None
            with self._lock:
                if self._queue:
                    msg = self._queue.pop()
            if msg and self._ser and self._ser.is_open:
                try:
                    self._ser.write(msg)
                except serial.SerialException:
                    pass
            time.sleep(0.02)

    def _apply_reversal(self, ch1, ch2, ch3):
        if CHANNEL_REVERSE_STEER:
            ch1 = 2 * self._neutral - ch1
        if CHANNEL_REVERSE_DRIVE:
            ch2 = 2 * self._neutral - ch2
        if CHANNEL_REVERSE_WEAPON:
            ch3 = 2 * self._neutral - ch3
        return (max(1000, min(2000, ch1)),
                max(1000, min(2000, ch2)),
                max(1000, min(2000, ch3)))

    def send(self, ch1: int, ch2: int, ch3: int):
        ch1 = max(1000, min(2000, int(ch1)))
        ch2 = max(1000, min(2000, int(ch2)))
        ch3 = max(1000, min(2000, int(ch3)))
        ch1, ch2, ch3 = self._apply_reversal(ch1, ch2, ch3)

        chars = []
        # Steer
        diff1 = ch1 - self._ch1
        if abs(diff1) >= self._step:
            n = min(abs(diff1) // self._step, 8)
            chars.extend(["d" if diff1 > 0 else "a"] * n)
            self._ch1 += n * self._step * (1 if diff1 > 0 else -1)
            self._ch1 = max(1000, min(2000, self._ch1))
        # Drive
        diff2 = ch2 - self._ch2
        if abs(diff2) >= self._step:
            n = min(abs(diff2) // self._step, 8)
            chars.extend(["w" if diff2 > 0 else "s"] * n)
            self._ch2 += n * self._step * (1 if diff2 > 0 else -1)
            self._ch2 = max(1000, min(2000, self._ch2))
        # Weapon
        if ch3 != self._ch3:
            chars.append("x" if ch3 >= 1500 else "z")
            self._ch3 = ch3
        # Snap neutral
        if (ch1 == self._neutral and ch2 == self._neutral
                and abs(self._ch1 - self._neutral) < self._step * 2
                and abs(self._ch2 - self._neutral) < self._step * 2):
            chars = ["c"]
            self._ch1 = self._neutral
            self._ch2 = self._neutral

        if chars:
            with self._lock:
                self._queue.append("".join(chars).encode())

    def send_neutral(self):
        with self._lock:
            self._queue.append(b" ")
        self._ch1 = self._neutral
        self._ch2 = self._neutral
        self._ch3 = WEAPON_OFF_VALUE

    def close(self):
        self._stop = True
        if self._ser and self._ser.is_open:
            self.send_neutral()
            time.sleep(0.15)
            self._ser.close()


# ──────────────────────────────────────────────────────────────────────────────
# ARENA BOUNDS
# ──────────────────────────────────────────────────────────────────────────────
class ArenaBounds:
    def __init__(self, tl, br, robot_offset):
        self.tl_raw = tl
        self.br_raw = br
        self.offset = robot_offset
        self._recompute()

    def _recompute(self):
        self.safe_tl = (self.tl_raw[0] + self.offset, self.tl_raw[1] + self.offset)
        self.safe_br = (self.br_raw[0] - self.offset, self.br_raw[1] - self.offset)

    def clamp(self, pt: Tuple[float, float]) -> Tuple[float, float]:
        x = max(self.safe_tl[0], min(self.safe_br[0], pt[0]))
        y = max(self.safe_tl[1], min(self.safe_br[1], pt[1]))
        return (x, y)

    def inside(self, pt) -> bool:
        return (self.safe_tl[0] <= pt[0] <= self.safe_br[0] and
                self.safe_tl[1] <= pt[1] <= self.safe_br[1])

    def change_offset(self, delta):
        self.offset = max(0, self.offset + delta)
        self._recompute()
        print(f"[BOUNDS] Robot offset = {self.offset}px")

    def draw(self, frame):
        cv2.rectangle(frame, self.tl_raw, self.br_raw, (255, 255, 255), 2)
        _draw_dashed_rect(frame, self.safe_tl, self.safe_br, (255, 220, 0), 1)
        cv2.putText(frame, f"safe zone  offset={self.offset}px",
                    (self.safe_tl[0] + 4, self.safe_tl[1] + 16),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 220, 0), 1)


def _draw_dashed_rect(frame, tl, br, color, thickness, dash=12):
    pts = [(tl[0], tl[1]), (br[0], tl[1]),
           (br[0], br[1]), (tl[0], br[1]), (tl[0], tl[1])]
    for i in range(len(pts) - 1):
        x0, y0 = pts[i];  x1, y1 = pts[i + 1]
        dx, dy = x1 - x0, y1 - y0
        length = math.hypot(dx, dy)
        if length < 1:
            continue
        steps = int(length / (dash * 2))
        for s in range(steps):
            t0 = (2 * s    ) * dash / length
            t1 = (2 * s + 1) * dash / length
            cv2.line(frame,
                     (int(x0 + dx * t0), int(y0 + dy * t0)),
                     (int(x0 + dx * t1), int(y0 + dy * t1)),
                     color, thickness)


arena = ArenaBounds(ARENA_TL, ARENA_BR, ROBOT_OFFSET)


# ──────────────────────────────────────────────────────────────────────────────
# BÉZIER PATH PLANNER
# ──────────────────────────────────────────────────────────────────────────────
def _cubic_bezier(p0, p1, p2, p3, n=60) -> np.ndarray:
    t   = np.linspace(0, 1, n).reshape(-1, 1)
    mt  = 1 - t
    return (mt**3 * np.array(p0) +
            3 * mt**2 * t * np.array(p1) +
            3 * mt * t**2 * np.array(p2) +
            t**3 * np.array(p3))


def build_path(r1: Tuple, r2: Tuple, target: Tuple,
               bounds: ArenaBounds,
               r1_heading_deg: float = 0.0) -> np.ndarray:
    """
    Build a knock-through path from R1's current position to R2,
    aligned so that R2 is knocked toward the target tag.

    No velocity prediction — paths are based on current positions only
    and re-planned every frame R1 or R2 moves > PATH_REPLAN_DIST pixels.

    Geometry:
      push_unit  = normalise(target − R2)     direction R2 must travel after impact
      entry_pt   = R2 − push_unit * STANDOFF  where R1 must be just before impact
      Bézier departs R1 along its heading, curves to arrive at entry_pt
      aimed along −push_unit (straight at R2).
      A short straight segment then carries R1 all the way through R2.
    """
    r1_f  = np.array(r1,     dtype=float)
    r2_f  = np.array(r2,     dtype=float)
    tgt_f = np.array(target, dtype=float)

    # Direction R2 must travel to reach the target (push direction)
    push_vec  = tgt_f - r2_f
    push_len  = np.linalg.norm(push_vec)
    push_unit = push_vec / push_len if push_len > 1e-3 else np.array([1.0, 0.0])

    # entry_pt: on the far side of R2 from the target tag
    entry_pt = r2_f - push_unit * APPROACH_STANDOFF

    # Clamp into safe arena
    r1_f     = np.array(bounds.clamp(tuple(r1_f)))
    entry_pt = np.array(bounds.clamp(tuple(entry_pt)))
    r2_clamped = np.array(bounds.clamp(tuple(r2_f)))

    # R1 forward unit vector from its heading
    h_rad    = math.radians(r1_heading_deg)
    fwd_unit = np.array([math.cos(h_rad), math.sin(h_rad)])

    chord  = entry_pt - r1_f
    length = np.linalg.norm(chord)

    if length > 1e-3:
        # C1: depart along R1's current heading
        c1 = r1_f + fwd_unit * min(length * 0.45, 220.0)
        # C2: arrive at entry_pt from the push direction so the last
        #     section of the curve is aimed straight at R2
        c2 = entry_pt - push_unit * min(length * 0.35, 180.0)
        c1 = np.array(bounds.clamp(tuple(c1)))
        c2 = np.array(bounds.clamp(tuple(c2)))
    else:
        c1 = r1_f.copy()
        c2 = entry_pt.copy()

    curve    = _cubic_bezier(r1_f, c1, c2, entry_pt, n=55)
    # Short straight segment that drives R1 all the way through R2
    straight = np.linspace(entry_pt, r2_clamped, 10)[1:]
    return np.vstack([curve, straight])


def build_evasion_path(r1: Tuple, evade_pt: Tuple,
                       bounds: ArenaBounds,
                       r1_heading_deg: float = 0.0) -> np.ndarray:
    """
    Simple single-segment Bézier from R1 to the evasion waypoint.
    Separate from build_path so it never receives a zero push_vec.
    """
    r1_f  = np.array(r1,      dtype=float)
    tgt_f = np.array(evade_pt, dtype=float)

    r1_f  = np.array(bounds.clamp(tuple(r1_f)))
    tgt_f = np.array(bounds.clamp(tuple(tgt_f)))

    chord  = tgt_f - r1_f
    length = np.linalg.norm(chord)

    h_rad    = math.radians(r1_heading_deg)
    fwd_unit = np.array([math.cos(h_rad), math.sin(h_rad)])

    if length > 1e-3:
        c1 = r1_f  + fwd_unit * min(length * 0.4, 160.0)
        c2 = tgt_f - (chord / length) * min(length * 0.2, 80.0)
        c1 = np.array(bounds.clamp(tuple(c1)))
        c2 = np.array(bounds.clamp(tuple(c2)))
    else:
        c1 = r1_f.copy()
        c2 = tgt_f.copy()

    return _cubic_bezier(r1_f, c1, c2, tgt_f, n=30)


# ──────────────────────────────────────────────────────────────────────────────
# DRAWING — path & knock axis
# ──────────────────────────────────────────────────────────────────────────────
def draw_path(frame, path: Optional[np.ndarray], r2: Optional[Tuple],
              target: Optional[Tuple], target_id: int):
    if path is None or len(path) < 2:
        return

    # Approach curve — green → orange gradient
    pts = path.astype(np.int32)
    for i in range(len(pts) - 1):
        frac  = i / max(len(pts) - 2, 1)
        color = (0, int(200 * (1 - frac) + 100 * frac), int(255 * frac))
        cv2.line(frame, tuple(pts[i]), tuple(pts[i + 1]), color, 3)

    # Arrow at path end
    if len(pts) >= 4:
        cv2.arrowedLine(frame, tuple(pts[-4]), tuple(pts[-1]),
                        (0, 180, 255), 2, tipLength=0.4)

    # Knock-through axis (dashed red line: entry side → Tag)
    if r2 and target:
        r2_f  = np.array(r2,     dtype=float)
        tgt_f = np.array(target, dtype=float)
        push  = tgt_f - r2_f
        pl    = np.linalg.norm(push)
        if pl > 1e-3:
            unit    = push / pl
            # Extend line from behind R2 all the way to the tag
            behind  = r2_f - unit * APPROACH_STANDOFF * 1.2
            _draw_dashed_line(frame,
                              (int(behind[0]), int(behind[1])),
                              (int(tgt_f[0]),  int(tgt_f[1])),
                              (0, 0, 255), thickness=2, dash=14)
            # Arrow showing R2's knock direction
            cv2.arrowedLine(frame,
                            (int(r2_f[0]), int(r2_f[1])),
                            (int(tgt_f[0]), int(tgt_f[1])),
                            (0, 0, 255), 2, tipLength=0.18)

        cv2.circle(frame, (int(r2[0]), int(r2[1])), 12, (0, 100, 255), 2)
        cv2.putText(frame, "hit here",
                    (int(r2[0]) + 14, int(r2[1]) - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 100, 255), 1)

    if target:
        cv2.drawMarker(frame, (int(target[0]), int(target[1])),
                       (0, 0, 255), cv2.MARKER_STAR, 24, 2)
        cv2.putText(frame, f"TAG {target_id} (target)",
                    (int(target[0]) + 14, int(target[1]) - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)


def _draw_dashed_line(frame, p0, p1, color, thickness=1, dash=12):
    dx, dy = p1[0] - p0[0], p1[1] - p0[1]
    length = math.hypot(dx, dy)
    if length < 1:
        return
    steps = int(length / (dash * 2))
    for s in range(steps + 1):
        t0 = min((2 * s    ) * dash / length, 1.0)
        t1 = min((2 * s + 1) * dash / length, 1.0)
        cv2.line(frame,
                 (int(p0[0] + dx * t0), int(p0[1] + dy * t0)),
                 (int(p0[0] + dx * t1), int(p0[1] + dy * t1)),
                 color, thickness)


# ──────────────────────────────────────────────────────────────────────────────
# PURE-PURSUIT CONTROLLER
# ──────────────────────────────────────────────────────────────────────────────
def pure_pursuit(r1: Tuple, heading_deg: float,
                 path: np.ndarray,
                 lookahead: float = LOOKAHEAD_DIST) -> Tuple[int, int]:
    """Returns (drive_pwm, steer_pwm) each in [1000, 2000]."""
    if path is None or len(path) < 2:
        return 1500, 1500

    r1_f = np.array(r1, dtype=float)

    lookahead_pt = None
    for pt in path:
        if np.linalg.norm(pt - r1_f) >= lookahead:
            lookahead_pt = pt
            break
    if lookahead_pt is None:
        lookahead_pt = path[-1]

    diff = lookahead_pt - r1_f
    dist = np.linalg.norm(diff)
    if dist < 5:
        return 1500, 1500

    target_angle = math.degrees(math.atan2(diff[1], diff[0]))
    error_angle  = (target_angle - heading_deg + 360) % 360
    if error_angle > 180:
        error_angle -= 360

    turn  = int(error_angle / 180.0 * MAX_TURN_SPEED)
    steer = 1500 + max(-MAX_TURN_SPEED, min(MAX_TURN_SPEED, turn))

    speed_factor = max(0.2, 1.0 - abs(error_angle) / 120.0)
    drive = 1500 + max(-MAX_DRIVE_SPEED,
                       min(MAX_DRIVE_SPEED, int(MAX_DRIVE_SPEED * speed_factor)))
    return drive, steer


# ──────────────────────────────────────────────────────────────────────────────
# THREAT EVASION
# ──────────────────────────────────────────────────────────────────────────────
def evasion_offset(r1: Tuple, r2: Tuple,
                   r2_vel: Tuple[float, float]) -> Optional[Tuple]:
    """Return a perpendicular dodge point if R2 is heading toward R1."""
    vx, vy = r2_vel
    speed  = math.hypot(vx, vy)
    if speed < 1.5:
        return None

    dx, dy      = r1[0] - r2[0], r1[1] - r2[1]
    angle_to_r1 = math.degrees(math.atan2(dy, dx))
    vel_angle   = math.degrees(math.atan2(vy, vx))
    diff        = abs((angle_to_r1 - vel_angle + 360) % 360)
    if diff > 180:
        diff = 360 - diff
    if diff > THREAT_CONE_DEG:
        return None

    perp_angle = math.radians(vel_angle + 90)
    return arena.clamp((r1[0] + math.cos(perp_angle) * EVASION_OFFSET,
                        r1[1] + math.sin(perp_angle) * EVASION_OFFSET))


# ──────────────────────────────────────────────────────────────────────────────
# ROBOT1 HEADING ESTIMATION
# ──────────────────────────────────────────────────────────────────────────────
def _tag_forward_deg(info) -> Optional[float]:
    solve_pose(info)
    if info["rvec"] is None:
        return None
    R, _ = cv2.Rodrigues(info["rvec"])
    fwd_x, fwd_y = R[0, 1], R[1, 1]
    if math.hypot(fwd_x, fwd_y) < 1e-6:
        return None
    return math.degrees(math.atan2(fwd_y, fwd_x))


def estimate_heading(tags: dict, kf: KalmanTracker) -> float:
    r1_tag_ids = [i for i in (ROBOT1_TAG_IDS if ROBOT1_TAG_IDS else tags.keys())
                  if i in tags and i not in (TARGET_TAG_A, TARGET_TAG_B)]

    fwd_vectors = []
    for tid in r1_tag_ids:
        deg = _tag_forward_deg(tags[tid])
        if deg is not None:
            fwd_vectors.append((math.cos(math.radians(deg)),
                                math.sin(math.radians(deg))))
    if fwd_vectors:
        mx = sum(v[0] for v in fwd_vectors) / len(fwd_vectors)
        my = sum(v[1] for v in fwd_vectors) / len(fwd_vectors)
        return math.degrees(math.atan2(my, mx))

    vx, vy = kf.velocity()
    if math.hypot(vx, vy) > 0.5:
        return math.degrees(math.atan2(vy, vx))
    return 0.0


# ──────────────────────────────────────────────────────────────────────────────
# ROBOT1 ADAPTIVE COLOR TRACKER
# ──────────────────────────────────────────────────────────────────────────────
class Robot1ColorTracker:
    def __init__(self):
        self.hsv_center = None
        self.tol        = {"h": 18, "s": 70, "v": 70}
        self._cnt       = 0
        self._profile   = "robot1_color.json"
        self._load()

    def _load(self):
        if os.path.exists(self._profile):
            with open(self._profile) as f:
                d = json.load(f)
            self.hsv_center = tuple(d["center"])
            self.tol.update(d.get("tol", {}))
            print(f"[R1COLOR] Loaded: {self.hsv_center}")

    def save(self):
        if self.hsv_center is None:
            return
        with open(self._profile, "w") as f:
            json.dump({"center": list(self.hsv_center), "tol": self.tol}, f, indent=2)

    def maybe_resample(self, hsv_frame, r1_pos):
        self._cnt += 1
        if r1_pos is None or self._cnt % COLOR_RESAMPLE_INTERVAL != 0:
            return
        x, y   = r1_pos
        r      = COLOR_SAMPLE_RADIUS
        fh, fw = hsv_frame.shape[:2]
        patch  = hsv_frame[max(0, y-r):min(fh, y+r), max(0, x-r):min(fw, x+r)]
        if patch.size == 0:
            return
        h, s, v = (int(np.median(patch[:, :, c])) for c in range(3))
        if s > 40:
            self.hsv_center = (h, s, v)
            self.save()
            print(f"[R1COLOR] Resampled → HSV {self.hsv_center}")

    def detect(self, frame, hsv_frame):
        if self.hsv_center is None:
            return None
        h, s, v = self.hsv_center
        lo = np.array([max(0,   h - self.tol["h"]),
                       max(0,   s - self.tol["s"]),
                       max(0,   v - self.tol["v"])])
        hi = np.array([min(179, h + self.tol["h"]),
                       min(255, s + self.tol["s"]),
                       min(255, v + self.tol["v"])])
        if lo[0] > hi[0]:
            mask = cv2.bitwise_or(
                cv2.inRange(hsv_frame, np.array([0,   lo[1], lo[2]]), hi),
                cv2.inRange(hsv_frame, lo, np.array([179, hi[1], hi[2]])))
        else:
            mask = cv2.inRange(hsv_frame, lo, hi)
        k    = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
        ctrs, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not ctrs:
            return None
        best = max(ctrs, key=cv2.contourArea)
        if cv2.contourArea(best) < MIN_CONTOUR_AREA:
            return None
        M = cv2.moments(best)
        if M["m00"] == 0:
            return None
        return (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))


# ──────────────────────────────────────────────────────────────────────────────
# COLOR TRACKER  (Robot 2)
# ──────────────────────────────────────────────────────────────────────────────
hsv_lower         = np.array([0,   100, 100])
hsv_upper         = np.array([10,  255, 255])
tolerance_r2      = DEFAULT_TOL.copy()
picked_hsv        = None
current_hsv_frame = None
running           = True
TRACKBAR_WIN      = "Color Tuning"
_cam_ref          = None


def _clamp_v(v, lo, hi): return max(lo, min(hi, v))


def apply_tolerance_r2(h=None, s=None, v=None):
    global hsv_lower, hsv_upper
    if h is None:
        if picked_hsv is None: return
        h, s, v = picked_hsv
    hsv_lower = np.array([_clamp_v(h - tolerance_r2["h"], 0, 179),
                          _clamp_v(s - tolerance_r2["s"], 0, 255),
                          _clamp_v(v - tolerance_r2["v"], 0, 255)])
    hsv_upper = np.array([_clamp_v(h + tolerance_r2["h"], 0, 179),
                          _clamp_v(s + tolerance_r2["s"], 0, 255),
                          _clamp_v(v + tolerance_r2["v"], 0, 255)])


def save_profile():
    with open(PROFILE_FILE, "w") as f:
        json.dump({"lower": hsv_lower.tolist(), "upper": hsv_upper.tolist(),
                   "picked": list(picked_hsv) if picked_hsv else None,
                   "tolerance": tolerance_r2}, f, indent=2)
    print(f"[SAVED] {PROFILE_FILE}")


def load_profile():
    global hsv_lower, hsv_upper, picked_hsv, tolerance_r2
    if not os.path.exists(PROFILE_FILE): return
    with open(PROFILE_FILE) as f:
        d = json.load(f)
    hsv_lower  = np.array(d["lower"])
    hsv_upper  = np.array(d["upper"])
    picked_hsv = tuple(d["picked"]) if d.get("picked") else None
    tolerance_r2.update(d.get("tolerance", {}))
    print("[LOADED] R2 color profile.")
    _sync_trackbars()


_PANEL_W, _PANEL_H = 520, 340
_SLIDER_MARGIN = 20
_SLIDER_H = 28
_SLIDER_GAP = 8
_BAR_X = 180
_BAR_W = _PANEL_W - _BAR_X - _SLIDER_MARGIN

_SLIDERS = [
    ("H Center",     "hc",  0, 179),
    ("H Tolerance",  "ht",  1,  90),
    ("S Center",     "sc",  0, 255),
    ("S Tolerance",  "st",  1, 127),
    ("V Center",     "vc",  0, 255),
    ("V Tolerance",  "vt",  1, 127),
    ("Exposure(0=au)", "exp", 0, 500),
]

_tuning_values = {
    "hc": 90, "ht": 15,
    "sc": 150, "st": 60,
    "vc": 150, "vt": 60,
    "exp": 0 if EXPOSURE is None else int(EXPOSURE),
}
_tuning_dragging = None

def tuning_mouse_cb(event, x, y, flags, param):
    global _tuning_dragging, _tuning_values
    
    def x_to_val(x_pos, lo, hi):
        frac = max(0.0, min(1.0, (x_pos - _BAR_X) / max(_BAR_W, 1)))
        return int(lo + frac * (hi - lo))

    if event == cv2.EVENT_LBUTTONDOWN:
        for i, (_, key, lo, hi) in enumerate(_SLIDERS):
            bx1, by1 = _BAR_X, _SLIDER_MARGIN + i*(_SLIDER_H + _SLIDER_GAP) + 4
            bx2, by2 = _BAR_X + _BAR_W, by1 + _SLIDER_H - 8
            if bx1 <= x <= bx2 and by1 <= y <= by2:
                _tuning_dragging = key
                _tuning_values[key] = x_to_val(x, lo, hi)
                break
    elif event == cv2.EVENT_MOUSEMOVE and (flags & cv2.EVENT_FLAG_LBUTTON):
        if _tuning_dragging:
            for _, key, lo, hi in _SLIDERS:
                if key == _tuning_dragging:
                    _tuning_values[key] = x_to_val(x, lo, hi)
                    break
    elif event == cv2.EVENT_LBUTTONUP:
        _tuning_dragging = None


def create_trackbars():
    cv2.namedWindow(TRACKBAR_WIN, cv2.WINDOW_AUTOSIZE | cv2.WINDOW_GUI_NORMAL)
    cv2.setMouseCallback(TRACKBAR_WIN, tuning_mouse_cb)

    if picked_hsv:
        _tuning_values["hc"] = int(picked_hsv[0])
        _tuning_values["sc"] = int(picked_hsv[1])
        _tuning_values["vc"] = int(picked_hsv[2])
    _tuning_values["ht"] = tolerance_r2.get("h", 15)
    _tuning_values["st"] = tolerance_r2.get("s", 60)
    _tuning_values["vt"] = tolerance_r2.get("v", 60)


def _sync_trackbars():
    if picked_hsv:
        _tuning_values["hc"] = int(picked_hsv[0])
        _tuning_values["sc"] = int(picked_hsv[1])
        _tuning_values["vc"] = int(picked_hsv[2])
    _tuning_values["ht"] = tolerance_r2.get("h", 15)
    _tuning_values["st"] = tolerance_r2.get("s", 60)
    _tuning_values["vt"] = tolerance_r2.get("v", 60)


def read_trackbars():
    global tolerance_r2
    
    panel = np.full((_PANEL_H, _PANEL_W, 3), (30, 30, 30), dtype=np.uint8)
    for i, (label, key, lo, hi) in enumerate(_SLIDERS):
        y = _SLIDER_MARGIN + i * (_SLIDER_H + _SLIDER_GAP)
        val = _tuning_values[key]
        bx1, by1 = _BAR_X, y + 4
        bx2, by2 = _BAR_X + _BAR_W, y + _SLIDER_H - 4
        
        cv2.putText(panel, label, (_SLIDER_MARGIN, y + _SLIDER_H - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (220, 220, 220), 1)
        cv2.rectangle(panel, (bx1, by1), (bx2, by2), (60, 60, 60), -1)
        
        fill_x = int(_BAR_X + ((val - lo) / max(hi - lo, 1)) * _BAR_W)
        cv2.rectangle(panel, (bx1, by1), (fill_x, by2), (0, 200, 255), -1)
        cv2.circle(panel, (fill_x, (by1 + by2) // 2), 8, (255, 255, 255), -1)
        cv2.circle(panel, (fill_x, (by1 + by2) // 2), 8, (100, 100, 100), 1)
        cv2.putText(panel, str(val), (bx2 + 6, y + _SLIDER_H - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 200), 1)

    cv2.imshow(TRACKBAR_WIN, panel)

    hc = _tuning_values["hc"]
    ht = _tuning_values["ht"]
    sc = _tuning_values["sc"]
    st = _tuning_values["st"]
    vc = _tuning_values["vc"]
    vt = _tuning_values["vt"]
    tolerance_r2["h"] = max(1, ht)
    tolerance_r2["s"] = max(1, st)
    tolerance_r2["v"] = max(1, vt)
    apply_tolerance_r2(hc, sc, vc)
    if _cam_ref is not None:
        exp = _tuning_values["exp"]
        if exp == 0:
            _cam_ref.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3)
        else:
            _cam_ref.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
            _cam_ref.set(cv2.CAP_PROP_EXPOSURE, exp)


def mouse_pick(event, x, y, _flags, _param):
    global picked_hsv, is_drawing, custom_drawn_path, CHANNEL_REVERSE_DRIVE, _CURRENT_CAM_INDEX, _CAM_SWITCH_PENDING
    
    # Check for custom button clicks
    fw = FRAME_WIDTH
    if event == cv2.EVENT_LBUTTONDOWN:
        if fw - 200 <= x <= fw - 100 and 100 <= y <= 130:
            CHANNEL_REVERSE_DRIVE = not CHANNEL_REVERSE_DRIVE
            print(f"[UI] Invert Drive: {CHANNEL_REVERSE_DRIVE}")
            return
        if fw - 95 <= x <= fw - 5 and 100 <= y <= 130:
            if _AVAILABLE_CAMERAS:
                _CURRENT_CAM_INDEX = (_CURRENT_CAM_INDEX + 1) % len(_AVAILABLE_CAMERAS)
                _CAM_SWITCH_PENDING = True
            return
    
    if path_draw_mode:
        if event == cv2.EVENT_LBUTTONDOWN:
            is_drawing = True
            custom_drawn_path = [(x, y)]
        elif event == cv2.EVENT_MOUSEMOVE and is_drawing:
            custom_drawn_path.append((x, y))
            if len(custom_drawn_path) > 200: # limit length
                custom_drawn_path.pop(0)
        elif event == cv2.EVENT_LBUTTONUP:
            is_drawing = False
            custom_drawn_path.append((x, y))
        return

    if event == cv2.EVENT_LBUTTONDOWN and current_hsv_frame is not None:
        h, s, v    = current_hsv_frame[y, x]
        picked_hsv = (int(h), int(s), int(v))
        _tuning_values["hc"] = int(h)
        _tuning_values["sc"] = int(s)
        _tuning_values["vc"] = int(v)
        apply_tolerance_r2(int(h), int(s), int(v))
        print(f"[R2 PICK] HSV {picked_hsv}")


def detect_color(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    if hsv_lower[0] > hsv_upper[0]:
        mask = cv2.bitwise_or(
            cv2.inRange(hsv, np.array([0, hsv_lower[1], hsv_lower[2]]), hsv_upper),
            cv2.inRange(hsv, hsv_lower, np.array([179, hsv_upper[1], hsv_upper[2]])))
    else:
        mask = cv2.inRange(hsv, hsv_lower, hsv_upper)
    k    = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,   k)
    mask = cv2.morphologyEx(mask, cv2.MORPH_DILATE, k)
    ctrs, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not ctrs: return None, mask
    best = max(ctrs, key=cv2.contourArea)
    if cv2.contourArea(best) < MIN_CONTOUR_AREA: return None, mask
    M = cv2.moments(best)
    if M["m00"] == 0: return None, mask
    return (int(M["m10"]/M["m00"]), int(M["m01"]/M["m00"]),
            cv2.contourArea(best), best), mask


# ──────────────────────────────────────────────────────────────────────────────
# APRILTAG DETECTION
# ──────────────────────────────────────────────────────────────────────────────
_roi:        dict = {}
_roi_miss:   dict = {}
_mid_source: str  = ""
_tag_offset: dict = {}


def make_detector():
    return _at.Detector(
        families=TAG_FAMILY, nthreads=DETECTOR_THREADS,
        quad_decimate=QUAD_DECIMATE, quad_sigma=0.0,
        refine_edges=1, decode_sharpening=0.25)


def _update_roi(tid, corners_int, fw, fh):
    _roi[tid]      = (max(0,  corners_int[:, 0].min() - ROI_PADDING),
                      max(0,  corners_int[:, 1].min() - ROI_PADDING),
                      min(fw, corners_int[:, 0].max() + ROI_PADDING),
                      min(fh, corners_int[:, 1].max() + ROI_PADDING))
    _roi_miss[tid] = 0


def _build_tag(corners, center, ox=0, oy=0):
    corners = corners + np.array([ox, oy])
    return {"center": (center[0]+ox, center[1]+oy),
            "corners_2d": corners,
            "corners_int": corners.astype(np.int32),
            "rvec": None, "tvec": None, "_pose_done": False}


def _detect_gray(detector, gray):
    return [_parse_det(d) for d in detector.detect(gray)]


def detect_tags(frame, detector):
    fh, fw = frame.shape[:2]
    tags   = {}
    for tid, roi in list(_roi.items()):
        if roi is None: continue
        x1, y1, x2, y2 = roi
        crop = frame[y1:y2, x1:x2]
        if crop.size == 0:
            _roi_miss[tid] = _roi_miss.get(tid, 0) + 1
            continue
        found = {fid: _build_tag(c, ctr, x1, y1)
                 for c, ctr, fid in _detect_gray(
                     detector, cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY))}
        if tid in found:
            tags[tid] = found[tid]
            _update_roi(tid, tags[tid]["corners_int"], fw, fh)
        else:
            _roi_miss[tid] = _roi_miss.get(tid, 0) + 1
            if _roi_miss[tid] >= ROI_REACQUIRE_FRAMES:
                _roi[tid] = None

    need_ids  = set(ROBOT1_TAG_IDS) if ROBOT1_TAG_IDS else None
    need_full = (not _roi or
                 (need_ids and not need_ids.issubset(tags)) or
                 (not need_ids and len(tags) < 2))
    if need_full:
        gray_full = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        for c, ctr, tid in _detect_gray(detector, gray_full):
            if tid not in tags:
                tags[tid] = _build_tag(c, ctr)
                _update_roi(tid, tags[tid]["corners_int"], fw, fh)
    return tags


def solve_pose(info):
    if info["_pose_done"]: return
    info["_pose_done"] = True
    ok, rvec, tvec = cv2.solvePnP(
        TAG_POINTS_3D, info["corners_2d"],
        CAMERA_MATRIX, DIST_COEFFS,
        flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if ok:
        info["rvec"], info["tvec"] = rvec, tvec


def _tag_yaw(info):
    solve_pose(info)
    if info["rvec"] is None: return None
    R, _ = cv2.Rodrigues(info["rvec"])
    return math.atan2(R[1, 0], R[0, 0])


def _rot2d(dx, dy, a):
    c, s = math.cos(a), math.sin(a)
    return c*dx - s*dy, s*dx + c*dy


def _learn_offset(tid, info, mx, my):
    cx, cy = info["center"]
    yaw    = _tag_yaw(info)
    if yaw is None:
        _tag_offset[tid] = (mx - cx, my - cy, None)
    else:
        ldx, ldy = _rot2d(mx-cx, my-cy, -yaw)
        _tag_offset[tid] = (ldx, ldy, yaw)


def _apply_offset(tid, info):
    if tid not in _tag_offset: return None
    ldx, ldy, _ = _tag_offset[tid]
    cx, cy = info["center"]
    yaw    = _tag_yaw(info)
    if yaw is None:
        return (int(cx + ldx), int(cy + ldy))
    wdx, wdy = _rot2d(ldx, ldy, yaw)
    return (int(cx + wdx), int(cy + wdy))


def robot1_midpoint(tags):
    global _mid_source
    r1_ids = [i for i in (ROBOT1_TAG_IDS if ROBOT1_TAG_IDS else tags.keys())
              if i in tags and i not in (TARGET_TAG_A, TARGET_TAG_B)]
    if len(r1_ids) >= 2:
        a, b   = tags[r1_ids[0]]["center"], tags[r1_ids[1]]["center"]
        mx, my = (a[0]+b[0])/2, (a[1]+b[1])/2
        _learn_offset(r1_ids[0], tags[r1_ids[0]], mx, my)
        _learn_offset(r1_ids[1], tags[r1_ids[1]], mx, my)
        _mid_source = "both"
        return (int(mx), int(my))
    if len(r1_ids) == 1:
        pt = _apply_offset(r1_ids[0], tags[r1_ids[0]])
        _mid_source = f"tag {r1_ids[0]} +offset" if pt else f"tag {r1_ids[0]} no offset"
        return pt
    _mid_source = ""
    return None


# ──────────────────────────────────────────────────────────────────────────────
# MISC DRAWING
# ──────────────────────────────────────────────────────────────────────────────
def draw_lookahead(frame, r1, path, lookahead=LOOKAHEAD_DIST):
    if r1 is None or path is None: return
    r1_f = np.array(r1, dtype=float)
    for pt in path:
        if np.linalg.norm(pt - r1_f) >= lookahead:
            cv2.circle(frame, (int(pt[0]), int(pt[1])), 7, (255, 0, 255), 2)
            cv2.line(frame, r1, (int(pt[0]), int(pt[1])), (255, 0, 255), 1)
            break


def draw_heading_arrow(frame, r1, heading_deg, length=60):
    if r1 is None: return
    h_rad = math.radians(heading_deg)
    tip   = (int(r1[0] + math.cos(h_rad) * length),
             int(r1[1] + math.sin(h_rad) * length))
    cv2.arrowedLine(frame, r1, tip, (255, 255, 255), 2, tipLength=0.3)


def draw_threat_indicator(frame, r2, evade_pt):
    if evade_pt is None: return
    cv2.putText(frame, "EVADE", (20, 120),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
    if r2:
        cv2.arrowedLine(frame,
                        (int(r2[0]), int(r2[1])),
                        (int(evade_pt[0]), int(evade_pt[1])),
                        (0, 0, 255), 2, tipLength=0.2)


def draw_auto_hud(frame, auto_mode, target_id, ch1, ch2, ch3, fps):
    fh, fw   = frame.shape[:2]
    mode_str = "AUTO"   if auto_mode else "MANUAL"
    mode_col = (0, 255, 80) if auto_mode else (0, 140, 255)
    cv2.rectangle(frame, (fw - 200, 0), (fw, 90), (30, 30, 30), -1)
    cv2.putText(frame, mode_str,           (fw-190, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8,  mode_col,       2)
    cv2.putText(frame, f"TGT: TAG {target_id}", (fw-190, 54), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200,200,200), 1)
    cv2.putText(frame, f"CH1:{ch1} CH2:{ch2} CH3:{ch3}", (fw-190, 76), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (160,160,160), 1)
    
    # Draw Invert Drive button
    inv_col = (0, 200, 0) if CHANNEL_REVERSE_DRIVE else (100, 100, 100)
    cv2.rectangle(frame, (fw - 200, 100), (fw - 100, 130), inv_col, -1)
    cv2.rectangle(frame, (fw - 200, 100), (fw - 100, 130), (200, 200, 200), 1)
    cv2.putText(frame, "INVERT", (fw - 182, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)

    # Draw Cam Switch button
    cv2.rectangle(frame, (fw - 95, 100), (fw - 5, 130), (150, 50, 50), -1)
    cv2.rectangle(frame, (fw - 95, 100), (fw - 5, 130), (200, 200, 200), 1)
    cv2.putText(frame, "CAM++", (fw - 70, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)

    if path_draw_mode:
        cv2.putText(frame, "PATH DRAW MODE", (fw // 2 - 120, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 0, 255), 2)
        cv2.putText(frame, "Click & Drag on Tracker to draw path", (fw // 2 - 160, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    cv2.putText(frame, f"FPS:{int(fps)}", (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255,255,255), 2)
    
    status_msg = "A=auto T=target B=bounds P=path draw +/-=offset Q=quit"
    try:
        status_msg += f" | Cam: {_AVAILABLE_CAMERAS[_CURRENT_CAM_INDEX]}"
    except IndexError:
        pass
    cv2.putText(frame, status_msg, (10, fh-10), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (160,160,160), 1)


# ──────────────────────────────────────────────────────────────────────────────
# TERMINAL
# ──────────────────────────────────────────────────────────────────────────────
def terminal_loop():
    global running
    print("\n[CMD] save | load | info | quit\n")
    while running:
        try:
            cmd = input("> ").strip().lower()
        except EOFError:
            break
        if   cmd == "quit": running = False
        elif cmd == "save": save_profile()
        elif cmd == "load": load_profile()
        elif cmd == "info": print(f"  R2 HSV lower={hsv_lower.tolist()} upper={hsv_upper.tolist()}")
        else: print("  Commands: save  load  info  quit")


# ──────────────────────────────────────────────────────────────────────────────
# SIGNAL HANDLER & LOG
# ──────────────────────────────────────────────────────────────────────────────
def _signal_handler(sig, _frame):
    global running
    print(f"\n[SIGNAL] {signal.Signals(sig).name} — shutting down...")
    running = False

signal.signal(signal.SIGINT,  _signal_handler)
signal.signal(signal.SIGTERM, _signal_handler)


def init_log():
    new = not os.path.exists(POSITION_LOG)
    f   = open(POSITION_LOG, "a", newline="")
    w   = csv.writer(f)
    if new:
        w.writerow(["timestamp","r1_x","r1_y","r1_src",
                    "r2_x","r2_y","r2_src","ch1","ch2","ch3","auto"])
    return f, w


def log_row(writer, r1, r1s, r2, r2s, ch1, ch2, ch3, auto):
    ts = datetime.now().isoformat(timespec="milliseconds")
    writer.writerow([ts,
                     r1[0] if r1 else "", r1[1] if r1 else "", r1s,
                     r2[0] if r2 else "", r2[1] if r2 else "", r2s,
                     ch1, ch2, ch3, int(auto)])


# ──────────────────────────────────────────────────────────────────────────────
# MAIN
# ──────────────────────────────────────────────────────────────────────────────
def main():
    global current_hsv_frame, running, _cam_ref, path_draw_mode, _CAM_SWITCH_PENDING

    detector = make_detector()
    scan_cameras()
    active_device = _AVAILABLE_CAMERAS[_CURRENT_CAM_INDEX] if _AVAILABLE_CAMERAS else CAMERA_DEVICE

    try:
        cam = CameraStream(active_device, FRAME_WIDTH, FRAME_HEIGHT)
    except RuntimeError as e:
        print(f"[ERROR] {e}"); return

    _cam_ref = cam
    arduino  = ArduinoSerial(ARDUINO_PORT, ARDUINO_BAUD)
    r1_color = Robot1ColorTracker()

    cv2.namedWindow("Tracker", cv2.WINDOW_NORMAL | cv2.WINDOW_GUI_NORMAL)
    cv2.namedWindow("Mask",    cv2.WINDOW_NORMAL | cv2.WINDOW_GUI_NORMAL)
    create_trackbars()
    cv2.setMouseCallback("Tracker", mouse_pick)
    load_profile()

    log_file, log_writer = init_log()
    threading.Thread(target=terminal_loop, daemon=True).start()

    auto_mode       = False
    show_bounds     = True
    target_id       = TARGET_TAG_A
    current_path    = None
    last_r1_path    = None
    last_r2_path    = None
    current_heading = 0.0
    ch1 = ch2 = 1500
    ch3 = WEAPON_OFF_VALUE
    last_t = time.time()
    retreat_end_time = 0.0

    while running:
        if _CAM_SWITCH_PENDING:
            new_cam = _AVAILABLE_CAMERAS[_CURRENT_CAM_INDEX]
            print(f"[CAM] Switching to {new_cam}...")
            cam.release()
            try:
                cam = CameraStream(new_cam, FRAME_WIDTH, FRAME_HEIGHT)
                _cam_ref = cam
            except RuntimeError as e:
                print(f"[ERROR] Failed to switch: {e}")
            _CAM_SWITCH_PENDING = False

        frame = cam.read()
        if frame is None:
            time.sleep(0.005); continue

        now    = time.time()
        dt     = max(now - last_t, 1e-4)
        fps    = 1.0 / dt
        last_t = now

        hsv_frame         = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        current_hsv_frame = hsv_frame
        read_trackbars()

        # ── Robot1 ────────────────────────────────────────────────────────────
        tags    = detect_tags(frame, detector)
        raw_mid = robot1_midpoint(tags)

        if raw_mid:
            r1_pos = kf1.update(raw_mid[0], raw_mid[1], dt)
            r1_src = _mid_source
        else:
            color_pos = r1_color.detect(frame, hsv_frame)
            if color_pos:
                r1_pos = kf1.update(color_pos[0], color_pos[1], dt)
                r1_src = "color-fallback"
            else:
                r1_pos = kf1.predict(dt)
                r1_src = "coast" if r1_pos else "lost"

        r1_color.maybe_resample(hsv_frame, r1_pos)
        current_heading = estimate_heading(tags, kf1)

        # ── Robot2 ────────────────────────────────────────────────────────────
        color_result, mask = detect_color(frame)
        if color_result:
            cx, cy, *_ = color_result
            r2_pos = kf2.update(cx, cy, dt)
            r2_src = "color"
        else:
            r2_pos = kf2.predict(dt)
            r2_src = "coast" if r2_pos else "lost"

        r2_vel = kf2.velocity()

        # ── Target tag ────────────────────────────────────────────────────────
        target_pos = None
        if target_id in tags:
            tc = tags[target_id]["center"]
            target_pos = (int(tc[0]), int(tc[1]))

        # ── Path planning (replans when either robot moves > threshold) ───────
        needs_plan = current_path is None
        if r1_pos and last_r1_path:
            if math.hypot(r1_pos[0]-last_r1_path[0],
                          r1_pos[1]-last_r1_path[1]) > PATH_REPLAN_DIST:
                needs_plan = True
        if r2_pos and last_r2_path:
            if math.hypot(r2_pos[0]-last_r2_path[0],
                          r2_pos[1]-last_r2_path[1]) > PATH_REPLAN_DIST:
                needs_plan = True

        if needs_plan and r1_pos and r2_pos and target_pos:
            current_path = build_path(r1_pos, r2_pos, target_pos,
                                      arena, r1_heading_deg=current_heading)
            last_r1_path = r1_pos
            last_r2_path = r2_pos

        if target_pos is None:
            current_path = None

        # ── Threat evasion ────────────────────────────────────────────────────
        threat_evade = None
        if r1_pos and r2_pos:
            dist = math.hypot(r1_pos[0] - r2_pos[0], r1_pos[1] - r2_pos[1])
            if dist < 60 and now > retreat_end_time:
                # Contact made: trigger an explicit retreat for 1.5s
                print(f"[RETIRE] Contact! Retreating for 1.5s.")
                retreat_end_time = now + 1.5
                
            if now < retreat_end_time:
                # Run away in the direct opposite direction
                escape_angle = math.atan2(r1_pos[1] - r2_pos[1], r1_pos[0] - r2_pos[0])
                threat_evade = arena.clamp((r1_pos[0] + math.cos(escape_angle) * EVASION_OFFSET * 1.5,
                                            r1_pos[1] + math.sin(escape_angle) * EVASION_OFFSET * 1.5))
            else:
                threat_evade = evasion_offset(r1_pos, r2_pos, r2_vel)

        # ── Control output ────────────────────────────────────────────────────
        if auto_mode and r1_pos:
            if path_draw_mode and len(custom_drawn_path) >= 2:
                pts = np.array(custom_drawn_path, dtype=np.float32)
                drive_cmd, steer_cmd = pure_pursuit(r1_pos, current_heading, pts)
                ch3 = WEAPON_OFF_VALUE
            elif threat_evade:
                evade_path       = build_evasion_path(r1_pos, threat_evade, arena,
                                                      r1_heading_deg=current_heading)
                drive_cmd, steer_cmd = pure_pursuit(r1_pos, current_heading, evade_path)
                ch3 = WEAPON_OFF_VALUE
            elif current_path is not None:
                drive_cmd, steer_cmd = pure_pursuit(r1_pos, current_heading, current_path)
                ch3 = WEAPON_ON_VALUE
            else:
                drive_cmd, steer_cmd = 1500, 1500
                ch3 = WEAPON_OFF_VALUE
            ch2 = drive_cmd
            ch1 = steer_cmd
            arduino.send(ch1, ch2, ch3)
        else:
            ch1 = ch2 = 1500
            ch3 = WEAPON_OFF_VALUE
            arduino.send_neutral()

        # ── Draw ──────────────────────────────────────────────────────────────
        for tid_draw, info in tags.items():
            ci     = info["corners_int"]
            cx_d   = int(info["center"][0])
            cy_d   = int(info["center"][1])
            c_tag  = (0, 255, 0) if tid_draw not in (TARGET_TAG_A, TARGET_TAG_B) else (0, 80, 255)
            cv2.polylines(frame, [ci.reshape(-1, 1, 2)], True, c_tag, 2)
            cv2.circle(frame, (cx_d, cy_d), 5, (0, 200, 255), -1)
            cv2.putText(frame, f"T{tid_draw}", (cx_d+8, cy_d-8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

        if r1_pos:
            col = (255, 200, 0) if r1_src != "coast" else (100, 100, 50)
            cv2.drawMarker(frame, r1_pos, col, cv2.MARKER_CROSS, 26, 3)
            cv2.putText(frame, f"R1 [{r1_src}]", (r1_pos[0]+14, r1_pos[1]-8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, col, 2)
            vx1, vy1 = kf1.velocity()
            if math.hypot(vx1, vy1) > 0.5:
                cv2.arrowedLine(frame, r1_pos,
                                (int(r1_pos[0]+vx1*3), int(r1_pos[1]+vy1*3)),
                                col, 2, tipLength=0.3)
            draw_heading_arrow(frame, r1_pos, current_heading, length=55)

        if r2_pos:
            cv2.circle(frame, r2_pos, 8, (0, 140, 255), -1)
            cv2.putText(frame, f"R2 [{r2_src}]", (r2_pos[0]+12, r2_pos[1]-8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 140, 255), 2)
            vx2, vy2 = r2_vel
            if math.hypot(vx2, vy2) > 0.5:
                cv2.arrowedLine(frame, r2_pos,
                                (int(r2_pos[0]+vx2*3), int(r2_pos[1]+vy2*3)),
                                (0, 0, 255), 2, tipLength=0.3)

        if not path_draw_mode:
            draw_path(frame, current_path, r2_pos, target_pos, target_id)
            draw_lookahead(frame, r1_pos, current_path)
        else:
            if len(custom_drawn_path) >= 2:
                for i in range(len(custom_drawn_path)-1):
                    cv2.line(frame, custom_drawn_path[i], custom_drawn_path[i+1], (255, 0, 255), 3)
            # also draw lookahead for the custom path
            if len(custom_drawn_path) >= 2:
                pts = np.array(custom_drawn_path, dtype=np.float32)
                draw_lookahead(frame, r1_pos, pts)

        draw_threat_indicator(frame, r2_pos, threat_evade)
        if show_bounds:
            arena.draw(frame)
        draw_auto_hud(frame, auto_mode, target_id, ch1, ch2, ch3, fps)

        log_row(log_writer, r1_pos, r1_src, r2_pos, r2_src, ch1, ch2, ch3, auto_mode)
        log_file.flush()

        cv2.imshow("Tracker", frame)
        cv2.imshow("Mask",    mask)

        key = cv2.waitKey(1) & 0xFF
        if   key == ord("q"): running = False
        elif key == ord("a"):
            auto_mode = not auto_mode
            print(f"[MODE] Autonomous: {auto_mode}")
            if not auto_mode: arduino.send_neutral()
        elif key == ord("t"):
            target_id    = TARGET_TAG_B if target_id == TARGET_TAG_A else TARGET_TAG_A
            current_path = None
            print(f"[TARGET] Switched to TAG {target_id}")
        elif key == ord("b"):
            show_bounds = not show_bounds
        elif key == ord("p"):
            path_draw_mode = not path_draw_mode
            if path_draw_mode:
                auto_mode = False # Pause auto to draw
                print("[MODE] Path Draw Enabled. Click and drag in Tracker window.")
            else:
                print("[MODE] Path Draw Disabled.")
        elif key in (ord("+"), ord("=")):
            arena.change_offset(+5); current_path = None
        elif key == ord("-"):
            arena.change_offset(-5); current_path = None

    arduino.send_neutral()
    arduino.close()
    cam.release()
    log_file.close()
    cv2.destroyAllWindows()
    print(f"\n[EXIT] Positions saved to {POSITION_LOG}")


if __name__ == "__main__":
    main()