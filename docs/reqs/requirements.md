# Software Requirements Specification — Combat Robot Controller

## 1. Purpose & Scope

This application provides autonomous and manual control of a combat robot (**Robot 1 / R1**) that must detect, pursue, and push an opponent robot (**Robot 2 / R2**) into a designated target zone (marked by AprilTag) inside a bounded arena. It runs on a vision-equipped Linux host and drives R1's ESC/servo via an Arduino Nano over serial.

---

## 2. Definitions

| Term | Meaning |
|------|---------|
| R1 | Our combat robot, tracked via two AprilTags (IDs 0, 1) or color fallback |
| R2 | Opponent robot, tracked via HSV color detection |
| Target Tag | AprilTag (ID 3 or 4) marking the push destination |
| Arena | Physical playing surface defined by pixel-coordinate bounding box |
| Knock-through | Maneuver where R1 rams R2 toward the target tag |
| Coasting | Kalman filter dead-reckoning when a detection source is temporarily lost |

---

## 3. Functional Requirements

### 3.1 Vision — Robot 1 Tracking

| ID | Requirement |
|----|-------------|
| FR-R1-01 | Detect AprilTag IDs 0 and 1 (tag25h9 family) on R1 and compute the midpoint as R1's position |
| FR-R1-02 | When only one R1 tag is visible, estimate midpoint using a previously-learned rotation-aware offset |
| FR-R1-03 | Provide a color-based fallback tracker for R1 with adaptive HSV re-sampling every N frames |
| FR-R1-04 | Smooth R1 position with a 6-state Kalman filter (x, y, vx, vy, ax, ay) and coast through brief occlusions (≤30 frames) |
| FR-R1-05 | Estimate R1 heading from tag pose (solvePnP) when available; fall back to velocity-based heading |
| FR-R1-06 | Use ROI-based detection first, falling back to full-frame scan after 6 consecutive misses |

### 3.2 Vision — Robot 2 Tracking

| ID | Requirement |
|----|-------------|
| FR-R2-01 | Track R2 by HSV color segmentation with user-tunable hue, saturation, and value tolerances |
| FR-R2-02 | Handle hue wrap-around (e.g., red spanning 0°/360°) |
| FR-R2-03 | Apply morphological open + dilate to clean the color mask before contour detection |
| FR-R2-04 | Smooth R2 position with a separate Kalman filter instance and coast through brief occlusions |
| FR-R2-05 | Allow the user to pick R2's color by clicking on the live video feed |

### 3.3 Vision — Target Tag

| ID | Requirement |
|----|-------------|
| FR-TG-01 | Detect AprilTag ID 3 or ID 4 as the current push target |
| FR-TG-02 | Allow runtime toggle of the active target tag via keyboard (`T` key) |

### 3.4 Path Planning

| ID | Requirement |
|----|-------------|
| FR-PP-01 | Compute a cubic Bézier approach curve from R1 to an entry point behind R2 (opposite side from target tag), followed by a straight segment through R2 toward the target |
| FR-PP-02 | Re-plan the path when either robot moves more than `PATH_REPLAN_DIST` (20 px) from the position used in the last plan |
| FR-PP-03 | Clamp all path control points within the arena safe zone |
| FR-PP-04 | Null the path when the target tag is not visible |

### 3.5 Motion Control

| ID | Requirement |
|----|-------------|
| FR-MC-01 | Implement pure-pursuit path following: compute drive and steer PWM values from heading error relative to the lookahead point |
| FR-MC-02 | Scale drive speed inversely with heading error to prevent full-speed turns |
| FR-MC-03 | Output three channels via serial: CH1 (steer), CH2 (drive), CH3 (weapon), each 1000–2000 µs with 1500 = neutral |
| FR-MC-04 | Activate the weapon channel (2000 µs) when following an attack path; deactivate (1000 µs) during evasion or idle |

### 3.6 Threat Evasion

| ID | Requirement |
|----|-------------|
| FR-TE-01 | Detect when R2's velocity vector points within a configurable threat cone (±40°) toward R1 |
| FR-TE-02 | Compute a perpendicular escape waypoint and follow a Bézier evasion path |
| FR-TE-03 | Evasion takes priority over attack path following |

### 3.7 Arena Bounds

| ID | Requirement |
|----|-------------|
| FR-AB-01 | Define a rectangular arena bounding box with a configurable robot-size safety offset |
| FR-AB-02 | Clamp all planned positions and waypoints to the safe zone |
| FR-AB-03 | Allow runtime adjustment of the safety offset via `+`/`-` keys |

### 3.8 User Interface

| ID | Requirement |
|----|-------------|
| FR-UI-01 | Display live annotated video with: tag outlines, R1/R2 markers, velocity arrows, heading arrows, planned path, knock-through axis, and threat indicators |
| FR-UI-02 | Display a HUD overlay showing: mode (AUTO/MANUAL), target tag ID, CH1/CH2/CH3 values, FPS |
| FR-UI-03 | Provide a separate color mask window |
| FR-UI-04 | Provide HSV tuning trackbars (H/S/V center + tolerance) and an exposure slider |
| FR-UI-05 | Provide a terminal-based command interface for save/load/info/quit |

### 3.9 Data Logging

| ID | Requirement |
|----|-------------|
| FR-DL-01 | Log every frame to `positions.csv`: timestamp, R1 position + source, R2 position + source, CH1, CH2, CH3, auto mode |
| FR-DL-02 | Append to existing log files; create with header if new |

### 3.10 Configuration Persistence

| ID | Requirement |
|----|-------------|
| FR-CP-01 | Save/load R2's HSV color profile to `color_profile.json` |
| FR-CP-02 | Save/load R1's adaptive color profile to `robot1_color.json` |

---

## 4. Non-Functional Requirements

| ID | Requirement | Target |
|----|-------------|--------|
| NFR-01 | **Latency**: Vision + control loop must run at ≥30 FPS on target hardware | 33 ms/frame |
| NFR-02 | **Camera startup**: Must acquire first frame within 5 seconds or fail gracefully | 5 s |
| NFR-03 | **Serial rate**: Arduino command stream at ~50 Hz | 20 ms |
| NFR-04 | **Graceful shutdown**: SIGINT/SIGTERM must send neutral commands, release camera, close serial, and flush logs | All resources freed |
| NFR-05 | **Optional dependencies**: Degrade gracefully if `pyserial` is not installed (disable serial output) | Warning only |
| NFR-06 | **Mandatory dependencies**: Fail immediately with a clear message if `pupil-apriltags` is missing | SystemExit |

---

## 5. Hardware Interface Requirements

| ID | Interface | Details |
|----|-----------|---------|
| HW-01 | USB Camera | V4L2, 1280×720 @ 60fps, MJPEG codec, configurable exposure |
| HW-02 | Arduino Nano | USB serial 115200 baud, newline-terminated `"CH1,CH2,CH3\n"` packets |
| HW-03 | RC Transmitter | FS-i6X trainer port wired to Arduino D9 (PPM) + GND |
| HW-04 | AprilTags | tag25h9 family, 33.02 mm (1.3 in) physical size |

---

## 6. Software Dependencies

| Package | Version | Purpose |
|---------|---------|---------|
| Python | ≥3.8 | Runtime |
| OpenCV (`cv2`) | ≥4.5 | Vision, GUI, Kalman filter |
| NumPy | ≥1.20 | Array math |
| pupil-apriltags | any | AprilTag detection (mandatory) |
| pyserial | any | Arduino communication (optional) |

---

## 7. Constraints & Assumptions

- Camera is mounted in a fixed overhead position with a downward view of the full arena.
- Camera intrinsics use rough estimates (focal length = frame width); a proper calibration (`color_track.py` exists) would improve pose accuracy.
- ArduinoNano firmware (`arduino_trainer.ino`) is assumed pre-flashed and providing PPM output on D9.
- The arena bounding box is defined in pixel coordinates hard-coded to the current camera resolution and mounting position.
