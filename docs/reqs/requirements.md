# Software Requirements Specification — Combat Robot Controller

## 1. Purpose & Scope

This application provides autonomous and manual control of a combat robot (**Robot 1 / R1**) that must detect, pursue, and push an opponent robot (**Robot 2 / R2**) inside a bounded arena. It runs on a Linux host equipped with a **Kinect V2** depth sensor and drives R1's ESC/servo via an Arduino Nano over serial.

---

## 2. Definitions

| Term | Meaning |
|------|---------|
| R1 | Our combat robot, tracked via two ArUco tags (top + bottom) |
| R2 | Opponent robot, detected via depth sensor (above-ground segmentation) |
| Arena | Physical playing surface bounded by walls |
| Ground Plane | The detected arena floor surface; anything above it is a potential object |
| Coasting | Kalman filter dead-reckoning when a detection source is temporarily lost |
| Registered Frame | Color image mapped to depth camera coordinates for pixel-level alignment |

---

## 3. Functional Requirements

### 3.1 Vision — Kinect V2 Streaming

| ID | Requirement |
|----|-------------|
| FR-KN-01 | Interface with Kinect V2 via `pylibfreenect2` to capture synchronised color, depth, and IR frames |
| FR-KN-02 | Support configurable processing pipeline backend (OpenGL, OpenCL, CPU) |
| FR-KN-03 | Provide a threaded frame acquisition model that decouples capture from processing |
| FR-KN-04 | Provide a standalone diagnostic viewer showing all four streams (Color, Depth, IR, Registered) |
| FR-KN-05 | Handle Kinect startup timeout (10 seconds) with graceful error reporting |
| FR-KN-06 | Handle the side-mounted camera angle (non-overhead perspective) |

### 3.2 Vision — Robot 1 Tracking (ArUco)

| ID | Requirement |
|----|-------------|
| FR-R1-01 | Detect ArUco markers (DICT_4X4_50 dictionary) on the Kinect IR stream for lighting-independent operation |
| FR-R1-02 | Support two tags per robot: one on top (ID 0) and one on bottom (ID 1) for upside-down operation |
| FR-R1-03 | Determine robot orientation (upright vs inverted) based on which tag is currently visible |
| FR-R1-04 | Estimate robot heading from ArUco marker pose (rotation matrix) |
| FR-R1-05 | Smooth R1 position with a 6-state Kalman filter (x, y, vx, vy, ax, ay) and coast through brief occlusions (≤30 frames) |
| FR-R1-06 | Optionally fall back to color-stream ArUco detection if IR detection is disabled |

### 3.3 Vision — Robot 2 Tracking (Depth-Based)

| ID | Requirement |
|----|-------------|
| FR-R2-01 | Detect the ground plane from the depth sensor to establish the arena floor surface |
| FR-R2-02 | Identify objects above the ground plane as potential obstacles/opponents |
| FR-R2-03 | Exclude the region corresponding to R1 (via ArUco position) from opponent candidates |
| FR-R2-04 | Track the largest remaining above-ground object as R2 |
| FR-R2-05 | Smooth R2 position with a separate Kalman filter instance and coast through brief occlusions |

### 3.4 Vision — Ground Plane Detection

| ID | Requirement |
|----|-------------|
| FR-GP-01 | Detect the arena floor as a plane in the depth image, accounting for the camera's angled mounting |
| FR-GP-02 | Support a calibration step to collect ground samples from known empty regions |
| FR-GP-03 | Robustly fit a plane model using RANSAC or similar outlier-resistant method |
| FR-GP-04 | Classify each depth pixel as "ground" or "above ground" based on the fitted plane + threshold |

### 3.5 Path Planning (Future Phase)

| ID | Requirement |
|----|-------------|
| FR-PP-01 | Compute an approach trajectory from R1 to R2 in depth-frame coordinates |
| FR-PP-02 | Re-plan the path when either robot moves significantly |
| FR-PP-03 | Clamp all path waypoints within the arena safe zone |

### 3.6 Motion Control

| ID | Requirement |
|----|-------------|
| FR-MC-01 | Implement path following to compute drive and steer PWM values |
| FR-MC-02 | Output three channels via serial: CH1 (steer), CH2 (drive), CH3 (weapon), each 1000–2000 µs |
| FR-MC-03 | Support channel reversal for each axis |

### 3.7 Arena Bounds

| ID | Requirement |
|----|-------------|
| FR-AB-01 | Define a rectangular arena bounding box in depth-frame pixel coordinates (512×424) |
| FR-AB-02 | Clamp all planned positions to the safe zone |
| FR-AB-03 | Allow runtime adjustment of the safety offset |

### 3.8 User Interface

| ID | Requirement |
|----|-------------|
| FR-UI-01 | Display live Kinect streams with overlaid detection results |
| FR-UI-02 | Display diagnostic information (FPS, detection status, robot positions) |
| FR-UI-03 | Provide keyboard controls for mode switching and parameter adjustment |

### 3.9 Data Logging

| ID | Requirement |
|----|-------------|
| FR-DL-01 | Log every frame: timestamp, R1 position + source, R2 position + source, control outputs, mode |
| FR-DL-02 | Append to existing log files; create with header if new |

---

## 4. Non-Functional Requirements

| ID | Requirement | Target |
|----|-------------|--------|
| NFR-01 | **Latency**: Vision + control loop must run at ≥25 FPS | 40 ms/frame |
| NFR-02 | **Kinect startup**: Must acquire first frame within 10 seconds or fail gracefully | 10 s |
| NFR-03 | **Serial rate**: Arduino command stream at ~50 Hz | 20 ms |
| NFR-04 | **Graceful shutdown**: Signal handlers must send neutral commands, release Kinect, close serial, and flush logs | All resources freed |
| NFR-05 | **Optional dependencies**: Degrade gracefully if `pyserial` is not installed | Warning only |
| NFR-06 | **Mandatory dependencies**: Fail with a clear message if `pylibfreenect2` is missing | RuntimeError |
| NFR-07 | **Lighting independence**: Robot tracking must function regardless of ambient lighting conditions | IR-based detection |

---

## 5. Hardware Interface Requirements

| ID | Interface | Details |
|----|-----------|---------|
| HW-01 | Kinect V2 | USB 3.0, via `libfreenect2`; provides color (1920×1080), depth (512×424), IR (512×424) at 30 fps |
| HW-02 | Arduino Nano | USB serial 9600 baud, single-character command protocol |
| HW-03 | RC Transmitter | FS-i6X trainer port wired to Arduino D9 (PPM) + GND |
| HW-04 | ArUco Tags | DICT_4X4_50, printed matte on paper/card; two per robot (top + bottom) |

---

## 6. Software Dependencies

| Package | Version | Purpose |
|---------|---------|---------|
| Python | ≥3.8 | Runtime |
| OpenCV (`cv2`) | ≥4.5 | Vision, GUI, ArUco detection, Kalman filter |
| NumPy | ≥1.20 | Array math |
| pylibfreenect2 | any | Kinect V2 interface (mandatory) |
| libfreenect2 | latest | C++ Kinect V2 library (build from source) |
| pyserial | any | Arduino communication (optional) |
| PyYAML | any | Configuration loading |

---

## 7. Constraints & Assumptions

- Kinect V2 requires a **USB 3.0** port for data bandwidth.
- Camera is mounted on the **side of the arena** at an angle (not overhead).
- The ground plane must be reasonably flat for depth-based detection to work.
- ArUco tags should be printed matte (glossy surfaces cause IR specular reflections).
- The Kinect V2 IR illuminator provides the lighting for tag detection; no external IR source needed.
- The depth sensor has a minimum range of approximately 0.5 m and maximum of ~4.5 m.
- Arduino Nano firmware (`arduino_trainer.ino`) is assumed pre-flashed and providing PPM output on D9.
