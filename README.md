# Combat Robot Controller

Autonomous vision-guided combat robot system. Uses a **Kinect V2** depth sensor with ArUco marker detection, depth-based opponent tracking, and Kalman-filtered state estimation to drive a combat robot in an arena.

## Hardware Requirements

| Component | Details |
|-----------|---------|
| **Kinect V2** | Microsoft Kinect for Xbox One (USB 3.0 required) |
| **Mounting** | Side-mounted at an angle looking into the arena (not overhead) |
| **Arduino Nano** | Flashed with `arduino_trainer.ino`, connected via USB serial |
| **RC Transmitter** | FS-i6X with trainer port wired to Arduino D9 (PPM) + GND |
| **ArUco Tags** | `DICT_4X4_50` dictionary — ID 0 on top of robot, ID 1 on bottom |

### Sensor Overview

The Kinect V2 provides three data streams:

| Stream | Resolution | FPS | Use |
|--------|-----------|-----|-----|
| **Color** | 1920×1080 | 30 | Visual reference / display |
| **Depth** | 512×424 | 30 | Opponent detection (above ground plane) |
| **IR** | 512×424 | 30 | ArUco tag detection (lighting-independent) |

### Tag Configuration

The robot has **two ArUco tags** — one on top and one on the bottom — so it remains trackable when flipped upside down. The system determines orientation by which tag is currently visible.

### Camera Mounting

The Kinect is mounted on the **side of the arena** looking inward at an angle (not overhead like the previous webcam setup). This means:
- Objects are seen at a perspective angle
- Ground plane detection is needed to distinguish arena floor from obstacles
- Depth values vary across the field of view based on geometry

### Wiring Diagram

```
Kinect V2 (USB 3.0) ──► Linux Host ──► Arduino Nano (USB Serial)
                                              │
                                              D9 (PPM out) ──► FS-i6X Trainer Port
                                              GND ───────────► FS-i6X Trainer GND
```

## Prerequisites

- **Python 3.8+** (tested on 3.12)
- **Linux** with USB 3.0 support
- **libfreenect2** built from source (see installation below)
- Kinect V2 connected via USB 3.0
- (Optional) Arduino Nano connected via USB

## Software Setup

```bash
# 1. Enter the project directory
cd Control/

# 2. Install libfreenect2 and pylibfreenect2
chmod +x install_kinect.sh
./install_kinect.sh

# 3. Create a virtual environment (if not already done)
python3 -m venv .venv

# 4. Activate it
source .venv/bin/activate

# 5. Install all Python dependencies
pip install -r requirements.txt
```

> **Note:** `libfreenect2` requires USB 3.0 and specific system libraries. The `install_kinect.sh` script handles all of this automatically, including udev rules for non-root access.

### Dependencies

| Package | Version | Purpose |
|---------|---------|---------|
| Python | ≥3.8 | Runtime |
| OpenCV (`cv2`) | ≥4.5 | Vision, GUI, ArUco detection, Kalman filter |
| NumPy | ≥1.20 | Array math |
| pylibfreenect2 | any | Kinect V2 interface (mandatory) |
| libfreenect2 | latest | C++ library for Kinect V2 (build from source) |
| pyserial | any | Arduino communication (optional — degrades gracefully) |
| PyYAML | any | Configuration loading |

## Configuration

All tunable parameters live in `config.yaml` at the project root. Key settings:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `kinect_pipeline` | `opengl` | Processing backend: `opengl`, `opencl`, or `cpu` |
| `aruco_dict` | `DICT_4X4_50` | OpenCV ArUco dictionary for tag detection |
| `robot_tag_top` | `0` | ArUco ID on top of robot |
| `robot_tag_bottom` | `1` | ArUco ID on bottom of robot |
| `detect_on_ir` | `true` | Detect tags on IR stream (lighting-independent) |
| `arena_tl` / `arena_br` | [50,50] / [462,374] | Arena bounds in depth-frame pixel coordinates |
| `robot_offset` | `20` | Safety inset from arena edge (px) |
| `max_drive_speed` / `max_turn_speed` | `400` | Max PWM offset from neutral |
| `arduino_port` | `null` | Serial port (`null` = auto-detect) |

See `config.yaml` for the full list with comments.

## Running the Application

### Quick Start — Kinect Viewer

```bash
# Make sure your venv is active
source .venv/bin/activate

# View all Kinect streams (Color, Depth, IR, Registered)
python -m robot_control.vision.kinect_viewer

# With a specific pipeline backend
python -m robot_control.vision.kinect_viewer --pipeline cpu
```

### Viewer Controls

| Key | Action |
|-----|--------|
| `Q` | Quit |

### Pre-Flight Checklist

Before launching, verify:

1. **Kinect V2 connected** via USB 3.0 port
2. **libfreenect2 installed** — `install_kinect.sh` should have been run
3. **udev rules applied** — unplug/replug Kinect after first install
4. **ArUco tags printed** — IDs 0 (top) and 1 (bottom) on robot
5. **(Optional) Arduino plugged in** — `ls /dev/ttyUSB*` or `ls /dev/ttyACM*`

### Running Tests

```bash
source .venv/bin/activate
python3 -m pytest tests/ -v
```

### Troubleshooting

| Problem | Solution |
|---------|----------|
| `No Kinect V2 devices found` | Check USB 3.0 connection; ensure udev rules are installed |
| `No module named pylibfreenect2` | Run `./install_kinect.sh` or `pip install pylibfreenect2` |
| `OpenGL pipeline not available` | Try `--pipeline cpu`; install OpenGL dev libraries |
| `Kinect V2 timed out` | Ensure no other process is using the Kinect |
| `Permission denied` | Run `sudo cp libfreenect2/platform/linux/udev/90-kinect2.rules /etc/udev/rules.d/` and replug |
| Low FPS | Try `--pipeline opengl` for GPU acceleration |

## Architecture

The system is being rewritten to follow a **sense → plan → act** pipeline using the Kinect V2:

```
robot_control/
├── __init__.py              # Package
├── __main__.py              # Entry point (control loop — WIP)
├── config.py                # YAML config loader + validation
├── serial_output.py         # Arduino serial writer
├── filters.py               # Kalman tracker (6-state)
├── arena.py                 # Arena bounds + safe zone
├── watchdog.py              # Safety watchdogs
├── state.py                 # Runtime state containers
├── log.py                   # CSV position logger
├── profile.py               # Profile persistence
└── vision/
    ├── __init__.py           # Vision subpackage
    ├── kinect_stream.py      # Kinect V2 frame acquisition (threaded)
    └── kinect_viewer.py      # Diagnostic stream viewer
```

### Vision Pipeline (planned)

```
Kinect V2 ──┬── IR Stream ──► ArUco Detection ──► Robot Position + Orientation
             ├── Depth Stream ──► Ground Plane ──► Opponent Detection
             └── Color Stream ──► Visual Display
```

**Robot detection**: ArUco tags detected on the IR stream (lighting-independent). Two tags (top/bottom) allow tracking even when the robot is flipped.

**Opponent detection**: Depth sensor identifies objects above the detected ground plane that are not at the robot's position.

For detailed design documentation, see [`docs/design/design.md`](docs/design/design.md).

## License

Internal project — not yet licensed for distribution.
