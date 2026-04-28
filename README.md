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

- **Pop!_OS** or Ubuntu 22.04+ (tested on Pop!_OS 24.04)
- **Python 3.8+** (tested on 3.12)
- **USB 3.0 port** for the Kinect V2
- (Optional) Arduino Nano connected via USB for motor control

## Installation

Two scripts handle the full setup. Run them **in order** from the project root:

### Step 1 — Python environment

```bash
cd Control/
chmod +x install.sh install_kinect.sh

# Creates .venv, installs Python packages (opencv, numpy, pyserial, pyyaml)
./install.sh
```

This will:
1. Verify `python3` and `python3-venv` are available
2. Create a `.venv/` virtual environment
3. Install all Python dependencies from `requirements.txt`
4. Add your user to the `dialout` group (for Arduino serial access)

### Step 2 — Kinect V2 driver

```bash
# Builds libfreenect2 from source, installs udev rules, installs pylibfreenect2
./install_kinect.sh
```

This will:
1. Install system build dependencies (`cmake`, `libusb`, `libglfw3`, etc.)
2. Clone and build [libfreenect2](https://github.com/OpenKinect/libfreenect2) to `~/libfreenect2/`
3. Install udev rules for non-root Kinect access
4. Set `LD_LIBRARY_PATH` in `~/.bashrc` and the venv's `activate` script
5. Build and install `pylibfreenect2` into the project venv

> **After the first install:** unplug and replug the Kinect V2 for udev rules to take effect, then run `source ~/.bashrc` or open a new terminal.

### Dependencies

| Package | Version | Purpose |
|---------|---------|---------|
| Python | ≥3.8 | Runtime |
| OpenCV (`cv2`) | ≥4.5 | Vision, GUI, ArUco detection, Kalman filter |
| NumPy | ≥1.20 | Array math |
| pylibfreenect2 | 0.1.4 | Kinect V2 interface (installed by `install_kinect.sh`) |
| libfreenect2 | latest | C++ library for Kinect V2 (built from source) |
| pyserial | any | Arduino communication (optional — degrades gracefully) |
| PyYAML | any | Configuration loading |

## Configuration

All tunable parameters live in `config.yaml` at the project root. Key settings:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `kinect_pipeline` | `opengl` | Processing backend: `opengl`, `opencl`, or `cpu` |
| `aruco_dict` | `DICT_APRILTAG_25H9` | OpenCV ArUco dictionary for tag detection |
| `robot_tag_top` | `0` | ArUco ID on top of robot |
| `robot_tag_bottom` | `1` | ArUco ID on bottom of robot |
| `detect_on_ir` | `false` | Detect tags on IR stream (`true`) or color stream (`false`) |
| `flip_horizontal` | `true` | Mirror all frames (Kinect mounted inverted) |
| `arena_tl` / `arena_br` | [50,50] / [462,374] | Arena bounds in depth-frame pixel coordinates |
| `robot_offset` | `20` | Safety inset from arena edge (px) |
| `max_drive_speed` / `max_turn_speed` | `400` | Max PWM offset from neutral |
| `arduino_port` | `null` | Serial port (`null` = auto-detect) |

See `config.yaml` for the full list with comments.

## Running the Application

> **Important:** Always run commands using `.venv/bin/python3` (or activate the venv first with `source .venv/bin/activate`). Using the system `python3` directly will likely segfault due to incompatible system OpenCV libraries.

### Quick Start — Kinect Viewer

```bash
# Option A: Run directly (recommended, no activation needed)
.venv/bin/python3 -m robot_control.vision.kinect_viewer

# Option B: Activate venv first
source .venv/bin/activate
python3 -m robot_control.vision.kinect_viewer

# With a specific pipeline backend (if OpenGL causes issues)
.venv/bin/python3 -m robot_control.vision.kinect_viewer --pipeline cpu
```

### Full Autonomous Controller

```bash
.venv/bin/python3 -m robot_control
```

### Viewer Controls

| Key | Action |
|-----|--------|
| `Q` | Quit |
| `Left Click` | Select ground calibration points (4 required), then lock onto obstacle |
| `Middle Click` | Exclude an obstacle from tracking |
| `Right Click` | Clear target lock |
| `Space` | Toggle autonomous mode (main controller only) |

### Pre-Flight Checklist

Before launching, verify:

1. **Kinect V2 connected** via USB 3.0 port
2. **libfreenect2 installed** — `install_kinect.sh` should have been run
3. **udev rules applied** — unplug/replug Kinect after first install
4. **ArUco tags printed** — IDs 0 (top) and 1 (bottom) on robot
5. **(Optional) Arduino plugged in** — `ls /dev/ttyUSB*` or `ls /dev/ttyACM*`

### Running Tests

```bash
.venv/bin/python3 -m pytest tests/ -v
```

### Troubleshooting

| Problem | Solution |
|---------|----------|
| `Segmentation fault` | You're using the system Python, not the venv. Use `.venv/bin/python3` |
| `No Kinect V2 devices found` | Check USB 3.0 connection; ensure udev rules are installed |
| `No module named pylibfreenect2` | Run `./install_kinect.sh` |
| `OpenGL pipeline not available` | Try `--pipeline cpu`; install OpenGL dev libraries |
| `Kinect V2 timed out` | Ensure no other process is using the Kinect |
| `Permission denied` | Replug the Kinect after `install_kinect.sh`; check udev rules |
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
