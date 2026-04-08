# Combat Robot Controller

Autonomous vision-guided combat robot system. Uses AprilTag detection, HSV color tracking, Kalman filtering, Bézier path planning, and pure-pursuit control to drive a combat robot that pushes an opponent toward a target zone.

## Hardware Requirements

| Component | Details |
|-----------|---------|
| **Camera** | USB camera with V4L2 support, 1280×720 @ 60fps (e.g., Logitech C920) |
| **Arduino Nano** | Flashed with `arduino_trainer.ino`, connected via USB serial |
| **RC Transmitter** | FS-i6X with trainer port wired to Arduino D9 (PPM) + GND |
| **AprilTags** | tag25h9 family, 33.02 mm (1.3 in), IDs 0 & 1 on Robot 1, IDs 3 & 4 as targets |

### Wiring Diagram

```
Camera (USB) ──► Linux Host ──► Arduino Nano (USB Serial)
                                       │
                                       D9 (PPM out) ──► FS-i6X Trainer Port
                                       GND ───────────► FS-i6X Trainer GND
```

## Software Setup

```bash
# Clone and enter the project
cd Control/

# Install Python dependencies
pip install -r requirements.txt

# (Optional) Run camera calibration
python color_track.py
```

### Dependencies

| Package | Version | Purpose |
|---------|---------|---------|
| Python | ≥3.8 | Runtime |
| OpenCV (`cv2`) | ≥4.5 | Vision, GUI, Kalman filter |
| NumPy | ≥1.20 | Array math |
| pupil-apriltags | any | AprilTag detection (mandatory) |
| pyserial | any | Arduino communication (optional) |
| PyYAML | any | Configuration loading |

## Configuration

All tunable parameters live in `config.yaml` at the project root. Key settings:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `camera_device` | `/dev/video1` | V4L2 camera path |
| `frame_width` / `frame_height` | 1280 / 720 | Camera resolution |
| `exposure` | 150 | Manual exposure (null for auto) |
| `arena_tl` / `arena_br` | [50, 50] / [1230, 670] | Arena pixel coordinates |
| `robot_offset` | 40 | Safety inset from arena edge (px) |
| `max_drive_speed` / `max_turn_speed` | 400 | Max PWM offset from neutral |
| `approach_standoff` | 80 | Lineup distance behind R2 (px) |
| `threat_cone_deg` | 40.0 | Evasion trigger angle |

See `config.yaml` for the full list.

## Running

```bash
# Run with default config
python -m robot_control

# Run with a custom config
python -m robot_control --config /path/to/my_config.yaml
```

## Operation Guide

### Keyboard Controls (on the Tracker window)

| Key | Action |
|-----|--------|
| `Q` | Quit |
| `A` | Toggle autonomous mode on/off |
| `T` | Toggle target tag (3 ↔ 4) |
| `B` | Toggle arena bounds overlay |
| `+` / `-` | Increase / decrease robot safety offset |
| **Click** | Pick Robot 2 color from live feed |

### Terminal Commands

Type into the terminal while the tracker is running:

| Command | Action |
|---------|--------|
| `save` | Save R2 color profile to JSON |
| `load` | Load R2 color profile from JSON |
| `info` | Print current R2 HSV bounds |
| `quit` | Shutdown |

### Startup Sequence

1. Position the overhead camera with a clear view of the full arena.
2. Place AprilTags on Robot 1 (IDs 0 and 1, one on each side).
3. Place target tags (IDs 3 and 4) in the arena.
4. Connect Arduino Nano via USB.
5. Run `python -m robot_control`.
6. Click on Robot 2 in the video feed to pick its color.
7. Press `A` to enable autonomous mode.

## Architecture

The system follows a **sense → plan → act** pipeline:

```
robot_control/
├── __init__.py              # Package
├── __main__.py              # Entry point + main loop
├── config.py                # YAML config loader + validation
├── camera.py                # Threaded camera capture
├── filters.py               # Kalman tracker (6-state)
├── serial_output.py         # Arduino serial writer
├── arena.py                 # Arena bounds + safe zone
├── watchdog.py              # Safety watchdogs
├── profile.py               # Color profile persistence
├── state.py                 # Runtime state containers
├── log.py                   # CSV position logger
├── detection/
│   ├── apriltag_tracker.py  # AprilTag detection + midpoint + heading
│   └── color_tracker.py     # HSV color tracking (R1 fallback + R2)
├── planning/
│   ├── path_planner.py      # Bézier path builder
│   ├── pursuit.py           # Pure-pursuit controller
│   └── evasion.py           # Threat evasion
└── ui/
    ├── drawing.py           # Path + marker drawing
    ├── hud.py               # HUD overlay
    └── trackbars.py         # HSV tuning trackbars
```

For detailed design documentation, see [`docs/design/design.md`](docs/design/design.md).

## License

Internal project — not yet licensed for distribution.
