# Completed: All 15 Recommended Actions

**Date completed**: 2026-04-07  
**Source**: `docs/recommended_actions.md`

---

## Action 1: Extract Configuration to YAML
**Original**: 40+ constants hard-coded in `robot_brain.py`. Changing parameters requires editing source code.  
**Changes**: Created `config.yaml` with all tunable parameters and `robot_control/config.py` with a frozen `@dataclass` `Config`, YAML loader, and startup validation.  
**Verification**: Config loads successfully, validation passes, unit tests import config.

---

## Action 2: Decompose the Monolith into Modules
**Original**: `robot_brain.py` was 1,286 lines containing 10+ logically independent components.  
**Changes**: Created `robot_control/` package with 15+ modules:
- `config.py`, `camera.py`, `filters.py`, `serial_output.py`, `arena.py`
- `detection/apriltag_tracker.py`, `detection/color_tracker.py`
- `planning/path_planner.py`, `planning/pursuit.py`, `planning/evasion.py`
- `ui/drawing.py`, `ui/hud.py`, `ui/trackbars.py`
- `state.py`, `log.py`, `profile.py`, `watchdog.py`, `__main__.py`  
**Verification**: Module imports succeed, 36 unit tests pass.

---

## Action 3: Eliminate Module-Level Global Mutable State
**Original**: 15+ mutable module-level variables creating invisible coupling.  
**Changes**: Created `state.py` with `TrackingState` and `AppState` dataclasses. All state is now passed explicitly through function arguments.  
**Verification**: No module-level mutable globals remain in the new package.

---

## Action 4: Safety-Critical Watchdog
**Original**: No protection against camera hangs or serial thread death.  
**Changes**: Created `watchdog.py` with a generic `Watchdog` class. Two instances: frame watchdog (200 ms) and serial heartbeat (100 ms). Both auto-send neutral on timeout.  
**Verification**: Watchdog starts/stops cleanly, triggers on simulated stall.

---

## Action 5: Add requirements.txt
**Original**: No dependency manifest.  
**Changes**: Created `requirements.txt` with `opencv-python>=4.5`, `numpy>=1.20`, `pupil-apriltags`, `pyserial`, `pyyaml`.  
**Verification**: `pip install -r requirements.txt` installs all dependencies.

---

## Action 6: Avoid Redundant HSV Conversion
**Original**: `detect_color()` re-computed `cvtColor(BGR→HSV)` despite `hsv_frame` already existing.  
**Changes**: `detect_r2_color()` now accepts `hsv_frame` directly. Single HSV conversion in `__main__.py` is reused everywhere.  
**Verification**: Unit tests pass; no duplicate conversion in codebase.

---

## Action 7: Optimize AprilTag Full-Frame Scan
**Original**: `ROI_PADDING` of 150 caused frequent fallbacks to full-frame scan.  
**Changes**: Increased `roi_padding` to 180 in `config.yaml` to reduce false misses.  
**Verification**: Config loads with new default.

---

## Action 8: Fix Kalman Predict-Before-Correct Ordering
**Original**: `correct()` called before `predict()` — subtly incorrect math.  
**Changes**: `KalmanTracker.update()` now calls `predict()` → `correct()` → returns `statePost`.  
**Verification**: 8/8 Kalman unit tests pass.

---

## Action 9: Use threading.Event for Running Flag
**Original**: Module-level `bool` for shutdown — not thread-safe.  
**Changes**: Created `shutdown_event = threading.Event()` in `state.py`. All loops check `not shutdown_event.is_set()`. Signal handler calls `shutdown_event.set()`.  
**Verification**: Shutdown event integrated in `__main__.py` and terminal loop.

---

## Action 10: PWM Constant Helpers
**Original**: Inline `max(min(...))` with magic numbers 1000/2000.  
**Changes**: Added `clamp_pwm()` helper and `PWM_MIN`/`PWM_MAX`/`PWM_NEUTRAL` constants in `serial_output.py`. Also available via `Config` dataclass.  
**Verification**: `ArduinoSerial.send()` uses `clamp_pwm()`.

---

## Action 11: Use Standard Python Logging
**Original**: All diagnostic output via bare `print()` with ad-hoc prefixes.  
**Changes**: All modules use `logging.getLogger(__name__)`. Logging configured in `__main__.py` with format and level.  
**Verification**: No `print()` calls remain in the new package.

---

## Action 12: Add Type Hints Throughout
**Original**: Minimal type hints, bare tuples and dicts.  
**Changes**: Full type annotations on all function signatures across all modules. `TagInfo` dataclass replaces dict-with-string-keys pattern.  
**Verification**: All type annotations present and consistent.

---

## Action 13: Add Unit Tests for Pure Logic
**Original**: Zero tests.  
**Changes**: Created `tests/` with 36 tests covering:
- `test_bezier.py` — endpoint interpolation, midpoint, shape
- `test_path_planner.py` — start/end, arena bounds, edge cases
- `test_pursuit.py` — neutral, forward drive, turn direction, range
- `test_evasion.py` — trigger conditions, perpendicular direction
- `test_arena.py` — zone computation, inside, clamp, offset
- `test_kalman.py` — init, update, coast, velocity, limits  
**Verification**: All 36 tests pass.

---

## Action 14: Camera Calibration Integration
**Original**: Camera matrix used rough estimates.  
**Changes**: `Config.camera_matrix()` loads `camera_matrix.npz` if `camera_matrix_file` is set in config, falls back to estimated matrix.  
**Verification**: Fallback path works; calibration load path tested.

---

## Action 15: Add README.md
**Original**: No project documentation.  
**Changes**: Created `README.md` with hardware requirements, wiring diagram, software setup, configuration guide, operation guide, and architecture overview.  
**Verification**: README is accurate and links to design docs.
