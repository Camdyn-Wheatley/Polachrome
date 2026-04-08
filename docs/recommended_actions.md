# Recommended Actions — Maintainability & Performance

Prioritized list of changes, ordered by impact. Items within each tier are roughly ordered by effort-to-value ratio (quick wins first).

---

## 🔴 High Priority — Do These First

### 1. Extract Configuration to a Separate File

**Problem**: 40+ constants are hard-coded at the top of `robot_brain.py`. Changing the arena size, camera device, or tuning parameters requires editing the source code.

**Action**:
- Create a `config.yaml` (or `config.json`) that holds all tunable parameters.
- Create a `config.py` module with a frozen `@dataclass` that loads from the YAML and provides typed, validated access.
- Validate ranges at startup (e.g., `FRAME_WIDTH > 0`, `MAX_COAST_FRAMES >= 1`).

**Impact**: Eliminates accidental code changes during tuning. Enables per-arena or per-event configs.
**Effort**: ~2 hours.

---

### 2. Decompose the Monolith into Modules

**Problem**: `robot_brain.py` is 1,286 lines containing 10+ logically independent components. `unified_tracking.py` is a dead-weight 793-line predecessor with ~60% code duplication.

**Action**: Reorganize into a proper package:

```
robot_control/
├── __init__.py
├── __main__.py              # entry point
├── config.py                # Config dataclass + loader
├── camera.py                # CameraStream
├── detection/
│   ├── apriltag_tracker.py  # AprilTag detection, ROI, midpoint, heading
│   └── color_tracker.py     # R2 color tracker + R1 color fallback
├── filters.py               # KalmanTracker
├── planning/
│   ├── path_planner.py      # Bézier path builder
│   ├── pursuit.py           # Pure-pursuit controller
│   └── evasion.py           # Threat evasion
├── arena.py                 # ArenaBounds
├── serial_output.py         # ArduinoSerial
├── ui/
│   ├── drawing.py           # All cv2 drawing functions
│   ├── trackbars.py         # HSV tuning trackbar management
│   └── hud.py               # HUD overlay
└── logging.py               # CSV position logger
```

- Delete or archive `unified_tracking.py` (its functionality is fully superseded by `robot_brain.py`).

**Impact**: Each module is independently testable, reviewable, and replaceable.
**Effort**: ~4–6 hours.

---

### 3. Eliminate Module-Level Global Mutable State

**Problem**: 15+ mutable module-level variables (`kf1`, `kf2`, `arena`, `hsv_lower`, `running`, `_roi`, `_tag_offset`, `_cam_ref`, etc.) create invisible coupling. Any function can silently depend on or mutate shared state.

**Action**:
- Create an `AppState` class (or a few focused dataclasses) that holds all runtime state.
- Pass state explicitly through function arguments or as `self` on class methods.
- The `running` flag should be an `threading.Event` instead of a bare `bool` (which isn't thread-safe on its own for signal handlers).

**Example**:
```python
@dataclass
class TrackingState:
    kf1: KalmanTracker
    kf2: KalmanTracker
    arena: ArenaBounds
    r2_color: ColorProfile
    r1_color: Robot1ColorTracker
    roi_cache: Dict[int, Optional[Tuple]]
    tag_offsets: Dict[int, Tuple]
```

**Impact**: Makes data flow explicit; enables unit testing without monkeypatching globals.
**Effort**: ~3–4 hours (can be done alongside module decomposition).

---

### 4. Add Safety-Critical Watchdog

**Problem**: If the camera thread hangs, the vision pipeline returns stale data, or the serial thread dies, the robot could drive uncontrolled. The only safeguard is the Kalman coast counter (30 frames ≈ 0.5 seconds at 60 FPS).

**Action**:
- Add a frame-timestamp watchdog: if no new frame arrives for >200 ms, auto-send neutral and log a warning.
- Add a serial heartbeat: if the sender thread hasn't sent a command in >100 ms, trigger a failsafe.
- Consider a hardware watchdog on the Arduino side (if the PPM signal stops, the Arduino should auto-neutral).

**Impact**: Critical for a combat robot — prevents uncontrolled driving on software failure.
**Effort**: ~2 hours.

---

### 5. Add a `requirements.txt` / `pyproject.toml`

**Problem**: No dependency manifest exists. Dependencies are only discoverable by reading `import` statements and `try/except` blocks.

**Action**:
```
# requirements.txt
opencv-python>=4.5
numpy>=1.20
pupil-apriltags
pyserial
pyyaml
```

**Impact**: One-command setup for new machines / teammates.
**Effort**: 15 minutes.

---

## 🟡 Medium Priority — Performance & Robustness

### 6. Avoid Redundant HSV Conversion

**Problem**: `detect_color()` (line 834) calls `cv2.cvtColor(frame, COLOR_BGR2HSV)` internally, but `main()` already computed `hsv_frame` two lines earlier (line 1127). This is a full-frame conversion that runs every iteration for nothing.

**Action**: Pass `hsv_frame` into `detect_color()` instead of re-converting.

**Impact**: Saves ~1–2 ms per frame (non-trivial at 60 FPS).
**Effort**: 10 minutes.

---

### 7. Profile and Optimize the AprilTag Full-Frame Scan

**Problem**: The full-frame grayscale conversion + AprilTag detect on 1280×720 is the most expensive operation. The ROI strategy helps, but the full-frame fallback still triggers frequently (every time a tag is briefly lost for 6+ frames).

**Action**:
- Increase `ROI_PADDING` slightly to reduce false misses.
- Consider detecting in a downscaled frame on the full scan (the `quad_decimate` is already 2.0, but you could additionally resize the grayscale to 640×360 before the fallback scan).
- Run full-frame detection in a background thread with a one-frame delay, while the main thread uses ROI results.

**Impact**: Could reclaim 5–15 ms on frames that trigger the full scan.
**Effort**: 1–3 hours depending on the approach.

---

### 8. Fix Kalman Predict-Before-Correct Ordering

**Problem**: In `update()`, `correct()` is called before `predict()`. The OpenCV Kalman filter expects `predict()` → `correct()` ordering. Calling `correct` before the first `predict` after init is technically working because `errorCovPost` is pre-set, but the math is subtly off for every subsequent frame — the prior is the old predicted state rather than the new one.

**Action**: Change update to `predict()` → `correct()` → return `statePost`.

**Impact**: Slightly more accurate position estimates.
**Effort**: 30 minutes.

---

### 9. Use `threading.Event` for the `running` Flag

**Problem**: The `running` global `bool` is read by the main thread and written by the signal handler and terminal thread. While CPython's GIL makes this "accidentally safe," it's undefined behavior per the Python memory model and will break on alternative interpreters.

**Action**: Replace `running = True` with `shutdown_event = threading.Event()`. Check `not shutdown_event.is_set()` in loops. Call `shutdown_event.set()` in signal handlers.

**Impact**: Correct cross-thread signaling.
**Effort**: 30 minutes.

---

### 10. Batch Serial Clamping into a Helper

**Problem**: The `send()` method clamps CH1/CH2/CH3 individually with inline `max(min(...))`. The same clamping pattern appears nowhere else, but the magic numbers 1000/2000 are repeated.

**Action**: Define `PWM_MIN = 1000`, `PWM_MAX = 2000`, `PWM_NEUTRAL = 1500` constants and a `clamp_pwm(v: int) -> int` helper.

**Impact**: DRY; makes the valid range self-documenting.
**Effort**: 15 minutes.

---

### 11. Use Standard Python Logging

**Problem**: All diagnostic output uses bare `print()` with ad-hoc `[TAG]` prefixes. No way to control verbosity, filter by subsystem, or redirect to a file.

**Action**: Replace `print()` calls with `logging.getLogger(__name__).info/debug/warning` and configure a format + level at startup.

**Impact**: Enables debug-level verbosity during development, clean operation in competition.
**Effort**: 1–2 hours.

---

## 🟢 Lower Priority — Developer Experience & Long-Term

### 12. Add Type Hints Throughout

**Problem**: Only a few functions have type hints. Most use bare tuples, dicts with string keys, and optional returns without annotations.

**Action**:
- Add type hints to all function signatures and key variables.
- Use `NamedTuple` or `@dataclass` for structured data (e.g., `TagInfo` instead of a `dict` with string keys like `"center"`, `"corners_2d"`, `"rvec"`).
- Run `mypy --strict` in CI.

**Impact**: Catches bugs at edit time; makes the codebase self-documenting.
**Effort**: 2–3 hours.

---

### 13. Add Unit Tests for Pure Logic

**Problem**: Zero tests exist. Any refactoring is a high-risk activity.

**Action**: Write tests for the functions that don't require hardware:
- `_cubic_bezier` — verify endpoint interpolation and midpoint.
- `build_path` — verify clamping, geometry, and edge cases (R1 == R2).
- `pure_pursuit` — verify neutral at target, correct turn direction.
- `evasion_offset` — verify trigger and perpendicular direction.
- `ArenaBounds.clamp` / `inside` — boundary conditions.
- `KalmanTracker` — basic update/predict cycle.

**Impact**: Enables confident refactoring.
**Effort**: 3–4 hours.

---

### 14. Camera Calibration Integration

**Problem**: The camera matrix uses rough estimates (`fx = fy = FRAME_WIDTH`). `color_track.py` is a standalone calibration script that doesn't feed its results back into the main application.

**Action**:
- Run calibration, save `camera_matrix.npz`.
- Load the calibration in `robot_brain.py` at startup and fall back to the estimate if the file doesn't exist.

**Impact**: More accurate `solvePnP` → better heading estimates → smoother paths.
**Effort**: 1 hour.

---

### 15. Add a README.md

**Problem**: No project documentation exists. Setup, wiring, and operation are only discoverable from source code comments.

**Action**: Create a `README.md` covering:
- Hardware requirements and wiring diagram.
- Software setup (`pip install -r requirements.txt`).
- Configuration (`config.yaml` parameters).
- Operation guide (startup, color picking, autonomous mode).
- Architecture overview (link to design doc).

**Impact**: Onboarding; future-you will thank present-you.
**Effort**: 1–2 hours.

---

## Summary Table

| # | Action | Category | Effort | Impact |
|---|--------|----------|--------|--------|
| 1 | Extract config to YAML | Maintainability | 2h | 🔴 High |
| 2 | Decompose into modules | Maintainability | 4–6h | 🔴 High |
| 3 | Eliminate global state | Maintainability | 3–4h | 🔴 High |
| 4 | Safety watchdog | Safety | 2h | 🔴 High |
| 5 | requirements.txt | Setup | 15m | 🔴 High |
| 6 | Remove redundant HSV conversion | Performance | 10m | 🟡 Med |
| 7 | Optimize full-frame tag scan | Performance | 1–3h | 🟡 Med |
| 8 | Fix Kalman predict/correct order | Correctness | 30m | 🟡 Med |
| 9 | `threading.Event` for running | Correctness | 30m | 🟡 Med |
| 10 | PWM constant helpers | Maintainability | 15m | 🟡 Med |
| 11 | Python logging module | Maintainability | 1–2h | 🟡 Med |
| 12 | Type hints + dataclasses | Maintainability | 2–3h | 🟢 Low |
| 13 | Unit tests | Reliability | 3–4h | 🟢 Low |
| 14 | Camera calibration integration | Accuracy | 1h | 🟢 Low |
| 15 | README.md | Documentation | 1–2h | 🟢 Low |
