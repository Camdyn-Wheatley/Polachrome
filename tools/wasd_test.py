#!/usr/bin/env python3
"""
wasd_test.py — Manual WASD driving test for the combat robot.

Sends single-character commands to the Arduino PPM trainer at 9600 baud.
The Arduino increments/decrements channel values by STEP per character.

Usage:
    source .venv/bin/activate
    python3 tools/wasd_test.py [--port /dev/ttyUSB0]

Controls:
    W / S    — Drive forward / backward
    A / D    — Steer left / right
    X        — Weapon ON
    Z        — Weapon OFF
    SPACE    — Emergency stop (all neutral)
    Q / ESC  — Quit
"""

from __future__ import annotations

import argparse
import os
import time
from pathlib import Path
from typing import Optional

import cv2
import numpy as np

# ── Serial setup ─────────────────────────────────────────────────────────────

try:
    import serial
    import serial.tools.list_ports

    _HAVE_SERIAL = True
except ImportError:
    _HAVE_SERIAL = False


def find_arduino(preferred_port: Optional[str] = None) -> Optional[str]:
    """Auto-detect the Arduino serial port."""
    if preferred_port:
        return preferred_port
    if not _HAVE_SERIAL:
        return None
    ports = [p.device for p in serial.tools.list_ports.comports()]
    return next(
        (p for p in ports if any(x in p for x in ("USB", "ACM", "COM"))),
        None,
    )


# ── Constants (must match Arduino sketch) ────────────────────────────────────

PWM_MIN = 1000
PWM_MAX = 2000
PWM_NEUTRAL = 1486
WEAPON_OFF = 1000
WEAPON_ON = 2000

# Must match Arduino STEP define
ARDUINO_STEP = 5

# How many direction characters to send per Python frame.
CHARS_PER_FRAME = 4

# Channel reversal (loaded from config.yaml if present)
REVERSE_STEER = False
REVERSE_DRIVE = False
REVERSE_WEAPON = False


def _load_reversal_from_config() -> None:
    """Load channel reversal settings from config.yaml if available."""
    global REVERSE_STEER, REVERSE_DRIVE, REVERSE_WEAPON
    config_path = Path(__file__).resolve().parent.parent / "config.yaml"
    if not config_path.exists():
        return
    try:
        import yaml
        with open(config_path) as f:
            raw = yaml.safe_load(f) or {}
        REVERSE_STEER = bool(raw.get("channel_reverse_steer", False))
        REVERSE_DRIVE = bool(raw.get("channel_reverse_drive", False))
        REVERSE_WEAPON = bool(raw.get("channel_reverse_weapon", False))
        if any([REVERSE_STEER, REVERSE_DRIVE, REVERSE_WEAPON]):
            print(f"[CFG] Reversals: steer={REVERSE_STEER} drive={REVERSE_DRIVE} weapon={REVERSE_WEAPON}")
    except Exception as e:
        print(f"[CFG] Could not load config.yaml: {e}")


def clamp(v: int) -> int:
    return max(PWM_MIN, min(PWM_MAX, v))


# ── Display ──────────────────────────────────────────────────────────────────

WIN = "WASD Test — Robot Drive"
WIDTH = 520
HEIGHT = 400


def draw_panel(
    ch1: int, ch2: int, ch3: int,
    port: Optional[str], connected: bool,
    fps: float,
) -> np.ndarray:
    """Render the control display panel."""
    panel = np.full((HEIGHT, WIDTH, 3), (30, 30, 30), dtype=np.uint8)
    font = cv2.FONT_HERSHEY_SIMPLEX

    # Title
    cv2.putText(panel, "WASD Robot Drive Test", (20, 40), font, 0.8, (0, 200, 255), 2)

    # Connection status
    if connected:
        status = f"Connected: {port} @ 9600"
        color = (0, 255, 80)
    else:
        status = "NOT CONNECTED — no Arduino found"
        color = (0, 0, 255)
    cv2.putText(panel, status, (20, 75), font, 0.5, color, 1)

    # Channel bars
    bar_y = 110
    for label, val, bar_color in [
        ("CH1  Steer", ch1, (255, 200, 0)),
        ("CH2  Drive", ch2, (0, 200, 255)),
        ("CH3  Weapon", ch3, (0, 100, 255)),
    ]:
        # Label + value
        cv2.putText(panel, label, (20, bar_y + 18), font, 0.5, (200, 200, 200), 1)
        cv2.putText(panel, str(val), (440, bar_y + 18), font, 0.5, (0, 255, 200), 1)

        # Bar background
        bx1, bx2 = 160, 430
        by1, by2 = bar_y + 2, bar_y + 24
        cv2.rectangle(panel, (bx1, by1), (bx2, by2), (60, 60, 60), -1)

        # Bar fill (centred on neutral)
        center_x = bx1 + (bx2 - bx1) // 2
        fill_frac = (val - PWM_MIN) / (PWM_MAX - PWM_MIN)
        fill_x = int(bx1 + fill_frac * (bx2 - bx1))

        if val >= PWM_NEUTRAL:
            cv2.rectangle(panel, (center_x, by1), (fill_x, by2), bar_color, -1)
        else:
            cv2.rectangle(panel, (fill_x, by1), (center_x, by2), bar_color, -1)

        # Center marker
        cv2.line(panel, (center_x, by1 - 2), (center_x, by2 + 2), (180, 180, 180), 1)

        bar_y += 50

    # Controls help
    help_y = 290
    cv2.putText(panel, "Controls:", (20, help_y), font, 0.5, (180, 180, 180), 1)
    helps = [
        "W/S = Drive fwd/back",
        "A/D = Steer left/right",
        "X/Z = Weapon on/off",
        "SPACE = Emergency stop",
        "Q/ESC = Quit",
    ]
    for i, h in enumerate(helps):
        cv2.putText(panel, h, (30, help_y + 22 + i * 20), font, 0.4, (140, 140, 140), 1)

    cv2.putText(panel, f"FPS: {int(fps)}", (420, HEIGHT - 15), font, 0.4, (100, 100, 100), 1)

    return panel


# ── Main ─────────────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(description="WASD Robot Drive Test")
    parser.add_argument("--port", type=str, default=None, help="Serial port (auto-detect if omitted)")
    args = parser.parse_args()

    # Load channel reversal from config.yaml
    _load_reversal_from_config()

    # Direction chars — swapped when channel is reversed so user controls
    # are always intuitive (W=forward, D=right) regardless of wiring.
    fwd_char = "s" if REVERSE_DRIVE else "w"
    bck_char = "w" if REVERSE_DRIVE else "s"
    rgt_char = "a" if REVERSE_STEER else "d"
    lft_char = "d" if REVERSE_STEER else "a"

    # Open serial at 9600 — must match Arduino sketch
    port = find_arduino(args.port)
    ser = None
    if port and _HAVE_SERIAL:
        try:
            ser = serial.Serial(port, 9600, timeout=0.1)
            time.sleep(2.0)  # wait for Arduino reset
            print(f"[OK] Connected to {port} @ 9600")
        except Exception as e:
            print(f"[ERR] Could not open {port}: {e}")
            ser = None
    else:
        print("[WARN] No Arduino found — running in display-only mode")

    # Create window
    cv2.namedWindow(WIN, cv2.WINDOW_AUTOSIZE)

    # Local tracking of channel values (mirrors Arduino state for display)
    ch1 = PWM_NEUTRAL   # steer
    ch2 = PWM_NEUTRAL   # drive
    ch3 = WEAPON_OFF    # weapon

    last_t = time.time()

    # Key-held tracking via timeout
    HOLD_TIMEOUT = 0.15  # seconds
    key_last_seen: dict[str, float] = {}

    def key_held(k: str) -> bool:
        return (time.time() - key_last_seen.get(k, 0)) < HOLD_TIMEOUT

    print("Ready — focus the WASD Test window and use keyboard controls")

    while True:
        now = time.time()
        dt = max(now - last_t, 1e-4)
        fps = 1.0 / dt
        last_t = now

        # ── Read all pending keys this frame ─────────────────────────
        quit_requested = False
        for _ in range(10):
            key = cv2.waitKey(2) & 0xFF
            if key == 255:
                break
            if key == 27 or chr(key).lower() == "q":
                quit_requested = True
                break
            elif key == ord(" "):
                ch1 = PWM_NEUTRAL
                ch2 = PWM_NEUTRAL
                ch3 = WEAPON_OFF
                key_last_seen.clear()
                if ser and ser.is_open:
                    ser.write(b" ")
            elif chr(key).lower() == "x":
                ch3 = WEAPON_ON
                if ser and ser.is_open:
                    ser.write(b"x")
            elif chr(key).lower() == "z":
                ch3 = WEAPON_OFF
                if ser and ser.is_open:
                    ser.write(b"z")
            elif chr(key).lower() in "wasd":
                key_last_seen[chr(key).lower()] = now

        if quit_requested:
            break

        # ── Build character commands for held direction keys ──────────
        chars: list[str] = []

        if key_held("w"):
            chars.extend([fwd_char] * CHARS_PER_FRAME)
            ch2 = clamp(ch2 + ARDUINO_STEP * CHARS_PER_FRAME)
        elif key_held("s"):
            chars.extend([bck_char] * CHARS_PER_FRAME)
            ch2 = clamp(ch2 - ARDUINO_STEP * CHARS_PER_FRAME)
        else:
            # Steer/drive released — snap to neutral (display)
            if ch2 != PWM_NEUTRAL:
                chars.append("c")
                ch2 = PWM_NEUTRAL
                ch1 = PWM_NEUTRAL  # 'c' centers both axes

        if key_held("d"):
            chars.extend([rgt_char] * CHARS_PER_FRAME)
            ch1 = clamp(ch1 + ARDUINO_STEP * CHARS_PER_FRAME)
        elif key_held("a"):
            chars.extend([lft_char] * CHARS_PER_FRAME)
            ch1 = clamp(ch1 - ARDUINO_STEP * CHARS_PER_FRAME)
        else:
            if ch1 != PWM_NEUTRAL and "c" not in chars:
                chars.append("c")
                ch1 = PWM_NEUTRAL
                ch2 = PWM_NEUTRAL

        # ── Send to Arduino ──────────────────────────────────────────
        if ser and ser.is_open and chars:
            try:
                ser.write("".join(chars).encode())
            except Exception:
                pass

        # Read any Arduino debug output
        if ser and ser.is_open and ser.in_waiting > 0:
            try:
                incoming = ser.read(ser.in_waiting).decode(errors="ignore")
                for ln in incoming.splitlines():
                    if ln.strip():
                        print(f"[ARDUINO] {ln.strip()}")
            except Exception:
                pass

        # ── Draw ─────────────────────────────────────────────────────
        panel = draw_panel(ch1, ch2, ch3, port, ser is not None and ser.is_open, fps)
        cv2.imshow(WIN, panel)

    # ── Cleanup ──────────────────────────────────────────────────────
    if ser and ser.is_open:
        time.sleep(0.05)
        if ser.in_waiting > 0:
            ser.read(ser.in_waiting)
        for _ in range(3):
            ser.write(b" ")  # emergency stop
            ser.flush()
            time.sleep(0.05)
        ser.close()
    cv2.destroyAllWindows()
    print("[EXIT] Test complete")


if __name__ == "__main__":
    main()
