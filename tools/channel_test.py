#!/usr/bin/env python3
"""
channel_test.py — RC channel slider test tool.

Opens an OpenCV window with trackbars for each RC channel to manually verify
that the Arduino serial link and PPM output are working correctly.

Usage:
    cd /home/camdyn/code/tracking/Control
    .venv/bin/python tools/channel_test.py [--port /dev/ttyUSB0]

Controls:
    Sliders     Adjust each channel (1000–2000 µs)
    N           All channels to neutral/default
    W           Kill weapon (CH3 → 1000)
    Q / ESC     Quit (sends neutral first)
"""

from __future__ import annotations

import argparse
import sys
import threading
import time
from typing import Optional

import cv2
import numpy as np

# ── Serial setup ─────────────────────────────────────────────────────────────

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("ERROR: pyserial is not installed. Run: pip install pyserial")
    sys.exit(1)

NEUTRAL = 1486
PWM_MIN = 1000
PWM_MAX = 2000

CHANNELS = [
    {"name": "CH1 Steer",  "default": NEUTRAL},
    {"name": "CH2 Drive",  "default": NEUTRAL},
    {"name": "CH3 Weapon", "default": PWM_MIN},
    {"name": "CH4 Aux",    "default": NEUTRAL},
    {"name": "CH5 Arm",    "default": 1800},
    {"name": "CH6 Enable", "default": 2000},
]

NUM_CHANNELS = len(CHANNELS)
WIN_NAME = "RC Channel Test"


def find_arduino_port() -> Optional[str]:
    """Auto-detect the Arduino serial port."""
    ports = [p.device for p in serial.tools.list_ports.comports()]
    return next(
        (p for p in ports if any(x in p for x in ("USB", "ACM", "COM"))),
        None,
    )


def draw_panel(values: list[int], port: str, tx_msg: str) -> np.ndarray:
    """Render a dark info panel showing current channel values."""
    w, h = 600, 340
    img = np.zeros((h, w, 3), dtype=np.uint8)
    img[:] = (30, 30, 46)  # dark background

    # Header
    cv2.putText(img, "RC Channel Test Tool", (20, 35),
                cv2.FONT_HERSHEY_SIMPLEX, 0.9, (137, 180, 250), 2)
    cv2.putText(img, f"Port: {port}", (20, 60),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (166, 227, 161), 1)

    # Channel bars
    bar_x = 160
    bar_w = 380
    for i, ch in enumerate(CHANNELS):
        y = 90 + i * 36
        val = values[i]
        pct = (val - PWM_MIN) / (PWM_MAX - PWM_MIN)

        # Label
        cv2.putText(img, ch["name"], (20, y + 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (205, 214, 244), 1)

        # Bar background
        cv2.rectangle(img, (bar_x, y - 10), (bar_x + bar_w, y + 10),
                      (49, 50, 68), -1)

        # Neutral marker
        neutral_x = bar_x + int((NEUTRAL - PWM_MIN) / (PWM_MAX - PWM_MIN) * bar_w)
        cv2.line(img, (neutral_x, y - 12), (neutral_x, y + 12),
                 (88, 91, 112), 1)

        # Fill bar
        fill_w = int(pct * bar_w)
        color = (166, 227, 161) if abs(val - NEUTRAL) < 20 else (250, 179, 135)
        if ch["name"] == "CH3 Weapon" and val > 1500:
            color = (131, 139, 243)  # red-ish for weapon on
        cv2.rectangle(img, (bar_x, y - 8), (bar_x + fill_w, y + 8),
                      color, -1)

        # Value text
        cv2.putText(img, str(val), (bar_x + bar_w + 10, y + 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (205, 214, 244), 1)

    # TX output
    cv2.putText(img, f"TX: {tx_msg}", (20, h - 40),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (166, 227, 161), 1)

    # Controls help
    cv2.putText(img, "N=Neutral  W=Kill Weapon  Q=Quit", (20, h - 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (88, 91, 112), 1)

    return img


def main() -> None:
    parser = argparse.ArgumentParser(description="RC Channel Slider Test Tool")
    parser.add_argument("--port", type=str, default=None,
                        help="Arduino serial port (auto-detect if omitted)")
    parser.add_argument("--baud", type=int, default=9600,
                        help="Serial baud rate (default: 9600)")
    args = parser.parse_args()

    port = args.port or find_arduino_port()
    if port is None:
        print("ERROR: No Arduino found. Use --port to specify manually.")
        sys.exit(1)

    # Open serial
    try:
        ser = serial.Serial(port, args.baud, timeout=0.1)
        time.sleep(2.0)  # wait for Arduino reset
        print(f"Connected to Arduino on {port} @ {args.baud}")
    except serial.SerialException as exc:
        print(f"ERROR: Could not open {port}: {exc}")
        sys.exit(1)

    # Create window with trackbars
    cv2.namedWindow(WIN_NAME, cv2.WINDOW_AUTOSIZE)

    for ch in CHANNELS:
        cv2.createTrackbar(ch["name"], WIN_NAME, ch["default"] - PWM_MIN,
                           PWM_MAX - PWM_MIN, lambda _: None)

    tx_msg = "—"
    print("Controls: N=Neutral  W=Kill Weapon  Q/ESC=Quit")

    try:
        while True:
            # Read slider values
            values = []
            for ch in CHANNELS:
                raw = cv2.getTrackbarPos(ch["name"], WIN_NAME)
                values.append(raw + PWM_MIN)

            # Send to Arduino
            msg = "CH:" + ",".join(str(v) for v in values) + "\n"
            try:
                ser.write(msg.encode())
                tx_msg = msg.strip()
            except Exception as exc:
                tx_msg = f"ERROR: {exc}"

            # Draw info panel
            panel = draw_panel(values, port, tx_msg)
            cv2.imshow(WIN_NAME, panel)

            # Handle keys
            key = cv2.waitKey(40) & 0xFF  # ~25 Hz
            if key in (ord("q"), ord("Q"), 27):  # Q or ESC
                break
            elif key in (ord("n"), ord("N")):
                # Reset all to defaults
                for i, ch in enumerate(CHANNELS):
                    cv2.setTrackbarPos(ch["name"], WIN_NAME,
                                       ch["default"] - PWM_MIN)
            elif key in (ord("w"), ord("W")):
                # Kill weapon
                cv2.setTrackbarPos("CH3 Weapon", WIN_NAME, 0)

    finally:
        # Send neutral before closing
        neutral_msg = f"CH:{NEUTRAL},{NEUTRAL},{PWM_MIN},{NEUTRAL},1800,2000\n"
        try:
            ser.write(neutral_msg.encode())
            time.sleep(0.1)
        except Exception:
            pass
        ser.close()
        cv2.destroyAllWindows()
        print("Closed.")


if __name__ == "__main__":
    main()
