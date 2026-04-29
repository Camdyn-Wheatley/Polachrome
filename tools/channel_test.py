#!/usr/bin/env python3
"""
channel_test.py — RC channel slider test tool.

Opens a GUI window with sliders for each RC channel to manually verify
that the Arduino serial link and PPM output are working correctly.

Usage:
    cd /home/camdyn/code/tracking/Control
    .venv/bin/python tools/channel_test.py [--port /dev/ttyUSB0]
"""

from __future__ import annotations

import argparse
import sys
import threading
import time
import tkinter as tk
from tkinter import ttk
from typing import Optional

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
    {"name": "CH1 — Steer",  "default": NEUTRAL},
    {"name": "CH2 — Drive",  "default": NEUTRAL},
    {"name": "CH3 — Weapon", "default": PWM_MIN},
    {"name": "CH4 — Aux",    "default": NEUTRAL},
    {"name": "CH5 — Arm",    "default": 1800},
    {"name": "CH6 — Enable", "default": 2000},
]

NUM_CHANNELS = len(CHANNELS)


def find_arduino_port() -> Optional[str]:
    """Auto-detect the Arduino serial port."""
    ports = [p.device for p in serial.tools.list_ports.comports()]
    return next(
        (p for p in ports if any(x in p for x in ("USB", "ACM", "COM"))),
        None,
    )


class ChannelTestApp:
    """Tkinter GUI with sliders for each RC channel."""

    def __init__(self, port: str, baud: int = 9600) -> None:
        self._ser: Optional[serial.Serial] = None
        self._stop = False

        # Open serial
        try:
            self._ser = serial.Serial(port, baud, timeout=0.1)
            time.sleep(2.0)  # wait for Arduino reset
            print(f"Connected to Arduino on {port} @ {baud}")
        except serial.SerialException as exc:
            print(f"ERROR: Could not open {port}: {exc}")
            sys.exit(1)

        # ── Build GUI ────────────────────────────────────────────────────
        self._root = tk.Tk()
        self._root.title("RC Channel Test")
        self._root.configure(bg="#1e1e2e")
        self._root.resizable(True, False)

        style = ttk.Style()
        style.theme_use("clam")
        style.configure("TScale", background="#1e1e2e", troughcolor="#313244")
        style.configure("TLabel", background="#1e1e2e", foreground="#cdd6f4",
                         font=("monospace", 11))
        style.configure("TButton", background="#45475a", foreground="#cdd6f4",
                         font=("monospace", 10))
        style.configure("Header.TLabel", background="#1e1e2e",
                         foreground="#89b4fa", font=("monospace", 14, "bold"))
        style.configure("Status.TLabel", background="#1e1e2e",
                         foreground="#a6e3a1", font=("monospace", 10))

        # Header
        header = ttk.Label(self._root, text="🎮  RC Channel Test Tool",
                           style="Header.TLabel")
        header.grid(row=0, column=0, columnspan=4, pady=(12, 4), padx=12)

        self._status_var = tk.StringVar(value=f"Connected: {port}")
        status = ttk.Label(self._root, textvariable=self._status_var,
                           style="Status.TLabel")
        status.grid(row=1, column=0, columnspan=4, pady=(0, 8))

        # Channel sliders
        self._sliders: list[tk.IntVar] = []
        self._value_labels: list[ttk.Label] = []

        for i, ch in enumerate(CHANNELS):
            row = i + 2

            # Label
            lbl = ttk.Label(self._root, text=ch["name"], style="TLabel")
            lbl.grid(row=row, column=0, sticky="w", padx=(12, 4), pady=3)

            # Slider
            var = tk.IntVar(value=ch["default"])
            slider = ttk.Scale(self._root, from_=PWM_MIN, to=PWM_MAX,
                               orient="horizontal", variable=var, length=350,
                               command=lambda val, idx=i: self._on_slider(idx))
            slider.grid(row=row, column=1, padx=4, pady=3)
            self._sliders.append(var)

            # Value readout
            val_lbl = ttk.Label(self._root, text=str(ch["default"]),
                                style="TLabel", width=6)
            val_lbl.grid(row=row, column=2, padx=4, pady=3)
            self._value_labels.append(val_lbl)

            # Center button (for steer/drive/aux channels)
            if ch["default"] == NEUTRAL:
                btn = ttk.Button(self._root, text="Center",
                                 command=lambda v=var, idx=i: self._center(v, idx),
                                 style="TButton")
                btn.grid(row=row, column=3, padx=(4, 12), pady=3)

        # ── Bottom buttons ───────────────────────────────────────────────
        btn_frame = tk.Frame(self._root, bg="#1e1e2e")
        btn_frame.grid(row=NUM_CHANNELS + 2, column=0, columnspan=4,
                       pady=(8, 12))

        ttk.Button(btn_frame, text="⏹  All Neutral", style="TButton",
                   command=self._all_neutral).pack(side="left", padx=6)
        ttk.Button(btn_frame, text="⚠  Kill Weapon", style="TButton",
                   command=self._kill_weapon).pack(side="left", padx=6)

        # Serial output label
        self._serial_var = tk.StringVar(value="TX: —")
        serial_lbl = ttk.Label(self._root, textvariable=self._serial_var,
                               style="Status.TLabel")
        serial_lbl.grid(row=NUM_CHANNELS + 3, column=0, columnspan=4,
                        pady=(0, 8))

        # ── Start sender thread ──────────────────────────────────────────
        self._sender_thread = threading.Thread(target=self._sender,
                                               daemon=True)
        self._sender_thread.start()

        self._root.protocol("WM_DELETE_WINDOW", self._on_close)

    def _on_slider(self, idx: int) -> None:
        """Update the value label when a slider moves."""
        val = self._sliders[idx].get()
        self._value_labels[idx].configure(text=str(val))

    def _center(self, var: tk.IntVar, idx: int) -> None:
        """Snap a channel back to neutral."""
        var.set(NEUTRAL)
        self._value_labels[idx].configure(text=str(NEUTRAL))

    def _all_neutral(self) -> None:
        """Reset all channels to their defaults."""
        for i, ch in enumerate(CHANNELS):
            self._sliders[i].set(ch["default"])
            self._value_labels[i].configure(text=str(ch["default"]))

    def _kill_weapon(self) -> None:
        """Force weapon channel to minimum."""
        self._sliders[2].set(PWM_MIN)
        self._value_labels[2].configure(text=str(PWM_MIN))

    def _sender(self) -> None:
        """Background thread that sends channel values at ~25 Hz."""
        while not self._stop:
            if self._ser and self._ser.is_open:
                try:
                    vals = [s.get() for s in self._sliders]
                    msg = "CH:" + ",".join(str(v) for v in vals) + "\n"
                    self._ser.write(msg.encode())
                    self._serial_var.set(f"TX: {msg.strip()}")
                except Exception as exc:
                    self._serial_var.set(f"TX ERROR: {exc}")
            time.sleep(0.04)  # ~25 Hz

    def _on_close(self) -> None:
        """Send neutral and close."""
        self._stop = True
        if self._ser and self._ser.is_open:
            neutral_msg = f"CH:{NEUTRAL},{NEUTRAL},{PWM_MIN},{NEUTRAL},1800,2000\n"
            try:
                self._ser.write(neutral_msg.encode())
                time.sleep(0.1)
            except Exception:
                pass
            self._ser.close()
        self._root.destroy()

    def run(self) -> None:
        """Start the tkinter main loop."""
        self._root.mainloop()


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

    print(f"Using port: {port}")
    app = ChannelTestApp(port, args.baud)
    app.run()


if __name__ == "__main__":
    main()
