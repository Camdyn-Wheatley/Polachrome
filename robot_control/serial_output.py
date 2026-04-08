"""
serial_output.py — Non-blocking serial writer for the Arduino Nano.

Sends CH1 (steer), CH2 (drive), CH3 (weapon) packets at ~50 Hz.
"""

from __future__ import annotations

import logging
import threading
import time
from collections import deque
from typing import Optional

from robot_control.config import Config

logger = logging.getLogger(__name__)

# ── PWM helpers (Action 10) ──────────────────────────────────────────────────

_PWM_MIN = 1000
_PWM_MAX = 2000
_PWM_NEUTRAL = 1500


def clamp_pwm(value: int) -> int:
    """Clamp a PWM value to the valid 1000–2000 µs range."""
    return max(_PWM_MIN, min(_PWM_MAX, int(value)))


# ── Serial availability ──────────────────────────────────────────────────────

try:
    import serial
    import serial.tools.list_ports

    _HAVE_SERIAL = True
except ImportError:
    _HAVE_SERIAL = False
    logger.warning("pyserial not installed — Arduino output disabled")


class ArduinoSerial:
    """Non-blocking serial writer with latest-only command queue."""

    def __init__(self, cfg: Config) -> None:
        self._cfg = cfg
        self._ser: Optional[object] = None
        self._lock = threading.Lock()
        self._queue: deque = deque(maxlen=1)
        self._stop = False
        self._last_send_time: float = time.time()
        self._send_time_lock = threading.Lock()

        if not _HAVE_SERIAL:
            logger.warning("pyserial missing — serial output disabled")
            return

        port = cfg.arduino_port
        if port is None:
            ports = [p.device for p in serial.tools.list_ports.comports()]
            port = next(
                (p for p in ports if any(x in p for x in ("USB", "ACM", "COM"))),
                None,
            )

        if port is None:
            logger.warning("No Arduino port found — serial output disabled")
            return

        try:
            self._ser = serial.Serial(port, cfg.arduino_baud, timeout=0.1)
            time.sleep(2.0)  # wait for Arduino reset after DTR
            logger.info("Arduino connected on %s @ %d", port, cfg.arduino_baud)
        except serial.SerialException as exc:
            logger.error("Could not open %s: %s", port, exc)
            self._ser = None
            return

        self._thread = threading.Thread(target=self._sender, daemon=True)
        self._thread.start()

    def _sender(self) -> None:
        while not self._stop:
            msg = None
            with self._lock:
                if self._queue:
                    msg = self._queue.pop()
            if msg and self._ser and self._ser.is_open:
                try:
                    self._ser.write(msg.encode())
                    with self._send_time_lock:
                        self._last_send_time = time.time()
                except Exception:
                    pass
            time.sleep(0.02)  # ~50 Hz

    def send(self, ch1: int, ch2: int, ch3: int) -> None:
        """Queue a command. Values are clamped to 1000–2000 µs."""
        ch1 = clamp_pwm(ch1)
        ch2 = clamp_pwm(ch2)
        ch3 = clamp_pwm(ch3)
        with self._lock:
            self._queue.append(f"{ch1},{ch2},{ch3}\n")

    def send_neutral(self) -> None:
        """Send neutral on all channels."""
        self.send(_PWM_NEUTRAL, _PWM_NEUTRAL, self._cfg.weapon_off_value)

    @property
    def last_send_age_ms(self) -> float:
        """Milliseconds since the last command was actually written to serial."""
        with self._send_time_lock:
            return (time.time() - self._last_send_time) * 1000.0

    def close(self) -> None:
        """Stop the sender thread and close the serial port."""
        self._stop = True
        if self._ser and self._ser.is_open:
            self.send_neutral()
            time.sleep(0.15)
            self._ser.close()
        logger.info("Arduino serial closed")
