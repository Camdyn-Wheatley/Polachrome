"""
serial_output.py — Non-blocking serial writer for the Arduino Nano.

Sends direct channel PWM values at 9600 baud to the Arduino PPM trainer.

Protocol (line-based, 9600 baud):
  CH:ch1,ch2,ch3,ch4,ch5,ch6\n
  Each value is a PWM microsecond value clamped to [1000, 2000].
  ch1=steer  ch2=drive  ch3=weapon  ch4=unused  ch5=arm  ch6=enable
"""

from __future__ import annotations

import logging
import threading
import time
from collections import deque
from typing import Optional

from robot_control.config import Config

logger = logging.getLogger(__name__)

# ── PWM helpers ──────────────────────────────────────────────────────────────

_PWM_MIN = 1000
_PWM_MAX = 2000


def clamp_pwm(value: int) -> int:
    """Clamp a PWM value to the valid 1000–2000 µs range."""
    return max(_PWM_MIN, min(_PWM_MAX, int(value)))


# ── Serial availability ─────────────────────────────────────────────────────

try:
    import serial
    import serial.tools.list_ports

    _HAVE_SERIAL = True
except ImportError:
    _HAVE_SERIAL = False
    logger.warning("pyserial not installed — Arduino output disabled")


class ArduinoSerial:
    """Non-blocking serial writer using direct channel value protocol.

    Sends absolute PWM values directly to the Arduino as a comma-separated
    line: ``CH:ch1,ch2,ch3,ch4,ch5,ch6\\n``.
    """

    def __init__(self, cfg: Config) -> None:
        self._cfg = cfg
        self._ser: Optional[object] = None
        self._lock = threading.Lock()
        self._queue: deque = deque(maxlen=1)
        self._stop = False
        self._last_send_time: float = time.time()
        self._send_time_lock = threading.Lock()

        self._neutral = cfg.pwm_neutral

        # Reversal config
        self._reverse_steer = cfg.channel_reverse_steer
        self._reverse_drive = cfg.channel_reverse_drive
        self._reverse_weapon = cfg.channel_reverse_weapon

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
                    self._ser.write(msg)
                    with self._send_time_lock:
                        self._last_send_time = time.time()
                except Exception:
                    pass
            time.sleep(0.02)  # ~50 Hz

    def _apply_reversal(self, ch1: int, ch2: int, ch3: int) -> tuple[int, int, int]:
        """Mirror PWM values around neutral if channel is reversed."""
        if self._reverse_steer:
            ch1 = 2 * self._neutral - ch1
        if self._reverse_drive:
            ch2 = 2 * self._neutral - ch2
        if self._reverse_weapon:
            ch3 = 2 * self._neutral - ch3
        return clamp_pwm(ch1), clamp_pwm(ch2), clamp_pwm(ch3)

    def send(self, ch1: int, ch2: int, ch3: int) -> None:
        """Queue a command. Values are absolute PWM (1000–2000 µs).

        Internally formats as ``CH:ch1,ch2,ch3,ch4,ch5,ch6\\n`` for the
        Arduino direct channel protocol.
        Channel reversal is applied based on config settings.
        """
        ch1 = clamp_pwm(ch1)
        ch2 = clamp_pwm(ch2)
        ch3 = clamp_pwm(ch3)
        ch1, ch2, ch3 = self._apply_reversal(ch1, ch2, ch3)
        msg = f"CH:{ch1},{ch2},{ch3},{self._neutral},1800,2000\n"
        with self._lock:
            self._queue.append(msg.encode())

    def send_neutral(self) -> None:
        """Send neutral on all channels."""
        msg = f"CH:{self._neutral},{self._neutral},1000,{self._neutral},1800,2000\n"
        with self._lock:
            self._queue.append(msg.encode())

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
