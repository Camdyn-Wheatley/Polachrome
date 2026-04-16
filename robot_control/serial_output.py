"""
serial_output.py — Non-blocking serial writer for the Arduino Nano.

Sends single-character commands at 9600 baud to the Arduino PPM trainer.
The Arduino increments/decrements channel values by STEP per character.

Protocol (single characters, 9600 baud):
  w/s = drive forward/back (CH2 ± STEP)
  a/d = steer left/right   (CH1 ± STEP)
  x/z = weapon on/off      (CH3 = 2000/1000)
  c   = center steer+drive  (CH1,CH2 → neutral)
  ' ' = emergency stop      (all neutral + weapon off)
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
    """Non-blocking serial writer using single-character command protocol.

    Translates absolute PWM values from the control system into the
    character-based protocol expected by the Arduino PPM trainer sketch.
    The Arduino maintains its own channel state and increments/decrements
    by STEP per character received.
    """

    def __init__(self, cfg: Config) -> None:
        self._cfg = cfg
        self._ser: Optional[object] = None
        self._lock = threading.Lock()
        self._queue: deque = deque(maxlen=1)
        self._stop = False
        self._last_send_time: float = time.time()
        self._send_time_lock = threading.Lock()

        # Track what the Arduino's channel state should be, so we can
        # compute the minimal character sequence to reach a target value.
        self._neutral = cfg.pwm_neutral
        self._step = cfg.arduino_step
        self._ch1 = cfg.pwm_neutral   # steer (Arduino side)
        self._ch2 = cfg.pwm_neutral   # drive (Arduino side)
        self._ch3 = cfg.weapon_off_value  # weapon (Arduino side)

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

    def _build_char_command(self, target_ch1: int, target_ch2: int, target_ch3: int) -> bytes:
        """Build a character command string to move from current state toward target."""
        chars: list[str] = []

        # Steer (CH1): 'a' = decrease, 'd' = increase
        diff1 = target_ch1 - self._ch1
        if abs(diff1) >= self._step:
            n = min(abs(diff1) // self._step, 8)  # cap chars per update
            char = "d" if diff1 > 0 else "a"
            chars.extend([char] * n)
            self._ch1 += n * self._step * (1 if diff1 > 0 else -1)
            self._ch1 = clamp_pwm(self._ch1)

        # Drive (CH2): 'w' = increase, 's' = decrease
        diff2 = target_ch2 - self._ch2
        if abs(diff2) >= self._step:
            n = min(abs(diff2) // self._step, 8)
            char = "w" if diff2 > 0 else "s"
            chars.extend([char] * n)
            self._ch2 += n * self._step * (1 if diff2 > 0 else -1)
            self._ch2 = clamp_pwm(self._ch2)

        # Weapon (CH3): absolute set via 'x' (on) or 'z' (off)
        if target_ch3 != self._ch3:
            if target_ch3 >= 1500:
                chars.append("x")
            else:
                chars.append("z")
            self._ch3 = target_ch3

        # If neutral requested and we're close, use 'c' for exact snap
        if (target_ch1 == self._neutral and target_ch2 == self._neutral
                and abs(self._ch1 - self._neutral) < self._step * 2
                and abs(self._ch2 - self._neutral) < self._step * 2):
            chars = ["c"]
            self._ch1 = self._neutral
            self._ch2 = self._neutral

        return "".join(chars).encode() if chars else b""

    def send(self, ch1: int, ch2: int, ch3: int) -> None:
        """Queue a command. Values are absolute PWM (1000–2000 µs).

        Internally translates to the character protocol expected by the Arduino.
        Channel reversal is applied based on config settings.
        """
        ch1 = clamp_pwm(ch1)
        ch2 = clamp_pwm(ch2)
        ch3 = clamp_pwm(ch3)
        ch1, ch2, ch3 = self._apply_reversal(ch1, ch2, ch3)
        msg = self._build_char_command(ch1, ch2, ch3)
        if msg:
            with self._lock:
                self._queue.append(msg)

    def send_neutral(self) -> None:
        """Send neutral on all channels."""
        with self._lock:
            self._queue.append(b" ")  # emergency stop char
        self._ch1 = self._neutral
        self._ch2 = self._neutral
        self._ch3 = self._cfg.weapon_off_value

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
