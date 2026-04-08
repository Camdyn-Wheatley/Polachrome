"""
watchdog.py — Safety-critical watchdog timers for the combat robot.

Monitors frame freshness and serial heartbeat. If either stalls beyond
a configurable threshold, the watchdog triggers a failsafe (neutral
commands + log warning).
"""

from __future__ import annotations

import logging
import threading
import time
from typing import Callable, Optional

logger = logging.getLogger(__name__)


class Watchdog:
    """
    Periodic watchdog that calls a failsafe callback when a monitored
    condition goes stale.

    Usage:
        wd = Watchdog("frame", threshold_ms=200, failsafe=arduino.send_neutral)
        wd.start()
        # In the hot path:
        wd.feed()           # reset the watchdog timer
        # On shutdown:
        wd.stop()
    """

    def __init__(
        self,
        name: str,
        threshold_ms: float,
        failsafe: Callable[[], None],
        check_interval_ms: float = 50.0,
    ) -> None:
        self.name = name
        self._threshold_s = threshold_ms / 1000.0
        self._check_interval_s = check_interval_ms / 1000.0
        self._failsafe = failsafe
        self._last_feed = time.time()
        self._lock = threading.Lock()
        self._stop = False
        self._triggered = False
        self._thread: Optional[threading.Thread] = None

    def feed(self) -> None:
        """Reset the watchdog timer — call this whenever the monitored event occurs."""
        with self._lock:
            self._last_feed = time.time()
            if self._triggered:
                logger.info("[%s] watchdog recovered", self.name)
                self._triggered = False

    def start(self) -> None:
        """Start the background watchdog thread."""
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        logger.info("[%s] watchdog started (threshold=%.0fms)", self.name, self._threshold_s * 1000)

    def stop(self) -> None:
        """Stop the watchdog thread."""
        self._stop = True
        if self._thread:
            self._thread.join(timeout=1.0)

    def _run(self) -> None:
        while not self._stop:
            with self._lock:
                age = time.time() - self._last_feed
                triggered = self._triggered
            if age > self._threshold_s and not triggered:
                logger.warning(
                    "[%s] watchdog TRIGGERED — %.0fms since last feed (threshold %.0fms)",
                    self.name,
                    age * 1000,
                    self._threshold_s * 1000,
                )
                with self._lock:
                    self._triggered = True
                try:
                    self._failsafe()
                except Exception as exc:
                    logger.error("[%s] watchdog failsafe error: %s", self.name, exc)
            time.sleep(self._check_interval_s)
