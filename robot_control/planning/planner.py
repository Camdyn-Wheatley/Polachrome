"""
planner.py — Pure pursuit path planner for the combat robot.

Calculates the necessary steering and drive motor commands to intercept
the opponent robot while avoiding obstacles, based on the current WorldState.
"""

from __future__ import annotations

import logging
import math
from typing import Tuple

from robot_control.config import Config
from robot_control.state import WorldState

logger = logging.getLogger(__name__)


class Planner:
    """Calculates motor PWM commands based on the unified vision state."""

    def __init__(self, cfg: Config) -> None:
        self.cfg = cfg

    def compute_commands(self, state: WorldState) -> Tuple[int, int, int]:
        """
        Compute the PWM commands to steer toward the opponent.

        Returns
        -------
        (ch1, ch2, ch3)
            ch1: Steering (1000-2000 µs)
            ch2: Drive (1000-2000 µs)
            ch3: Weapon (1000-2000 µs)
        """
        neutral = self.cfg.pwm_neutral
        ch1 = neutral
        ch2 = neutral
        ch3 = self.cfg.weapon_off_value

        if not state.auto_mode:
            return ch1, ch2, ch3

        if not state.robot_pos or not state.opponent_pos or state.robot_heading is None:
            return ch1, ch2, ch3

        # Vector to opponent
        rx, ry = state.robot_pos
        ox, oy = state.opponent_pos

        dx = ox - rx
        dy = oy - ry  # Note: y increases downwards in image space
        distance = math.hypot(dx, dy)
        target_angle = math.atan2(dy, dx)

        # Angle error
        angle_err = target_angle - state.robot_heading
        # Normalize to [-pi, pi]
        angle_err = (angle_err + math.pi) % (2 * math.pi) - math.pi

        # ── Steering (CH1) ───────────────────────────────────────────────────
        # Proportional control based on angle error.
        # angle_err > 0 means opponent is to the "right" (higher angle).
        turn_effort = angle_err / math.pi  # [-1.0, 1.0]
        
        # Boost small errors to overcome static friction, but cap at max_turn_speed
        turn_pwm = int(turn_effort * self.cfg.max_turn_speed * 1.5)
        turn_pwm = max(-self.cfg.max_turn_speed, min(self.cfg.max_turn_speed, turn_pwm))
        ch1 = neutral + turn_pwm

        # ── Drive (CH2) ──────────────────────────────────────────────────────
        # Drive forward if opponent is further than standoff distance.
        if distance > self.cfg.approach_standoff:
            # Slow down driving if we need to turn sharply (>45 deg)
            speed_factor = max(0.0, 1.0 - (abs(angle_err) / (math.pi / 4)))
            
            drive_effort = distance / 300.0  # normalize somewhat to screen pixels
            drive_effort = min(1.0, drive_effort)
            
            drive_pwm = int(drive_effort * self.cfg.max_drive_speed * speed_factor)
            ch2 = neutral + drive_pwm

        # ── Weapon (CH3) ─────────────────────────────────────────────────────
        # Fire weapon if opponent is close AND we are pointing roughly at them.
        if distance <= self.cfg.approach_standoff * 1.5 and abs(angle_err) < math.radians(20):
            ch3 = self.cfg.weapon_on_value

        return ch1, ch2, ch3
