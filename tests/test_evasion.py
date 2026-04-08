"""Tests for the threat evasion logic."""

import math

import pytest

from robot_control.arena import ArenaBounds
from robot_control.config import Config
from robot_control.planning.evasion import evasion_offset


@pytest.fixture
def cfg():
    return Config(
        arena_tl=(0, 0),
        arena_br=(1280, 720),
        robot_offset=0,
        threat_cone_deg=40.0,
        evasion_offset=120,
    )


@pytest.fixture
def bounds(cfg):
    return ArenaBounds(cfg)


class TestEvasionOffset:
    def test_no_trigger_when_slow(self, bounds, cfg):
        """Evasion should not trigger when R2 is moving slowly."""
        result = evasion_offset((400, 400), (200, 400), (0.5, 0.0), bounds, cfg)
        assert result is None

    def test_trigger_when_heading_toward(self, bounds, cfg):
        """R2 heading directly at R1 should trigger evasion."""
        # R2 at (200, 400) moving east at speed 5 toward R1 at (400, 400)
        result = evasion_offset((400, 400), (200, 400), (5.0, 0.0), bounds, cfg)
        assert result is not None

    def test_no_trigger_when_heading_away(self, bounds, cfg):
        """R2 heading away from R1 should not trigger."""
        # R2 at (200, 400) moving west (away from R1 at 400, 400)
        result = evasion_offset((400, 400), (200, 400), (-5.0, 0.0), bounds, cfg)
        assert result is None

    def test_perpendicular_direction(self, bounds, cfg):
        """Dodge point should be perpendicular to R2's velocity."""
        r1 = (400, 400)
        r2 = (200, 400)
        vel = (5.0, 0.0)  # heading east
        result = evasion_offset(r1, r2, vel, bounds, cfg)
        assert result is not None
        # Perpendicular to east is north or south (±90°)
        dx = result[0] - r1[0]
        dy = result[1] - r1[1]
        # The evade point should be offset primarily in the y direction
        assert abs(dy) > abs(dx), "Dodge should be perpendicular to velocity"

    def test_result_in_arena(self, bounds, cfg):
        """Dodge point must be inside the arena."""
        result = evasion_offset((100, 100), (50, 100), (5.0, 0.0), bounds, cfg)
        if result:
            assert bounds.inside(result)
