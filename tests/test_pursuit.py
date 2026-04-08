"""Tests for the pure-pursuit controller."""

import numpy as np
import pytest

from robot_control.config import Config
from robot_control.planning.pursuit import pure_pursuit


@pytest.fixture
def cfg():
    return Config()


class TestPurePursuit:
    def test_neutral_when_no_path(self, cfg):
        drive, steer = pure_pursuit((100, 100), 0.0, None, cfg)
        assert drive == cfg.pwm_neutral
        assert steer == cfg.pwm_neutral

    def test_neutral_at_target(self, cfg):
        """When R1 is already at the path end, should return neutral."""
        path = np.array([[100.0, 100.0], [101.0, 101.0]])
        drive, steer = pure_pursuit((100, 100), 0.0, path, cfg)
        assert drive == cfg.pwm_neutral
        assert steer == cfg.pwm_neutral

    def test_forward_drive(self, cfg):
        """Pointing straight at a far target → high drive, near-neutral steer."""
        path = np.array([[100.0, 300.0], [200.0, 300.0], [500.0, 300.0]])
        drive, steer = pure_pursuit((100, 300), 0.0, path, cfg)
        assert drive > cfg.pwm_neutral, "Should be driving forward"

    def test_turn_direction(self, cfg):
        """Target to the right → steer should increase above neutral."""
        path = np.array([[100.0, 100.0], [200.0, 200.0], [300.0, 300.0]])
        # Heading 0° (east), target is to the south-east → positive error
        drive, steer = pure_pursuit((100, 100), 0.0, path, cfg)
        assert steer > cfg.pwm_neutral, "Should be turning right"

    def test_values_in_pwm_range(self, cfg):
        """All outputs must be within [1000, 2000]."""
        path = np.array([[0.0, 0.0], [1000.0, 1000.0]])
        drive, steer = pure_pursuit((0, 0), 90.0, path, cfg)
        assert 1000 <= drive <= 2000
        assert 1000 <= steer <= 2000
