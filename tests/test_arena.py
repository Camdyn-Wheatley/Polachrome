"""Tests for ArenaBounds."""

import pytest

from robot_control.arena import ArenaBounds
from robot_control.config import Config


@pytest.fixture
def cfg():
    return Config(
        arena_tl=(50, 50),
        arena_br=(1230, 670),
        robot_offset=40,
    )


@pytest.fixture
def bounds(cfg):
    return ArenaBounds(cfg)


class TestArenaBounds:
    def test_safe_zone_computation(self, bounds):
        assert bounds.safe_tl == (90, 90)
        assert bounds.safe_br == (1190, 630)

    def test_inside_center(self, bounds):
        assert bounds.inside((640, 360))

    def test_inside_corner(self, bounds):
        assert bounds.inside((90, 90))
        assert bounds.inside((1190, 630))

    def test_outside(self, bounds):
        assert not bounds.inside((0, 0))
        assert not bounds.inside((1280, 720))
        assert not bounds.inside((89, 90))

    def test_clamp_inside(self, bounds):
        pt = (400, 300)
        assert bounds.clamp(pt) == pt

    def test_clamp_below(self, bounds):
        x, y = bounds.clamp((0, 0))
        assert x == 90
        assert y == 90

    def test_clamp_above(self, bounds):
        x, y = bounds.clamp((2000, 2000))
        assert x == 1190
        assert y == 630

    def test_change_offset(self, bounds):
        bounds.change_offset(10)
        assert bounds.offset == 50
        assert bounds.safe_tl == (100, 100)
        assert bounds.safe_br == (1180, 620)

    def test_offset_cannot_go_negative(self, bounds):
        bounds.change_offset(-100)
        assert bounds.offset == 0
