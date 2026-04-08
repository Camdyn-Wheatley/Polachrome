"""Tests for the path planner."""

import math

import numpy as np
import pytest

from robot_control.arena import ArenaBounds
from robot_control.config import Config
from robot_control.planning.path_planner import build_path


@pytest.fixture
def cfg():
    return Config(
        arena_tl=(50, 50),
        arena_br=(1230, 670),
        robot_offset=40,
        approach_standoff=80,
    )


@pytest.fixture
def bounds(cfg):
    return ArenaBounds(cfg)


class TestBuildPath:
    def test_path_not_empty(self, bounds, cfg):
        path = build_path((200, 300), (600, 400), (900, 300), bounds, cfg)
        assert len(path) > 0

    def test_path_starts_near_r1(self, bounds, cfg):
        r1 = (200, 300)
        path = build_path(r1, (600, 400), (900, 300), bounds, cfg)
        start = path[0]
        dist = math.hypot(start[0] - r1[0], start[1] - r1[1])
        assert dist < 5.0, f"Path start {start} too far from R1 {r1}"

    def test_path_ends_near_r2(self, bounds, cfg):
        r2 = (600, 400)
        path = build_path((200, 300), r2, (900, 300), bounds, cfg)
        end = path[-1]
        # Should end near R2 (within arena bounds clamping tolerance)
        dist = math.hypot(end[0] - r2[0], end[1] - r2[1])
        assert dist < 50.0, f"Path end {end} too far from R2 {r2}"

    def test_path_points_inside_arena(self, bounds, cfg):
        path = build_path((200, 300), (600, 400), (900, 300), bounds, cfg)
        for pt in path:
            assert bounds.inside((pt[0], pt[1])), f"Point {pt} outside arena"

    def test_r1_equals_r2(self, bounds, cfg):
        """Edge case: R1 and R2 at the same position."""
        path = build_path((400, 400), (400, 400), (900, 300), bounds, cfg)
        assert len(path) > 0
