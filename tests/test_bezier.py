"""Tests for the cubic Bézier function."""

import numpy as np
import pytest

from robot_control.planning.path_planner import _cubic_bezier


class TestCubicBezier:
    """Verify endpoint interpolation, midpoint, and shape."""

    def test_endpoints_match(self):
        """Curve must start at p0 and end at p3."""
        p0 = np.array([0.0, 0.0])
        p1 = np.array([10.0, 20.0])
        p2 = np.array([30.0, 20.0])
        p3 = np.array([40.0, 0.0])
        curve = _cubic_bezier(p0, p1, p2, p3, n=100)
        np.testing.assert_allclose(curve[0], p0, atol=1e-6)
        np.testing.assert_allclose(curve[-1], p3, atol=1e-6)

    def test_straight_line(self):
        """When all control points are collinear, the curve is a straight line."""
        p0 = np.array([0.0, 0.0])
        p1 = np.array([10.0, 0.0])
        p2 = np.array([20.0, 0.0])
        p3 = np.array([30.0, 0.0])
        curve = _cubic_bezier(p0, p1, p2, p3, n=50)
        np.testing.assert_allclose(curve[:, 1], 0.0, atol=1e-6)

    def test_midpoint_symmetric(self):
        """Symmetric control points should give a predictable midpoint."""
        p0 = np.array([0.0, 0.0])
        p1 = np.array([0.0, 10.0])
        p2 = np.array([10.0, 10.0])
        p3 = np.array([10.0, 0.0])
        curve = _cubic_bezier(p0, p1, p2, p3, n=101)
        mid = curve[50]
        # Midpoint at t=0.5: 0.125*p0 + 0.375*p1 + 0.375*p2 + 0.125*p3
        expected = 0.125 * p0 + 0.375 * p1 + 0.375 * p2 + 0.125 * p3
        np.testing.assert_allclose(mid, expected, atol=1e-3)

    def test_output_shape(self):
        """Output should have shape (n, 2)."""
        curve = _cubic_bezier([0, 0], [1, 1], [2, 2], [3, 3], n=25)
        assert curve.shape == (25, 2)
