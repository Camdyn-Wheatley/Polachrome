"""Tests for KalmanTracker."""

import pytest

from robot_control.filters import KalmanTracker


class TestKalmanTracker:
    def test_init_state(self):
        kf = KalmanTracker(max_coast_frames=10)
        assert kf.position_float() is None

    def test_first_update(self):
        kf = KalmanTracker(max_coast_frames=10)
        pos = kf.update(100.0, 200.0, 0.033)
        assert pos is not None
        # Should be close to the initial measurement
        assert abs(pos[0] - 100) < 10
        assert abs(pos[1] - 200) < 10

    def test_coast_returns_prediction(self):
        kf = KalmanTracker(max_coast_frames=10)
        kf.update(100.0, 200.0, 0.033)
        pos = kf.predict(0.033)
        assert pos is not None

    def test_coast_limit(self):
        kf = KalmanTracker(max_coast_frames=3)
        kf.update(100.0, 200.0, 0.033)
        assert kf.predict(0.033) is not None  # coast 1
        assert kf.predict(0.033) is not None  # coast 2
        assert kf.predict(0.033) is not None  # coast 3
        assert kf.predict(0.033) is None      # coast 4 → exceeded

    def test_velocity_after_updates(self):
        kf = KalmanTracker(max_coast_frames=10)
        # Move right at ~100 px/frame
        for i in range(10):
            kf.update(float(i * 100), 200.0, 0.033)
        vx, vy = kf.velocity()
        assert vx > 0, "Should have positive x velocity"

    def test_coasting_property(self):
        kf = KalmanTracker(max_coast_frames=10)
        kf.update(100.0, 200.0, 0.033)
        assert not kf.coasting
        kf.predict(0.033)
        assert kf.coasting

    def test_update_resets_coast(self):
        kf = KalmanTracker(max_coast_frames=10)
        kf.update(100.0, 200.0, 0.033)
        kf.predict(0.033)
        assert kf.coast == 1
        kf.update(110.0, 200.0, 0.033)
        assert kf.coast == 0

    def test_predict_before_init_returns_none(self):
        kf = KalmanTracker(max_coast_frames=10)
        assert kf.predict(0.033) is None
