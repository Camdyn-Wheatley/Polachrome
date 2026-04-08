"""
config.py — Configuration loader for the combat robot controller.

Loads tunable parameters from a YAML file into a frozen dataclass with
startup validation. Falls back to sensible defaults if the config file
is missing.
"""

from __future__ import annotations

import logging
import os
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np

logger = logging.getLogger(__name__)

_DEFAULT_CONFIG_PATH = Path(__file__).resolve().parent.parent / "config.yaml"


@dataclass(frozen=True)
class Config:
    """Immutable configuration for the robot controller."""

    # Camera
    camera_device: str = "/dev/video1"
    frame_width: int = 1280
    frame_height: int = 720
    exposure: Optional[int] = 150

    # AprilTag detection
    tag_family: str = "tag25h9"
    tag_size: float = 0.03302
    robot1_tag_ids: Tuple[int, ...] = (0, 1)
    quad_decimate: float = 2.0
    detector_threads: int = 2

    # ROI tracking
    roi_padding: int = 180
    roi_reacquire_frames: int = 6

    # Color tracking (Robot 2)
    profile_file: str = "color_profile.json"
    min_contour_area: int = 500
    default_tolerance_h: int = 15
    default_tolerance_s: int = 60
    default_tolerance_v: int = 60

    # Kalman filter
    max_coast_frames: int = 30

    # Logging
    position_log: str = "positions.csv"

    # Arena bounds (pixel coordinates)
    arena_tl: Tuple[int, int] = (50, 50)
    arena_br: Tuple[int, int] = (1230, 670)
    robot_offset: int = 40

    # Path planner
    path_replan_dist: int = 20
    lookahead_dist: int = 80
    approach_standoff: int = 80

    # Motion control (PWM microseconds)
    pwm_min: int = 1000
    pwm_max: int = 2000
    pwm_neutral: int = 1500
    max_drive_speed: int = 400
    max_turn_speed: int = 400
    weapon_on_value: int = 2000
    weapon_off_value: int = 1000

    # Threat evasion
    threat_cone_deg: float = 40.0
    evasion_offset: int = 120

    # Target tags
    target_tag_a: int = 3
    target_tag_b: int = 4

    # Arduino serial
    arduino_port: Optional[str] = None
    arduino_baud: int = 115200

    # Robot 1 color fallback
    color_resample_interval: int = 60
    color_sample_radius: int = 12

    # Camera calibration
    camera_matrix_file: Optional[str] = None

    @property
    def default_tolerance(self) -> Dict[str, int]:
        """Return the default HSV tolerance dict."""
        return {
            "h": self.default_tolerance_h,
            "s": self.default_tolerance_s,
            "v": self.default_tolerance_v,
        }

    @property
    def robot1_tag_id_set(self) -> set:
        """Return robot1_tag_ids as a set for fast membership tests."""
        return set(self.robot1_tag_ids)

    def camera_matrix(self) -> np.ndarray:
        """Load or compute the camera intrinsic matrix."""
        if self.camera_matrix_file and os.path.exists(self.camera_matrix_file):
            try:
                data = np.load(self.camera_matrix_file)
                logger.info("Loaded camera matrix from %s", self.camera_matrix_file)
                return data["camera_matrix"]
            except Exception as exc:
                logger.warning(
                    "Failed to load %s, falling back to estimate: %s",
                    self.camera_matrix_file,
                    exc,
                )
        return np.array(
            [
                [self.frame_width, 0, self.frame_width / 2.0],
                [0, self.frame_width, self.frame_height / 2.0],
                [0, 0, 1.0],
            ],
            dtype=np.float64,
        )

    def dist_coeffs(self) -> np.ndarray:
        """Return distortion coefficients (zero if no calibration loaded)."""
        if self.camera_matrix_file and os.path.exists(self.camera_matrix_file):
            try:
                data = np.load(self.camera_matrix_file)
                if "dist_coeffs" in data:
                    return data["dist_coeffs"]
            except Exception:
                pass
        return np.zeros((4, 1), dtype=np.float64)

    def tag_points_3d(self) -> np.ndarray:
        """Return the 3D corner coordinates for a tag of size ``tag_size``."""
        h = self.tag_size / 2.0
        return np.array(
            [[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]],
            dtype=np.float64,
        )


def _validate(cfg: Config) -> None:
    """Validate configuration values at startup."""
    assert cfg.frame_width > 0, f"frame_width must be > 0, got {cfg.frame_width}"
    assert cfg.frame_height > 0, f"frame_height must be > 0, got {cfg.frame_height}"
    assert cfg.max_coast_frames >= 1, (
        f"max_coast_frames must be >= 1, got {cfg.max_coast_frames}"
    )
    assert cfg.pwm_min < cfg.pwm_max, (
        f"pwm_min ({cfg.pwm_min}) must be < pwm_max ({cfg.pwm_max})"
    )
    assert cfg.pwm_min <= cfg.pwm_neutral <= cfg.pwm_max, (
        f"pwm_neutral ({cfg.pwm_neutral}) must be between "
        f"pwm_min ({cfg.pwm_min}) and pwm_max ({cfg.pwm_max})"
    )
    assert cfg.tag_size > 0, f"tag_size must be > 0, got {cfg.tag_size}"
    assert cfg.roi_padding >= 0, f"roi_padding must be >= 0, got {cfg.roi_padding}"
    assert cfg.lookahead_dist > 0, (
        f"lookahead_dist must be > 0, got {cfg.lookahead_dist}"
    )
    logger.info("Configuration validated OK")


def load_config(path: Optional[str] = None) -> Config:
    """
    Load configuration from a YAML file and return a validated ``Config``.

    Falls back to defaults if the file doesn't exist.
    """
    config_path = Path(path) if path else _DEFAULT_CONFIG_PATH

    if not config_path.exists():
        logger.warning("Config file %s not found — using defaults", config_path)
        cfg = Config()
        _validate(cfg)
        return cfg

    try:
        import yaml
    except ImportError:
        logger.warning("PyYAML not installed — using default config")
        cfg = Config()
        _validate(cfg)
        return cfg

    with open(config_path, "r") as f:
        raw = yaml.safe_load(f) or {}

    # Flatten nested tolerance into separate fields
    tol = raw.pop("default_tolerance", None)
    if isinstance(tol, dict):
        raw.setdefault("default_tolerance_h", tol.get("h", 15))
        raw.setdefault("default_tolerance_s", tol.get("s", 60))
        raw.setdefault("default_tolerance_v", tol.get("v", 60))

    # Convert lists to tuples where the dataclass expects tuples
    for key in ("robot1_tag_ids", "arena_tl", "arena_br"):
        if key in raw and isinstance(raw[key], list):
            raw[key] = tuple(raw[key])

    # Filter out keys not in Config to avoid TypeError
    valid_keys = {f.name for f in Config.__dataclass_fields__.values()}
    filtered = {k: v for k, v in raw.items() if k in valid_keys}

    cfg = Config(**filtered)
    _validate(cfg)
    logger.info("Loaded config from %s", config_path)
    return cfg
