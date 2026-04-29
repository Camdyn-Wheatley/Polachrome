"""
config.py — Configuration loader for the combat robot controller.

Loads tunable parameters from a YAML file into a frozen dataclass with
startup validation. Falls back to sensible defaults if the config file
is missing.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Tuple

logger = logging.getLogger(__name__)

_DEFAULT_CONFIG_PATH = Path(__file__).resolve().parent.parent / "config.yaml"


@dataclass(frozen=True)
class Config:
    """Immutable configuration for the robot controller."""

    # ── Kinect V2 ────────────────────────────────────────────────────────
    kinect_pipeline: str = "opengl"

    # ── Depth Segmentation ───────────────────────────────────────────────
    arena_max_depth_mm: int = 3500
    obstacle_min_height_mm: float = 12.0
    obstacle_min_area_px: int = 25
    obstacle_max_area_px: int = 5000

    # ── ArUco detection ──────────────────────────────────────────────────
    aruco_dict: str = "DICT_APRILTAG_25H9"  # AprilTag 25h9
    robot_tag_top: int = 0
    robot_tag_bottom: int = 1
    arena_floor_tag: int = 3
    detect_on_ir: bool = False      # False = color camera, True = IR
    flip_horizontal: bool = True    # Mirror all frames (Kinect mounted inverted)

    # ── Kalman filter ────────────────────────────────────────────────────
    max_coast_frames: int = 30

    # ── Logging ──────────────────────────────────────────────────────────
    position_log: str = "positions.csv"

    # ── Arena bounds (pixel coordinates — depth frame space) ─────────────
    arena_tl: Tuple[int, int] = (50, 50)
    arena_br: Tuple[int, int] = (462, 374)
    robot_offset: int = 20

    # ── Path planner (placeholder — will be rewritten) ───────────────────
    path_replan_dist: int = 20
    lookahead_dist: int = 80
    approach_standoff: int = 80

    # ── Motion control (PWM microseconds) ────────────────────────────────
    pwm_min: int = 1000
    pwm_max: int = 2000
    pwm_neutral: int = 1486
    max_drive_speed: int = 400
    max_turn_speed: int = 400
    weapon_on_value: int = 2000
    weapon_off_value: int = 1000

    # ── Arduino serial ───────────────────────────────────────────────────
    arduino_port: Optional[str] = None
    arduino_baud: int = 9600

    # ── Channel reversal ─────────────────────────────────────────────────
    channel_reverse_steer: bool = False
    channel_reverse_drive: bool = True
    channel_reverse_weapon: bool = False

    @property
    def robot_tag_ids(self) -> set:
        """Return the set of ArUco IDs belonging to our robot."""
        return {self.robot_tag_top, self.robot_tag_bottom}


def _validate(cfg: Config) -> None:
    """Validate configuration values at startup."""
    valid_pipelines = {"opengl", "opencl", "cpu"}
    assert cfg.kinect_pipeline in valid_pipelines, (
        f"kinect_pipeline must be one of {valid_pipelines}, got {cfg.kinect_pipeline!r}"
    )
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
    assert cfg.lookahead_dist > 0, (
        f"lookahead_dist must be > 0, got {cfg.lookahead_dist}"
    )
    assert cfg.robot_tag_top != cfg.robot_tag_bottom, (
        f"robot_tag_top and robot_tag_bottom must be different IDs, "
        f"got {cfg.robot_tag_top} and {cfg.robot_tag_bottom}"
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

    # Convert lists to tuples where the dataclass expects tuples
    for key in ("arena_tl", "arena_br"):
        if key in raw and isinstance(raw[key], list):
            raw[key] = tuple(raw[key])

    # Filter out keys not in Config to avoid TypeError
    valid_keys = {f.name for f in Config.__dataclass_fields__.values()}
    filtered = {k: v for k, v in raw.items() if k in valid_keys}

    cfg = Config(**filtered)
    _validate(cfg)
    logger.info("Loaded config from %s", config_path)
    return cfg
