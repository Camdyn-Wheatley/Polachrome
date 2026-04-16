"""
__main__.py — Entry point for the combat robot controller.

Run with:  python -m robot_control [--config path/to/config.yaml]

NOTE: Vision and path planning are being rewritten for Kinect V2.
      This entry point currently only initialises the control subsystem
      (Arduino serial, config) and launches the Kinect viewer for testing.
"""

from __future__ import annotations

import argparse
import logging
import signal

from robot_control.config import load_config
from robot_control.state import shutdown_event

logger = logging.getLogger(__name__)


def _signal_handler(sig: int, _frame: object) -> None:
    logger.warning("Signal %s — shutting down...", signal.Signals(sig).name)
    shutdown_event.set()


def main() -> None:
    parser = argparse.ArgumentParser(description="Combat Robot Controller")
    parser.add_argument(
        "--config", type=str, default=None, help="Path to config.yaml"
    )
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(name)s] %(levelname)s: %(message)s",
        datefmt="%H:%M:%S",
    )

    cfg = load_config(args.config)

    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    logger.info("Combat Robot Controller — Vision rewrite in progress")
    logger.info("Config loaded: pipeline=%s, aruco_dict=%s", cfg.kinect_pipeline, cfg.aruco_dict)
    logger.info(
        "Robot tags: top=%d, bottom=%d, detect_on_ir=%s",
        cfg.robot_tag_top, cfg.robot_tag_bottom, cfg.detect_on_ir,
    )
    logger.info("")
    logger.info("To view Kinect streams, run:")
    logger.info("  python -m robot_control.vision.kinect_viewer")
    logger.info("")
    logger.info("Full autonomous control loop will be restored in a future phase.")


if __name__ == "__main__":
    main()
