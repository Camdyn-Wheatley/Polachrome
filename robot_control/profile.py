"""
profile.py — Profile persistence for the combat robot controller.

Currently a stub — R2 colour profiles are no longer used (opponent is
detected via depth sensor). This module will be repurposed in the future
for ground-plane calibration persistence.
"""

from __future__ import annotations

import logging

logger = logging.getLogger(__name__)


def save_profile(path: str, data: dict) -> None:
    """Save a calibration profile to JSON."""
    import json
    with open(path, "w") as f:
        json.dump(data, f, indent=2)
    logger.info("Saved profile to %s", path)


def load_profile(path: str) -> dict:
    """Load a calibration profile from JSON. Returns empty dict if not found."""
    import json
    import os
    if not os.path.exists(path):
        return {}
    with open(path) as f:
        data = json.load(f)
    logger.info("Loaded profile from %s", path)
    return data
