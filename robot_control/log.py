"""
log.py — CSV position logger for the combat robot controller.
"""

from __future__ import annotations

import csv
import logging
import os
from datetime import datetime
from typing import IO, Optional, Tuple

logger = logging.getLogger(__name__)


def init_log(path: str) -> Tuple[IO, csv.writer]:
    """Open (or create) the CSV position log and return ``(file, writer)``."""
    new = not os.path.exists(path)
    f = open(path, "a", newline="")
    w = csv.writer(f)
    if new:
        w.writerow(
            [
                "timestamp",
                "r1_x",
                "r1_y",
                "r1_src",
                "r2_x",
                "r2_y",
                "r2_src",
                "ch1",
                "ch2",
                "ch3",
                "auto",
            ]
        )
    logger.info("Position log: %s", path)
    return f, w


def log_row(
    writer: csv.writer,
    r1: Optional[Tuple[int, int]],
    r1_src: str,
    r2: Optional[Tuple[int, int]],
    r2_src: str,
    ch1: int,
    ch2: int,
    ch3: int,
    auto: bool,
) -> None:
    """Write one frame's data to the CSV log."""
    ts = datetime.now().isoformat(timespec="milliseconds")
    writer.writerow(
        [
            ts,
            r1[0] if r1 else "",
            r1[1] if r1 else "",
            r1_src,
            r2[0] if r2 else "",
            r2[1] if r2 else "",
            r2_src,
            ch1,
            ch2,
            ch3,
            int(auto),
        ]
    )
