"""Combat Robot Controller — Autonomous vision-guided combat robot system."""

import os as _os

# Force a readable Qt theme for OpenCV trackbar windows.
# Without this, light desktop themes render white text on white backgrounds.
_os.environ.setdefault("QT_STYLE_OVERRIDE", "Fusion")

__version__ = "1.0.0"
