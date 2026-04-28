#!/usr/bin/env bash
# ──────────────────────────────────────────────────────────────────────────────
# install.sh — Set up the Python environment for the Combat Robot Controller.
#
# Creates a virtual environment, installs Python dependencies, and checks
# system prerequisites. Run this BEFORE install_kinect.sh.
#
# Usage:
#   chmod +x install.sh
#   ./install.sh
# ──────────────────────────────────────────────────────────────────────────────
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV_DIR="${SCRIPT_DIR}/.venv"

echo "============================================"
echo "  Combat Robot Controller — Python Setup"
echo "============================================"
echo ""

# ── Step 1: Check for python3 ────────────────────────────────────────────────
echo "[1/5] Checking for Python 3..."
if ! command -v python3 &>/dev/null; then
    echo "  ERROR: python3 not found."
    echo "  Install it with: sudo apt install python3"
    exit 1
fi
PYTHON_VERSION=$(python3 --version 2>&1)
echo "  Found: ${PYTHON_VERSION}"

# ── Step 2: Check for python3-venv ───────────────────────────────────────────
echo ""
echo "[2/5] Checking for python3-venv..."
if ! python3 -m venv --help &>/dev/null; then
    echo "  python3-venv not installed. Installing..."
    sudo apt-get update -qq
    sudo apt-get install -y -qq python3-venv
fi
echo "  python3-venv is available."

# ── Step 3: Create virtual environment ───────────────────────────────────────
echo ""
echo "[3/5] Setting up virtual environment..."
if [ -d "${VENV_DIR}" ]; then
    echo "  Virtual environment already exists at ${VENV_DIR}"
else
    echo "  Creating virtual environment at ${VENV_DIR}..."
    python3 -m venv "${VENV_DIR}"
    echo "  Created."
fi

# ── Step 4: Install Python dependencies ─────────────────────────────────────
echo ""
echo "[4/5] Installing Python dependencies..."
"${VENV_DIR}/bin/pip" install --upgrade pip 2>&1 | tail -1
"${VENV_DIR}/bin/pip" install -r "${SCRIPT_DIR}/requirements.txt" 2>&1 | tail -5

# Guard against the opencv-python / opencv-python-headless conflict.
# Having both installed causes segfaults and broken cv2 stubs.
if "${VENV_DIR}/bin/pip" show opencv-python-headless &>/dev/null; then
    echo "  Removing conflicting opencv-python-headless..."
    "${VENV_DIR}/bin/pip" uninstall -y opencv-python-headless 2>&1 | tail -1
    # Force-reinstall the GUI variant to repair any broken cv2 stubs.
    "${VENV_DIR}/bin/pip" install --force-reinstall "opencv-python>=4.5" 2>&1 | tail -1
fi
echo "  Python dependencies installed."

# ── Step 5: Serial access (dialout group) ────────────────────────────────────
echo ""
echo "[5/5] Checking serial port access..."
if groups | grep -q dialout; then
    echo "  User is already in the dialout group."
else
    echo "  Adding user to dialout group for Arduino serial access..."
    sudo usermod -a -G dialout "$USER"
    echo "  Added. You must LOG OUT and log back in for this to take effect."
fi

echo ""
echo "============================================"
echo "  Python setup complete!"
echo ""
echo "  Next step — install Kinect V2 support:"
echo "    ./install_kinect.sh"
echo ""
echo "  Or if you don't need the Kinect, you can"
echo "  run tools directly:"
echo "    .venv/bin/python3 -m robot_control.vision.kinect_viewer"
echo "============================================"
