#!/usr/bin/env bash
# ──────────────────────────────────────────────────────────────────────────────
# install_kinect.sh — Install libfreenect2 and pylibfreenect2 on Ubuntu/Debian
#
# Prerequisites:
#   sudo apt install build-essential cmake pkg-config \
#       libusb-1.0-0-dev libturbojpeg0-dev libglfw3-dev \
#       libva-dev libjpeg-dev libopenni2-dev
#
# For OpenCL support (optional, for GPU depth processing):
#   sudo apt install beignet-dev  # Intel
#   # or install NVIDIA/AMD OpenCL SDK
#
# Usage:
#   chmod +x install_kinect.sh
#   ./install_kinect.sh
# ──────────────────────────────────────────────────────────────────────────────
set -euo pipefail

FREENECT2_DIR="${HOME}/libfreenect2"
INSTALL_PREFIX="/usr/local"

echo "============================================"
echo "  Kinect V2 Setup — libfreenect2 + Python"
echo "============================================"
echo ""

# ── Step 1: Install system dependencies ──────────────────────────────────────
echo "[1/5] Installing system dependencies..."
sudo apt-get update -qq
sudo apt-get install -y -qq \
    build-essential cmake pkg-config \
    libusb-1.0-0-dev libturbojpeg0-dev libglfw3-dev \
    libva-dev libjpeg-dev libopenni2-dev

# ── Step 2: Clone and build libfreenect2 ─────────────────────────────────────
echo ""
echo "[2/5] Building libfreenect2..."
if [ -d "${FREENECT2_DIR}" ]; then
    echo "  libfreenect2 directory already exists at ${FREENECT2_DIR}"
    echo "  Pulling latest changes..."
    cd "${FREENECT2_DIR}" && git pull
else
    git clone https://github.com/OpenKinect/libfreenect2.git "${FREENECT2_DIR}"
fi

cd "${FREENECT2_DIR}"
mkdir -p build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX="${INSTALL_PREFIX}"
make -j"$(nproc)"
sudo make install

# ── Step 3: Set up udev rules (non-root Kinect access) ──────────────────────
echo ""
echo "[3/5] Installing udev rules for non-root Kinect access..."
sudo cp "${FREENECT2_DIR}/platform/linux/udev/90-kinect2.rules" /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
echo "  NOTE: You may need to unplug and replug the Kinect for rules to take effect."

# ── Step 4: Set environment variables ────────────────────────────────────────
echo ""
echo "[4/5] Setting environment variables..."
PROFILE_LINE="export LIBFREENECT2_INSTALL_PREFIX=${INSTALL_PREFIX}"
LD_LINE="export LD_LIBRARY_PATH=\${LD_LIBRARY_PATH:+\$LD_LIBRARY_PATH:}${INSTALL_PREFIX}/lib"

if ! grep -q "LIBFREENECT2_INSTALL_PREFIX" ~/.bashrc 2>/dev/null; then
    echo "" >> ~/.bashrc
    echo "# Kinect V2 (libfreenect2)" >> ~/.bashrc
    echo "${PROFILE_LINE}" >> ~/.bashrc
    echo "${LD_LINE}" >> ~/.bashrc
    echo "  Added libfreenect2 paths to ~/.bashrc"
else
    echo "  libfreenect2 paths already in ~/.bashrc"
fi

# Export for current session.
export LIBFREENECT2_INSTALL_PREFIX="${INSTALL_PREFIX}"
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:+$LD_LIBRARY_PATH:}${INSTALL_PREFIX}/lib"

# ── Step 5: Install Python package ──────────────────────────────────────────
echo ""
echo "[5/5] Installing pylibfreenect2 Python package..."

# Use the project venv if it exists.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ -f "${SCRIPT_DIR}/.venv/bin/pip" ]; then
    echo "  Using project venv at ${SCRIPT_DIR}/.venv"
    "${SCRIPT_DIR}/.venv/bin/pip" install pylibfreenect2
else
    pip install pylibfreenect2
fi

echo ""
echo "============================================"
echo "  Kinect V2 setup complete!"
echo ""
echo "  Test with:"
echo "    source ~/.bashrc"
echo "    cd ${SCRIPT_DIR}"
echo "    source .venv/bin/activate"
echo "    python -m robot_control.vision.kinect_viewer"
echo "============================================"
