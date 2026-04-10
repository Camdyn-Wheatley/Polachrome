#!/bin/bash

# Robot Tracking Control Installer
echo "Installing Python dependencies..."
python3 -m pip install --upgrade pip
python3 -m pip install -r requirements.txt

echo "Make sure your user is in the dialout group for serial access:"
echo "  sudo usermod -a -G dialout \$USER"
echo ""
echo "Installation complete. You may need to log out and log back in to apply the dialout group."
