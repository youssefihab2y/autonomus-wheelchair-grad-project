#!/bin/bash
# Quick fix to install missing gz command for Gazebo Fortress

echo "Installing gz-sim7-cli (provides 'gz sim' command)..."
sudo apt update
sudo apt install -y gz-sim7-cli ignition-tools

echo ""
echo "Verifying installation..."
if command -v gz &> /dev/null; then
    echo "✅ gz command found!"
    gz --version
else
    echo "⚠️  gz command still not found"
    echo "Trying alternative: ign command..."
    if command -v ign &> /dev/null; then
        echo "✅ ign command found (alternative)"
        ign --version
    fi
fi
