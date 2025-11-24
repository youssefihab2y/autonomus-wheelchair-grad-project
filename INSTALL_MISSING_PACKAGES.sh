#!/bin/bash
# Install all missing required packages for wheelchair project

set -e

echo "=========================================="
echo "📦 INSTALLING MISSING PACKAGES"
echo "=========================================="
echo ""

# Update package index
echo "[1/2] Updating package index..."
sudo apt update

# Install all required packages
echo ""
echo "[2/2] Installing required packages..."
sudo apt install -y \
    ros-humble-ros-gz-sim \
    ros-humble-ros-gz-bridge \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-teleop-twist-keyboard \
    ros-humble-rviz2 \
    ros-humble-controller-manager \
    ros-humble-ros2-controllers \
    ros-humble-diff-drive-controller \
    ros-humble-gz-ros2-control \
    ros-humble-xacro \
    python3-colcon-common-extensions \
    gz-sim7-cli

echo ""
echo "=========================================="
echo "✅ INSTALLATION COMPLETE!"
echo "=========================================="
echo ""
echo "Verifying installation..."
./CHECK_REQUIRED_PACKAGES.sh

