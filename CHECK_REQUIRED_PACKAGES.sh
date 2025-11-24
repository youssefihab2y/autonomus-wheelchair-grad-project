#!/bin/bash
# Check and install missing required packages for wheelchair project

set -e

echo "=========================================="
echo "📦 CHECKING REQUIRED PACKAGES"
echo "=========================================="
echo ""

MISSING_PACKAGES=()

# Check required packages
check_package() {
    local pkg=$1
    local name=$2
    
    if dpkg -l | grep -q "^ii.*$pkg"; then
        echo "✅ $name: INSTALLED"
        return 0
    else
        echo "❌ $name: MISSING"
        MISSING_PACKAGES+=("$pkg")
        return 1
    fi
}

echo "=== Core ROS 2 Packages ==="
check_package "ros-humble-ros-gz-sim" "ros-gz-sim"
check_package "ros-humble-ros-gz-bridge" "ros-gz-bridge"
check_package "ros-humble-robot-state-publisher" "robot-state-publisher"
check_package "ros-humble-joint-state-publisher" "joint-state-publisher"
check_package "ros-humble-joint-state-publisher-gui" "joint-state-publisher-gui"

echo ""
echo "=== Controller Packages ==="
check_package "ros-humble-controller-manager" "controller-manager"
check_package "ros-humble-ros2-controllers" "ros2-controllers"
check_package "ros-humble-diff-drive-controller" "diff-drive-controller"
check_package "ros-humble-gz-ros2-control" "gz-ros2-control"

echo ""
echo "=== Utility Packages ==="
check_package "ros-humble-teleop-twist-keyboard" "teleop-twist-keyboard"
check_package "ros-humble-rviz2" "rviz2"

echo ""
echo "=== Build Tools ==="
if command -v colcon &> /dev/null; then
    echo "✅ colcon: INSTALLED ($(colcon --version 2>/dev/null | head -1))"
else
    echo "❌ colcon: MISSING"
    MISSING_PACKAGES+=("python3-colcon-common-extensions")
fi

if dpkg -l | grep -q "^ii.*ros-humble-xacro"; then
    echo "✅ xacro: INSTALLED"
else
    echo "❌ xacro: MISSING"
    MISSING_PACKAGES+=("ros-humble-xacro")
fi

echo ""
echo "=== Gazebo Packages ==="
if command -v gz &> /dev/null; then
    echo "✅ gz command: INSTALLED"
else
    echo "❌ gz command: MISSING"
    MISSING_PACKAGES+=("gz-sim7-cli")
fi

if dpkg -l | grep -q "^ii.*ignition-fortress"; then
    echo "✅ Gazebo Fortress: INSTALLED"
else
    echo "❌ Gazebo Fortress: MISSING"
    MISSING_PACKAGES+=("ignition-fortress")
fi

echo ""
echo "=========================================="
if [ ${#MISSING_PACKAGES[@]} -eq 0 ]; then
    echo "✅ ALL REQUIRED PACKAGES ARE INSTALLED!"
    echo ""
    echo "You're ready to build and run the project!"
else
    echo "⚠️  MISSING PACKAGES DETECTED"
    echo ""
    echo "Missing packages:"
    for pkg in "${MISSING_PACKAGES[@]}"; do
        echo "  - $pkg"
    done
    echo ""
    echo "To install missing packages, run:"
    echo "  sudo apt update"
    echo "  sudo apt install -y ${MISSING_PACKAGES[*]}"
    echo ""
    echo "Or run the install script:"
    echo "  ./INSTALL_MISSING_PACKAGES.sh"
fi
echo "=========================================="

