#!/bin/bash
# Reinstall ROS 2 Humble and Gazebo Fortress following official documentation
# Reference: 
# - ROS 2 Humble: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html
# - Gazebo Fortress: https://gazebosim.org/docs/fortress/install

set -e

echo "=========================================="
echo "📦 REINSTALLING ROS 2 HUMBLE + GAZEBO FORTRESS"
echo "=========================================="
echo ""

# Check Ubuntu version
UBUNTU_VERSION=$(lsb_release -rs)
if [ "$UBUNTU_VERSION" != "22.04" ]; then
    echo "⚠️  WARNING: This script is designed for Ubuntu 22.04"
    echo "   Detected version: $UBUNTU_VERSION"
    echo "   Continue anyway? (y/n)"
    read -r response
    if [ "$response" != "y" ]; then
        exit 1
    fi
fi

# ==================== STEP 1: SET UP LOCALE ====================
echo "[1/8] Setting up locale..."
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
echo "✅ Locale configured"

# ==================== STEP 2: INSTALL PREREQUISITES ====================
echo ""
echo "[2/8] Installing prerequisites..."
sudo apt update && sudo apt install -y \
    curl \
    gnupg \
    lsb-release \
    wget \
    software-properties-common
echo "✅ Prerequisites installed"

# ==================== STEP 3: ADD ROS 2 REPOSITORY ====================
echo ""
echo "[3/8] Adding ROS 2 Humble repository..."
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
echo "✅ ROS 2 repository added"

# ==================== STEP 4: ADD GAZEBO REPOSITORY ====================
echo ""
echo "[4/8] Adding Gazebo Fortress repository..."
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
echo "✅ Gazebo repository added"

# ==================== STEP 5: UPDATE PACKAGE INDEX ====================
echo ""
echo "[5/8] Updating package index..."
sudo apt update
echo "✅ Package index updated"

# ==================== STEP 6: INSTALL ROS 2 HUMBLE ====================
echo ""
echo "[6/8] Installing ROS 2 Humble Desktop..."
sudo apt install -y \
    ros-humble-desktop \
    python3-argcomplete \
    python3-colcon-common-extensions
echo "✅ ROS 2 Humble Desktop installed"

# ==================== STEP 7: INSTALL GAZEBO FORTRESS ====================
echo ""
echo "[7/8] Installing Gazebo Fortress..."
sudo apt install -y \
    ignition-fortress \
    gz-sim7-cli \
    ignition-tools
echo "✅ Gazebo Fortress installed (including gz command)"

# ==================== STEP 8: INSTALL PROJECT-SPECIFIC PACKAGES ====================
echo ""
echo "[8/8] Installing project-specific ROS 2 packages..."
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
    ros-humble-gz-ros2-control
echo "✅ Project-specific packages installed"

# ==================== STEP 9: SET UP ENVIRONMENT ====================
echo ""
echo "[9/9] Setting up environment..."
# Source ROS 2
source /opt/ros/humble/setup.bash

# Add to bashrc if not already present
if ! grep -q "source /opt/ros/humble/setup.bash" ~/.bashrc; then
    echo "" >> ~/.bashrc
    echo "# ROS 2 Humble" >> ~/.bashrc
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    echo "✅ Added ROS 2 sourcing to ~/.bashrc"
else
    echo "✅ ROS 2 sourcing already in ~/.bashrc"
fi

# ==================== VERIFICATION ====================
echo ""
echo "=========================================="
echo "✅ INSTALLATION COMPLETE!"
echo "=========================================="
echo ""
echo "Installed versions:"
echo "  ROS 2: $(ros2 --version 2>/dev/null | head -1 || echo 'Not found in PATH')"
echo "  Gazebo: $(gz sim --version 2>/dev/null | head -1 || echo 'Not found in PATH')"
echo ""
echo "Next steps:"
echo "1. Close and reopen your terminal (or run: source ~/.bashrc)"
echo "2. Verify installation:"
echo "   - ROS 2: ros2 --help"
echo "   - Gazebo: gz sim --help"
echo "3. Build your workspace:"
echo "   cd autonomous_wheelchair"
echo "   colcon build --symlink-install"
echo "   source install/setup.bash"
echo ""
echo "To test ROS 2:"
echo "  Terminal 1: ros2 run demo_nodes_cpp talker"
echo "  Terminal 2: ros2 run demo_nodes_py listener"
echo ""
echo "To test Gazebo:"
echo "  gz sim shapes.sdf"
echo ""



