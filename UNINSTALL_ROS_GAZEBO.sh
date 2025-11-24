#!/bin/bash
# Uninstall ROS 2 and Gazebo following official documentation
# This script removes all ROS 2 and Gazebo packages from the system

set -e

echo "=========================================="
echo "🗑️  UNINSTALLING ROS 2 AND GAZEBO"
echo "=========================================="
echo ""
echo "⚠️  WARNING: This will remove all ROS 2 and Gazebo packages!"
echo "Press Ctrl+C to cancel, or Enter to continue..."
read

echo ""
echo "[1/5] Removing ROS 2 packages..."
sudo apt-get purge -y ros-* || true
sudo apt-get autoremove -y || true
sudo apt-get autoclean || true

echo ""
echo "[2/5] Removing ROS 2 directories..."
sudo rm -rf /opt/ros || true
sudo rm -rf ~/.ros || true

echo ""
echo "[3/5] Removing Gazebo packages..."
sudo apt-get purge -y ignition-* || true
sudo apt-get purge -y gazebo* || true
sudo apt-get autoremove -y || true

echo ""
echo "[4/5] Removing Gazebo directories..."
sudo rm -rf /usr/share/ignition || true
sudo rm -rf /usr/share/gazebo* || true
sudo rm -rf ~/.gazebo || true
sudo rm -rf ~/.ignition || true

echo ""
echo "[5/5] Removing repository sources..."
sudo rm -f /etc/apt/sources.list.d/ros2*.list || true
sudo rm -f /etc/apt/sources.list.d/gazebo*.list || true
sudo rm -f /usr/share/keyrings/ros-archive-keyring.gpg || true
sudo rm -f /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg || true

echo ""
echo "[6/6] Cleaning up bashrc..."
# Remove ROS sourcing from bashrc if present
sed -i '/source.*ros.*setup.bash/d' ~/.bashrc || true
sed -i '/source.*gazebo.*setup.bash/d' ~/.bashrc || true

echo ""
echo "✅ Uninstallation complete!"
echo ""
echo "To complete the cleanup, please:"
echo "1. Close and reopen your terminal"
echo "2. Run: sudo apt update"
echo "3. Run the REINSTALL_ROS_GAZEBO.sh script to reinstall"



