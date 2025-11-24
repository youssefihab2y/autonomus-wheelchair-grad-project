#!/bin/bash
# Test robot movement with keyboard control

cd /home/nadafouad/Downloads/Autonomous-Wheelchair-master-branch/autonomous_wheelchair
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "=========================================="
echo "🎮 KEYBOARD CONTROL TEST"
echo "=========================================="
echo ""
echo "Make sure Gazebo is running first!"
echo ""
echo "Controls:"
echo "  i - Move forward"
echo "  k - Move backward"
echo "  j - Turn left"
echo "  l - Turn right"
echo "  , - Increase linear speed"
echo "  . - Decrease linear speed"
echo "  u - Increase angular speed"
echo "  o - Decrease angular speed"
echo "  Space - Emergency stop"
echo "  q - Quit"
echo ""
echo "Starting keyboard teleop..."
echo ""

ros2 run teleop_twist_keyboard teleop_twist_keyboard

