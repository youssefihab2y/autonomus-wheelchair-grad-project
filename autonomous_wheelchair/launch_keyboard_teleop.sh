#!/bin/bash
# Launch keyboard teleop in a separate terminal
# This script can be run after the main simulation is launched

cd /home/nadafouad/Downloads/Autonomous-Wheelchair-master-branch/autonomous_wheelchair
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "=========================================="
echo "⌨️  KEYBOARD TELEOP CONTROL"
echo "=========================================="
echo ""
echo "Controls:"
echo "  i/k: Move forward/backward"
echo "  j/l: Turn left/right"
echo "  ,/.: Increase/decrease linear speed"
echo "  u/o: Increase/decrease angular speed"
echo "  Space: Emergency stop"
echo "  q: Quit"
echo ""
echo "Make sure the simulation is running first!"
echo ""

# Check if teleop_twist_keyboard is installed
if ! command -v ros2 &> /dev/null; then
    echo "❌ ERROR: ROS 2 not found. Please source ROS 2 setup.bash first"
    exit 1
fi

# Check if cmd_vel topic exists
echo "Checking for /cmd_vel topic..."
timeout 3 ros2 topic list | grep -q cmd_vel
if [ $? -eq 0 ]; then
    echo "✅ /cmd_vel topic found"
    echo ""
    echo "Starting keyboard teleop..."
    echo ""
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
else
    echo "❌ ERROR: /cmd_vel topic not found!"
    echo "   Make sure the simulation is running first:"
    echo "   ros2 launch wheelchair_gazebo warehouse_with_robot.launch.py"
    exit 1
fi






