#!/bin/bash
# Start keyboard control - MUST run in your own terminal (interactive)

cd /home/nadafouad/Downloads/Autonomous-Wheelchair-master-branch/autonomous_wheelchair
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "=========================================="
echo "⌨️  KEYBOARD CONTROL FOR WHEELCHAIR"
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
echo "⚠️  Make sure the simulation is running first!"
echo ""

# Check if cmd_vel topic exists
if ros2 topic list 2>&1 | grep -q "/cmd_vel"; then
    echo "✅ /cmd_vel topic found - Ready to control!"
    echo ""
    echo "Starting keyboard control..."
    echo ""
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
else
    echo "❌ ERROR: /cmd_vel topic not found!"
    echo ""
    echo "Please make sure the simulation is running:"
    echo "  ros2 launch wheelchair_gazebo warehouse_with_robot.launch.py"
    exit 1
fi



