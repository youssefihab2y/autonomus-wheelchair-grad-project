#!/bin/bash
# Launch RViz for wheelchair visualization

cd /home/nadafouad/Downloads/Autonomous-Wheelchair-master-branch/autonomous_wheelchair

# Source ROS 2 and workspace
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "=========================================="
echo "🚀 Launching RViz for Wheelchair"
echo "=========================================="
echo ""

# Launch RViz
ros2 launch wheelchair_description view_robot_rviz2.launch.py

