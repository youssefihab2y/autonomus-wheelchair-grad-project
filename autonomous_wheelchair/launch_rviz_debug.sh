#!/bin/bash
# Launch RViz with debug displays for troubleshooting

cd /home/nadafouad/Downloads/Autonomous-Wheelchair-master-branch/autonomous_wheelchair
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "=========================================="
echo "🔍 LAUNCHING RViz FOR DEBUGGING"
echo "=========================================="
echo ""
echo "RViz will show:"
echo "  - TF tree (check for missing frames)"
echo "  - Robot model"
echo "  - Joint states"
echo "  - Odometry"
echo "  - LiDAR scan"
echo ""
echo "Look for errors in:"
echo "  - TF tree (red/yellow frames = errors)"
echo "  - Console output (warnings/errors)"
echo ""

ros2 launch wheelchair_description view_robot_rviz2.launch.py
