#!/bin/bash
# Launch both Gazebo and RViz together

cd /home/nadafouad/Downloads/Autonomous-Wheelchair-master-branch/autonomous_wheelchair

# Source ROS 2 and workspace
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "=========================================="
echo "🚀 Launching Gazebo + RViz"
echo "=========================================="
echo ""

# Launch Gazebo in background
echo "[1/2] Launching Gazebo..."
ros2 launch wheelchair_gazebo warehouse_with_robot.launch.py &
GAZEBO_PID=$!

# Wait for Gazebo to initialize
echo "Waiting 10 seconds for Gazebo to initialize..."
sleep 10

# Launch RViz
echo "[2/2] Launching RViz..."
ros2 launch wheelchair_description view_robot_rviz2.launch.py

# Cleanup on exit
trap "kill $GAZEBO_PID 2>/dev/null" EXIT

