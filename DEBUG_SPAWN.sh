#!/bin/bash
# Debug script to check why robot isn't spawning

cd /home/nadafouad/Downloads/Autonomous-Wheelchair-master-branch/autonomous_wheelchair
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "=== Debugging Robot Spawn ==="
echo ""

echo "[1] Checking robot_state_publisher..."
if ros2 node list | grep -q robot_state_publisher; then
    echo "✅ robot_state_publisher is running"
else
    echo "❌ robot_state_publisher is NOT running"
fi

echo ""
echo "[2] Checking robot_description topic..."
if timeout 2 ros2 topic echo /robot_description --once 2>&1 | grep -q "robot"; then
    echo "✅ robot_description is publishing"
else
    echo "❌ robot_description is NOT publishing"
    echo "   Trying to manually publish..."
    ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(ros2 run xacro xacro src/wheelchair_description/urdf/wheelchair.urdf.xacro)" &
    sleep 2
fi

echo ""
echo "[3] Checking spawn entity node..."
if ros2 node list | grep -q spawn_wheelchair; then
    echo "✅ spawn_wheelchair node exists"
    ros2 node info /spawn_wheelchair 2>&1 | head -10
else
    echo "❌ spawn_wheelchair node NOT found"
fi

echo ""
echo "[4] Manually trying to spawn robot..."
ros2 run ros_gz_sim create \
    -world world_demo \
    -entity wheelchair_test \
    -topic robot_description \
    -x 0.0 \
    -y 0.0 \
    -z 0.5

echo ""
echo "=== Debug complete ==="
