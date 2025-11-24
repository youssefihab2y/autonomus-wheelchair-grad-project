#!/bin/bash
# Fix robot spawn issue

cd /home/nadafouad/Downloads/Autonomous-Wheelchair-master-branch/autonomous_wheelchair
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "=========================================="
echo "🔧 FIXING ROBOT SPAWN"
echo "=========================================="
echo ""

echo "[1] Checking robot_description topic..."
if timeout 2 ros2 topic echo /robot_description --once 2>&1 | grep -q "robot"; then
    echo "✅ robot_description is available"
else
    echo "❌ robot_description not available"
    echo "   Starting robot_state_publisher manually..."
    ros2 run robot_state_publisher robot_state_publisher &
    sleep 3
fi

echo ""
echo "[2] Checking if robot already exists in Gazebo..."
if ros2 topic list 2>/dev/null | grep -q "/odom"; then
    if timeout 1 ros2 topic hz /odom 2>&1 | grep -q "average"; then
        echo "✅ Robot appears to be spawned (odometry is publishing)"
        echo "   Check Gazebo window - robot should be visible!"
        exit 0
    fi
fi

echo ""
echo "[3] Attempting to spawn robot manually..."
ros2 run ros_gz_sim create \
    -world world_demo \
    -entity wheelchair \
    -topic robot_description \
    -x 0.0 \
    -y 0.0 \
    -z 0.5

echo ""
echo "Waiting 5 seconds for spawn to complete..."
sleep 5

echo ""
echo "[4] Verifying spawn..."
if timeout 2 ros2 topic hz /odom 2>&1 | grep -q "average"; then
    echo "✅ Robot spawned successfully!"
    echo "   Check Gazebo window - robot should be visible now!"
else
    echo "⚠️  Robot may still be spawning, or there's an issue"
    echo "   Check Gazebo window and look for any error messages"
fi

echo ""
echo "=========================================="
