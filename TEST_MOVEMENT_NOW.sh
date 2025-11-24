#!/bin/bash
# Test robot movement after fixes

cd /home/nadafouad/Downloads/Autonomous-Wheelchair-master-branch
source /opt/ros/humble/setup.bash
source autonomous_wheelchair/install/setup.bash

echo "=========================================="
echo "🤖 TESTING ROBOT MOVEMENT"
echo "=========================================="
echo ""

echo "[1] Checking if robot is ready..."
echo ""

# Check if topics exist
echo "Checking /cmd_vel topic:"
if timeout 1 ros2 topic list 2>/dev/null | grep -q "/cmd_vel"; then
    echo "✅ /cmd_vel topic exists"
else
    echo "❌ /cmd_vel topic NOT found - Gazebo might not be running"
    echo "   Launch Gazebo first: ros2 launch wheelchair_gazebo warehouse_with_robot.launch.py"
    exit 1
fi

echo ""
echo "Checking /odom topic:"
if timeout 1 ros2 topic list 2>/dev/null | grep -q "/odom"; then
    echo "✅ /odom topic exists"
else
    echo "⚠️  /odom topic NOT found - diff drive plugin might not be loaded"
fi

echo ""
echo "Checking /joint_states topic:"
if timeout 1 ros2 topic list 2>/dev/null | grep -q "/joint_states"; then
    echo "✅ /joint_states topic exists"
else
    echo "⚠️  /joint_states topic NOT found"
fi

echo ""
echo "=========================================="
echo "[2] TESTING MOVEMENT COMMANDS"
echo "=========================================="
echo ""

echo "Test 1: Move FORWARD (0.5 m/s) for 2 seconds..."
echo "Publishing to /cmd_vel..."
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
echo "✅ Command sent"
echo ""
echo "Wait 2 seconds and check if robot moved..."
sleep 2

echo ""
echo "Test 2: Move BACKWARD (-0.5 m/s) for 2 seconds..."
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: -0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
echo "✅ Command sent"
echo ""
echo "Wait 2 seconds..."
sleep 2

echo ""
echo "Test 3: ROTATE LEFT (0.5 rad/s) for 2 seconds..."
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"
echo "✅ Command sent"
echo ""
echo "Wait 2 seconds..."
sleep 2

echo ""
echo "Test 4: ROTATE RIGHT (-0.5 rad/s) for 2 seconds..."
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.5}}"
echo "✅ Command sent"
echo ""
echo "Wait 2 seconds..."
sleep 2

echo ""
echo "Test 5: STOP robot..."
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
echo "✅ Stop command sent"
echo ""

echo "=========================================="
echo "[3] CHECKING ODOMETRY"
echo "=========================================="
echo ""

echo "Current odometry (check if position changed):"
timeout 2 ros2 topic echo /odom --once 2>&1 | grep -A 5 "pose:" | head -10 || echo "No odometry data"

echo ""
echo "=========================================="
echo "✅ MOVEMENT TEST COMPLETE"
echo "=========================================="
echo ""
echo "What to check:"
echo "  1. In Gazebo: Did the robot move?"
echo "  2. In RViz: Did the robot position change?"
echo "  3. Odometry: Did position values change?"
echo ""
echo "If robot didn't move, check:"
echo "  - Gazebo console for plugin errors"
echo "  - Bridge is running: ros2 node list | grep bridge"
echo "  - Wheel transforms in RViz (should be OK now)"
echo ""

