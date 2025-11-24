#!/bin/bash
# Comprehensive movement debugging script

cd /home/nadafouad/Downloads/Autonomous-Wheelchair-master-branch
source /opt/ros/humble/setup.bash

echo "=========================================="
echo "🔍 MOVEMENT DEBUGGING"
echo "=========================================="
echo ""

echo "[1] Checking bridge configuration..."
echo "Bridge node info:"
ros2 node info /gz_bridge 2>&1 | grep -E "Subscribers|Publishers|arguments" | head -10
echo ""

echo "[2] Checking topic connections..."
echo "cmd_vel topic info:"
ros2 topic info /cmd_vel 2>&1
echo ""

echo "[3] Checking if bridge is receiving messages..."
timeout 3 ros2 topic hz /cmd_vel 2>&1 | head -5 || echo "No messages detected"
echo ""

echo "[4] Testing ROS /cmd_vel topic..."
echo "Publishing to /cmd_vel..."
timeout 2 ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}" 2>&1 | head -3
echo ""

echo "[5] Testing Gazebo model/wheelchair/cmd_vel topic..."
echo "Publishing to model/wheelchair/cmd_vel..."
timeout 2 ros2 topic pub --once model/wheelchair/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}" 2>&1 | head -3
echo ""

echo "[6] Checking odometry..."
timeout 2 ros2 topic echo /odom --once 2>&1 | grep -E "linear|angular|position" | head -5 || echo "No odometry data"
echo ""

echo "[7] Checking joint states..."
timeout 2 ros2 topic echo /joint_states --once 2>&1 | grep -E "wheel|velocity" | head -10 || echo "No joint states"
echo ""

echo "[8] Checking diff drive plugin configuration..."
echo "Joint names in URDF (should match plugin):"
grep -E "wheel1_joint|wheel2_joint" autonomous_wheelchair/src/wheelchair_description/urdf/wheelchair.urdf.xacro | head -5
echo ""
echo "Plugin configuration:"
grep -A 2 "left_joint\|right_joint" autonomous_wheelchair/src/wheelchair_description/urdf/wheelchair.gazebo.xacro | head -5
echo ""

echo "[9] Checking for errors in Gazebo..."
echo "Looking for error messages..."
# This would require checking Gazebo logs, but we can't easily do that here
echo "Check Gazebo window for any error messages"
echo ""

echo "=========================================="
echo "✅ Debug complete!"
echo "=========================================="

