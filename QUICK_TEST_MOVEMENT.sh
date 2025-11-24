#!/bin/bash
# Quick test - send movement commands programmatically

cd /home/nadafouad/Downloads/Autonomous-Wheelchair-master-branch/autonomous_wheelchair
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "=========================================="
echo "🧪 QUICK MOVEMENT TEST"
echo "=========================================="
echo ""

echo "[1/4] Testing forward movement (0.5 m/s)..."
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
sleep 2

echo "[2/4] Testing backward movement (-0.5 m/s)..."
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: -0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
sleep 2

echo "[3/4] Testing left turn (0.5 rad/s)..."
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"
sleep 2

echo "[4/4] Testing right turn (-0.5 rad/s)..."
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.5}}"
sleep 2

echo ""
echo "[5/5] Stopping robot..."
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

echo ""
echo "✅ Movement test complete!"
echo ""
echo "Check Gazebo window - the robot should have moved!"
echo "Check RViz window - you should see the robot model and sensor data!"

