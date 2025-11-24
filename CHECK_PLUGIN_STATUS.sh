#!/bin/bash
# Check if diff drive plugin is loaded and working

cd /home/nadafouad/Downloads/Autonomous-Wheelchair-master-branch
source /opt/ros/humble/setup.bash

echo "=========================================="
echo "🔍 CHECKING DIFF DRIVE PLUGIN STATUS"
echo "=========================================="
echo ""

echo "[1] Checking if bridge is forwarding messages..."
echo "Publishing to /cmd_vel and checking bridge activity..."
timeout 2 ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}" 2>&1 | head -3
echo ""

echo "[2] Checking odometry (should update if robot moves)..."
echo "Current odometry:"
timeout 2 ros2 topic echo /odom --once 2>&1 | grep -A 3 "pose:" | head -8 || echo "No odometry"
echo ""

echo "[3] Possible Issues if Robot Doesn't Move:"
echo ""
echo "  a) Diff drive plugin not loaded:"
echo "     - Check Gazebo console for plugin errors"
echo "     - Look for 'DiffDrive' or 'diff-drive' messages"
echo ""
echo "  b) Joint names don't match:"
echo "     - Plugin expects: wheel1_joint, wheel2_joint"
echo "     - Check if these joints exist in spawned model"
echo ""
echo "  c) Robot stuck or colliding:"
echo "     - Check if robot is floating (z=2.0 might be too high)"
echo "     - Check if robot is colliding with ground"
echo "     - Try resetting robot position in Gazebo"
echo ""
echo "  d) Plugin parameters wrong:"
echo "     - wheel_separation: 0.52 m"
echo "     - wheel_radius: 0.15 m"
echo "     - Check if these match actual robot dimensions"
echo ""

echo "[4] Debugging Steps:"
echo ""
echo "  1. In Gazebo, select 'autonomous_wheelchair' in Entity Tree"
echo "  2. Check Component Inspector for plugin status"
echo "  3. Use Gazebo Teleop panel:"
echo "     - Set topic to: model/wheelchair/cmd_vel"
echo "     - Use buttons to test movement"
echo "     - If Gazebo teleop works → Plugin is fine, bridge issue"
echo "     - If Gazebo teleop doesn't work → Plugin issue"
echo ""
echo "  5. Check Gazebo console for errors"
echo "  6. Verify robot spawn height (should be on ground, not floating)"
echo ""

echo "=========================================="

