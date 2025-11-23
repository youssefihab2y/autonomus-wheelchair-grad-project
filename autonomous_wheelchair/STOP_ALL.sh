#!/bin/bash
# Stop all ROS 2 and Gazebo processes

echo "🛑 Stopping ALL ROS and Gazebo processes..."

# Kill Gazebo processes (all variants)
killall -9 gz-server 2>/dev/null || true
killall -9 gz-client 2>/dev/null || true
killall -9 gz-sim 2>/dev/null || true
killall -9 gz 2>/dev/null || true
killall -9 ign-gazebo 2>/dev/null || true
killall -9 ign 2>/dev/null || true
killall -9 ruby 2>/dev/null || true  # Gazebo uses ruby wrapper

# Kill ROS 2 processes
killall -9 ros2_control_node 2>/dev/null || true
killall -9 controller_manager 2>/dev/null || true
killall -9 parameter_bridge 2>/dev/null || true
killall -9 gz_bridge 2>/dev/null || true
killall -9 robot_state_publisher 2>/dev/null || true
killall -9 static_transform_publisher 2>/dev/null || true
killall -9 rviz2 2>/dev/null || true
killall -9 teleop_twist_keyboard 2>/dev/null || true
killall -9 create 2>/dev/null || true  # ros_gz_sim create

# Kill ROS 2 launch processes
pkill -9 -f "ros2 launch" 2>/dev/null || true
pkill -9 -f "warehouse" 2>/dev/null || true
pkill -9 -f "spawn_robot" 2>/dev/null || true
pkill -9 -f "spawner" 2>/dev/null || true
pkill -9 -f "relay" 2>/dev/null || true
pkill -9 -f "ign gazebo" 2>/dev/null || true
pkill -9 -f "gz sim" 2>/dev/null || true

sleep 2

echo "✅ All processes stopped"
echo ""
echo "Verify with: ros2 node list"
echo "Should return nothing or very few nodes"
