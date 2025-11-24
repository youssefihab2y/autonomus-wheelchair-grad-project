#!/bin/bash
# Test movement after bridge fix

cd /home/nadafouad/Downloads/Autonomous-Wheelchair-master-branch
source /opt/ros/humble/setup.bash

echo "=========================================="
echo "🧪 TESTING MOVEMENT (After Bridge Fix)"
echo "=========================================="
echo ""

echo "IMPORTANT: Make sure Gazebo is running with the updated launch file!"
echo "If not, restart with:"
echo "  cd autonomous_wheelchair"
echo "  source /opt/ros/humble/setup.bash"
echo "  source install/setup.bash"
echo "  ros2 launch wheelchair_gazebo warehouse_with_robot.launch.py"
echo ""
echo "Press Enter to continue testing..."
read

echo ""
echo "[1] Checking bridge is running..."
if ros2 node list 2>/dev/null | grep -q gz_bridge; then
    echo "✅ Bridge is running"
else
    echo "❌ Bridge is NOT running - restart Gazebo!"
    exit 1
fi

echo ""
echo "[2] Checking cmd_vel topic..."
ros2 topic info /cmd_vel 2>&1 | head -5

echo ""
echo "[3] Testing movement - Forward (0.5 m/s)..."
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
sleep 3

echo ""
echo "[4] Testing movement - Backward (-0.5 m/s)..."
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: -0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
sleep 3

echo ""
echo "[5] Testing movement - Turn left (0.5 rad/s)..."
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"
sleep 3

echo ""
echo "[6] Stopping..."
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

echo ""
echo "[7] Checking odometry..."
timeout 2 ros2 topic echo /odom --once 2>&1 | grep -E "linear|angular|position" | head -5 || echo "No odometry yet"

echo ""
echo "=========================================="
echo "✅ Movement test complete!"
echo ""
echo "Check Gazebo window - robot should have moved!"
echo "If robot didn't move, check:"
echo "  1. Bridge is running: ros2 node list | grep bridge"
echo "  2. cmd_vel has subscribers: ros2 topic info /cmd_vel"
echo "  3. Check Gazebo for error messages"
echo "=========================================="

