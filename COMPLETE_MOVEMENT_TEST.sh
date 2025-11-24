#!/bin/bash
# Complete movement test with all debugging info

cd /home/nadafouad/Downloads/Autonomous-Wheelchair-master-branch
source /opt/ros/humble/setup.bash

echo "=========================================="
echo "🧪 COMPLETE MOVEMENT TEST"
echo "=========================================="
echo ""

echo "⚠️  IMPORTANT: Make sure Gazebo is running!"
echo "   If not, restart with updated launch file:"
echo "   cd autonomous_wheelchair"
echo "   source /opt/ros/humble/setup.bash"
echo "   source install/setup.bash"
echo "   ros2 launch wheelchair_gazebo warehouse_with_robot.launch.py"
echo ""
echo "Press Enter to continue..."
read

echo ""
echo "=== [1] Checking System Status ==="
echo ""

echo "Nodes:"
ros2 node list 2>/dev/null | head -10
echo ""

echo "Topics:"
ros2 topic list | grep -E "cmd_vel|odom|joint" | head -10
echo ""

echo "=== [2] Testing Movement Commands ==="
echo ""

echo "[2.1] Testing /cmd_vel topic (should work with fixed bridge)..."
timeout 3 ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" 2>&1 | head -3
echo "✅ Command sent to /cmd_vel"
sleep 2

echo ""
echo "[2.2] Testing backward movement..."
timeout 3 ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: -0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" 2>&1 | head -3
echo "✅ Command sent"
sleep 2

echo ""
echo "[2.3] Testing left turn..."
timeout 3 ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}" 2>&1 | head -3
echo "✅ Command sent"
sleep 2

echo ""
echo "[2.4] Stopping..."
timeout 3 ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" 2>&1 | head -3
echo "✅ Stop command sent"
echo ""

echo "=== [3] Checking Controller/Plugin Status ==="
echo ""

echo "[3.1] Checking odometry (should show movement if robot moved)..."
timeout 2 ros2 topic echo /odom --once 2>&1 | grep -E "position|linear|angular" | head -8 || echo "No odometry data"
echo ""

echo "[3.2] Checking joint states..."
timeout 2 ros2 topic echo /joint_states --once 2>&1 | grep -E "wheel|velocity|name" | head -10 || echo "No joint states (normal if using direct plugin)"
echo ""

echo "[3.3] Bridge status..."
if ros2 node list 2>/dev/null | grep -q gz_bridge; then
    echo "✅ Bridge is running"
    ros2 topic info /cmd_vel 2>&1 | head -5
else
    echo "❌ Bridge is NOT running!"
fi
echo ""

echo "=== [4] Debugging Tips ==="
echo ""
echo "If robot didn't move:"
echo "  1. Check Gazebo window - look for robot movement"
echo "  2. Use Gazebo's built-in Teleop panel:"
echo "     - Select 'autonomous_wheelchair' in Entity Tree"
echo "     - Go to Teleop panel"
echo "     - Set topic to '/cmd_vel' or 'model/wheelchair/cmd_vel'"
echo "     - Use buttons to test movement"
echo "  3. Check Gazebo console for plugin errors"
echo "  4. Verify bridge is receiving messages:"
echo "     ros2 topic hz /cmd_vel"
echo "  5. Check RViz TF tree for 'odom -> chassis' transform"
echo ""

echo "=========================================="
echo "✅ Test Complete!"
echo "=========================================="
echo ""
echo "📊 Check Results:"
echo "  - Did robot move in Gazebo? (Check 3D view)"
echo "  - Did odometry change? (Check /odom topic)"
echo "  - Any errors in Gazebo console?"
echo ""

