#!/bin/bash
# Debug script for turning issue

echo "=========================================="
echo "🔍 DEBUGGING TURNING ISSUE"
echo "=========================================="
echo ""

echo "[1/5] Checking if simulation is running..."
if ros2 node list 2>&1 | grep -q robot_state_publisher; then
    echo "✅ Simulation is running"
else
    echo "❌ ERROR: Simulation is not running!"
    echo "   Please launch first: ros2 launch wheelchair_gazebo warehouse_with_robot.launch.py"
    exit 1
fi
echo ""

echo "[2/5] Checking /cmd_vel topic..."
if ros2 topic list 2>&1 | grep -q "/cmd_vel"; then
    echo "✅ /cmd_vel topic exists"
    echo "   Current subscribers:"
    ros2 topic info /cmd_vel | grep -A 5 "Subscribers:"
else
    echo "❌ ERROR: /cmd_vel topic not found"
    exit 1
fi
echo ""

echo "[3/5] Checking joint states..."
echo "   Publishing a turn command (angular.z = 0.5 rad/s) for 2 seconds..."
timeout 2 ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}" \
  > /dev/null 2>&1 &
PUB_PID=$!

sleep 1
echo "   Checking joint velocities during turn..."
JOINT_VEL=$(timeout 1 ros2 topic echo /joint_states -n1 2>&1 | grep -A 20 "velocity:" | head -10)
if [ -n "$JOINT_VEL" ]; then
    echo "   Joint velocities:"
    echo "$JOINT_VEL"
else
    echo "   ⚠️  No joint velocities detected"
fi

wait $PUB_PID 2>/dev/null
echo ""

echo "[4/5] Checking odometry..."
echo "   Checking odometry during turn..."
ODOM=$(timeout 1 ros2 topic echo /odom -n1 2>&1 | grep -A 5 "twist:" | head -10)
if [ -n "$ODOM" ]; then
    echo "   Odometry twist:"
    echo "$ODOM"
else
    echo "   ⚠️  No odometry data"
fi
echo ""

echo "[5/5] Recommendations:"
echo "   1. In Gazebo, watch the wheels during turn command"
echo "   2. Check if front wheels are sliding or gripping"
echo "   3. Verify all 4 wheels are touching the ground"
echo "   4. Check wheel friction values in wheelchair.gazebo.xacro"
echo ""

echo "=========================================="
echo "✅ DEBUG COMPLETE"
echo "=========================================="
