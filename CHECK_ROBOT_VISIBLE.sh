#!/bin/bash
# Check if robot is visible and working in Gazebo

source /opt/ros/humble/setup.bash

echo "=========================================="
echo "🔍 CHECKING ROBOT VISIBILITY"
echo "=========================================="
echo ""

echo "[1] robot_state_publisher status:"
if ros2 node list 2>/dev/null | grep -q robot_state_publisher; then
    echo "✅ robot_state_publisher is RUNNING"
else
    echo "❌ robot_state_publisher is NOT running"
    exit 1
fi

echo ""
echo "[2] robot_description topic:"
if timeout 2 ros2 topic echo /robot_description --once 2>/dev/null | grep -q "robot"; then
    echo "✅ robot_description is PUBLISHING"
else
    echo "❌ robot_description is NOT publishing"
fi

echo ""
echo "[3] spawn_wheelchair node:"
if ros2 node list 2>/dev/null | grep -q spawn_wheelchair; then
    echo "✅ spawn_wheelchair node EXISTS"
else
    echo "⚠️  spawn_wheelchair node NOT found (may still be starting)"
fi

echo ""
echo "[4] Active topics (robot should publish these):"
TOPICS=$(ros2 topic list 2>/dev/null)
if echo "$TOPICS" | grep -q "/odom"; then
    echo "✅ /odom - Odometry topic exists"
    if timeout 1 ros2 topic hz /odom 2>&1 | grep -q "average"; then
        echo "   → Publishing data!"
    else
        echo "   → Not publishing yet (wait a few seconds)"
    fi
else
    echo "❌ /odom - Not found"
fi

if echo "$TOPICS" | grep -q "/joint_states"; then
    echo "✅ /joint_states - Joint states topic exists"
else
    echo "❌ /joint_states - Not found"
fi

if echo "$TOPICS" | grep -q "/scan"; then
    echo "✅ /scan - LiDAR topic exists"
else
    echo "❌ /scan - Not found"
fi

echo ""
echo "[5] TF frames (robot should have these):"
if timeout 2 ros2 run tf2_ros tf2_echo chassis odom 2>&1 | grep -q "Translation\|At time"; then
    echo "✅ TF transform from odom to chassis exists (robot is in world!)"
else
    echo "⚠️  TF transform not found yet (robot may still be spawning)"
fi

echo ""
echo "=========================================="
echo "📋 WHAT TO CHECK IN GAZEBO:"
echo "=========================================="
echo ""
echo "1. Look at the Gazebo window"
echo "2. The robot should be visible in the warehouse"
echo "3. If you don't see it:"
echo "   - Try zooming out (mouse wheel)"
echo "   - Try pressing 'Home' key to reset view"
echo "   - Check if robot spawned above ground (z=2.0)"
echo "   - Look for orange/black robot model"
echo ""
echo "4. If robot is there but not moving:"
echo "   - Wait 20-30 seconds for full initialization"
echo "   - Then test movement with: ./QUICK_TEST_MOVEMENT.sh"
echo ""
echo "=========================================="
