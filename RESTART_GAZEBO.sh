#!/bin/bash
# Restart Gazebo with robot - ensures everything starts properly

echo "=========================================="
echo "🔄 RESTARTING GAZEBO WITH ROBOT"
echo "=========================================="
echo ""

# Kill existing processes
echo "[1/4] Stopping existing Gazebo and ROS nodes..."
pkill -f "gz sim\|gazebo\|ros2 launch" 2>/dev/null || true
sleep 2

# Kill any remaining processes
pkill -f "robot_state_publisher\|spawn_wheelchair\|gz_bridge" 2>/dev/null || true
sleep 1

echo "✅ Stopped existing processes"
echo ""

# Build workspace
echo "[2/4] Building workspace..."
cd /home/nadafouad/Downloads/Autonomous-Wheelchair-master-branch/autonomous_wheelchair
source /opt/ros/humble/setup.bash
colcon build --symlink-install > /dev/null 2>&1
echo "✅ Workspace built"
echo ""

# Launch
echo "[3/4] Launching Gazebo with robot..."
echo "   (This will take 15-20 seconds to initialize)"
echo ""
source install/setup.bash

# Launch in background and capture output
ros2 launch wheelchair_gazebo warehouse_with_robot.launch.py > /tmp/gazebo_launch.log 2>&1 &
LAUNCH_PID=$!

echo "Launch process started (PID: $LAUNCH_PID)"
echo ""

# Wait and check status
echo "[4/4] Waiting for initialization..."
for i in {1..20}; do
    sleep 1
    if ros2 node list 2>/dev/null | grep -q robot_state_publisher; then
        echo "✅ robot_state_publisher is running!"
        break
    fi
    echo -n "."
done
echo ""

# Final status check
echo ""
echo "=== Status Check ==="
if ros2 node list 2>/dev/null | grep -q robot_state_publisher; then
    echo "✅ robot_state_publisher: RUNNING"
else
    echo "❌ robot_state_publisher: NOT RUNNING"
    echo "   Check logs: tail -f /tmp/gazebo_launch.log"
fi

if ros2 topic list 2>/dev/null | grep -q robot_description; then
    echo "✅ robot_description topic: EXISTS"
    if timeout 2 ros2 topic echo /robot_description --once 2>/dev/null | grep -q robot; then
        echo "✅ robot_description: PUBLISHING"
    else
        echo "⚠️  robot_description: NOT PUBLISHING YET (wait a few more seconds)"
    fi
else
    echo "❌ robot_description topic: NOT FOUND"
fi

if ros2 node list 2>/dev/null | grep -q spawn_wheelchair; then
    echo "✅ spawn_wheelchair: RUNNING"
else
    echo "⚠️  spawn_wheelchair: NOT RUNNING YET (wait a few more seconds)"
fi

echo ""
echo "=========================================="
echo "✅ Launch complete!"
echo ""
echo "Check Gazebo window - robot should appear in 10-15 seconds"
echo "If robot doesn't appear, check: tail -f /tmp/gazebo_launch.log"
echo "=========================================="

