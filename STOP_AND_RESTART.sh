#!/bin/bash
# Stop all processes and provide clear restart instructions

echo "=========================================="
echo "🛑 STOPPING ALL PROCESSES"
echo "=========================================="
echo ""

echo "Stopping Gazebo, ROS nodes, and launch processes..."
pkill -f "gz sim\|gazebo\|ros2 launch\|robot_state_publisher\|spawn_wheelchair\|gz_bridge" 2>/dev/null || true
sleep 3

echo "✅ All processes stopped"
echo ""
echo "=========================================="
echo "📋 TO RESTART PROPERLY:"
echo "=========================================="
echo ""
echo "1. Open a NEW terminal"
echo ""
echo "2. Run this command:"
echo "   cd /home/nadafouad/Downloads/Autonomous-Wheelchair-master-branch/autonomous_wheelchair"
echo "   source /opt/ros/humble/setup.bash"
echo "   source install/setup.bash"
echo "   ros2 launch wheelchair_gazebo warehouse_with_robot.launch.py"
echo ""
echo "3. Wait 15-20 seconds for Gazebo to initialize"
echo ""
echo "4. Check that these nodes are running:"
echo "   ros2 node list"
echo "   (Should see: robot_state_publisher, spawn_wheelchair, gz_bridge)"
echo ""
echo "5. Check that robot_description is publishing:"
echo "   ros2 topic echo /robot_description --once"
echo ""
echo "6. The robot should appear in Gazebo window!"
echo ""
echo "=========================================="

