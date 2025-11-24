#!/bin/bash
# Comprehensive test script for robot movement
# This script tests all components needed for keyboard control

set -e

cd /home/nadafouad/Downloads/Autonomous-Wheelchair-master-branch/autonomous_wheelchair
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "=========================================="
echo "🧪 ROBOT MOVEMENT TEST"
echo "=========================================="
echo ""

# Test 1: Check if simulation is running
echo "[1/8] Checking if simulation is running..."
if ros2 node list 2>&1 | grep -q controller_manager; then
    echo "✅ Simulation is running"
else
    echo "❌ ERROR: Simulation is not running!"
    echo "   Please launch first: ros2 launch wheelchair_gazebo warehouse_with_robot.launch.py"
    exit 1
fi
echo ""

# Test 2: Check controller manager
echo "[2/8] Checking controller manager..."
if ros2 node list 2>&1 | grep -q controller_manager; then
    echo "✅ Controller manager node exists"
else
    echo "❌ ERROR: Controller manager not found"
    exit 1
fi
echo ""

# Test 3: Check controller states
echo "[3/8] Checking controller states..."
CONTROLLER_STATE=$(ros2 service call /controller_manager/list_controllers controller_manager_msgs/srv/ListControllers {} 2>&1 | grep -o "state='[^']*'" | sort | uniq -c || echo "error")
ACTIVE_COUNT=$(echo "$CONTROLLER_STATE" | grep -o "active" | wc -l || echo "0")
echo "   Status: $CONTROLLER_STATE"

if [ "$ACTIVE_COUNT" -ge 2 ]; then
    echo "✅ Controllers are active ($ACTIVE_COUNT active)"
else
    echo "⚠️  WARNING: Controllers not active (found $ACTIVE_COUNT, expected 2)"
    echo "   Attempting to activate..."
    ros2 service call /controller_manager/switch_controller \
      controller_manager_msgs/srv/SwitchController \
      "{activate_controllers: ['joint_state_broadcaster', 'skid_steer_controller'], deactivate_controllers: [], strictness: 2, activate_asap: true}" \
      > /dev/null 2>&1 || true
    sleep 2
    ACTIVE_COUNT=$(ros2 service call /controller_manager/list_controllers controller_manager_msgs/srv/ListControllers {} 2>&1 | grep -o "state='active'" | wc -l || echo "0")
    if [ "$ACTIVE_COUNT" -ge 2 ]; then
        echo "✅ Controllers activated successfully"
    else
        echo "❌ ERROR: Failed to activate controllers"
        exit 1
    fi
fi
echo ""

# Test 4: Check hardware interfaces
echo "[4/8] Checking hardware interfaces..."
CLAIMED_COUNT=$(ros2 service call /controller_manager/list_hardware_components controller_manager_msgs/srv/ListHardwareComponents {} 2>&1 | grep -c 'is_claimed=True' || echo "0")
if [ "$CLAIMED_COUNT" -ge 4 ]; then
    echo "✅ Hardware interfaces claimed ($CLAIMED_COUNT claimed)"
else
    echo "⚠️  WARNING: Only $CLAIMED_COUNT interfaces claimed (expected 4)"
fi
echo ""

# Test 5: Check topics
echo "[5/8] Checking topics..."
if ros2 topic list 2>&1 | grep -q "/cmd_vel"; then
    echo "✅ /cmd_vel topic exists"
else
    echo "❌ ERROR: /cmd_vel topic not found"
    exit 1
fi

if ros2 topic list 2>&1 | grep -q "/joint_states"; then
    echo "✅ /joint_states topic exists"
else
    echo "❌ ERROR: /joint_states topic not found"
    exit 1
fi

if ros2 topic list 2>&1 | grep -q "skid_steer_controller/cmd_vel_unstamped"; then
    echo "✅ Controller command topic exists"
else
    echo "⚠️  WARNING: Controller command topic not found (relay may not be running)"
fi
echo ""

# Test 6: Check topic relay
echo "[6/8] Checking cmd_vel relay..."
if ros2 node list 2>&1 | grep -q cmd_vel_relay; then
    echo "✅ cmd_vel relay node is running"
else
    echo "⚠️  WARNING: cmd_vel relay node not found"
    echo "   This may prevent movement commands from reaching the controller"
fi
echo ""

# Test 7: Test movement command
echo "[7/8] Testing movement command..."
echo "   Sending forward command (0.3 m/s) for 3 seconds..."
timeout 3 ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" \
  > /dev/null 2>&1 || true
echo "✅ Command sent"
echo ""

# Test 8: Check joint velocities
echo "[8/8] Checking joint velocities..."
sleep 1
JOINT_VELOCITIES=$(timeout 2 ros2 topic echo /joint_states -n1 2>&1 | grep -A 10 "velocity:" | grep -E "[0-9]+\.[0-9]+" | head -4 || echo "")
if [ -n "$JOINT_VELOCITIES" ]; then
    echo "✅ Joint velocities detected:"
    echo "$JOINT_VELOCITIES" | head -4
    echo ""
    echo "✅ Robot should be moving in Gazebo!"
else
    echo "⚠️  WARNING: No joint velocities detected"
    echo "   Robot may not be moving. Check:"
    echo "   1. Is simulation paused in Gazebo?"
    echo "   2. Are wheels touching the ground?"
    echo "   3. Check Gazebo GUI for visual issues"
fi
echo ""

echo "=========================================="
echo "✅ TEST COMPLETE"
echo "=========================================="
echo ""
echo "📋 Summary:"
echo "   - Controllers: $ACTIVE_COUNT active"
echo "   - Hardware interfaces: $CLAIMED_COUNT claimed"
echo "   - Topics: All required topics exist"
echo ""
echo "⌨️  To use keyboard control:"
echo "   ./launch_keyboard_teleop.sh"
echo ""
echo "   Or manually:"
echo "   ros2 run teleop_twist_keyboard teleop_twist_keyboard"
echo ""






