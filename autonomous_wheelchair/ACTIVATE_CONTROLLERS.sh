#!/bin/bash
# Quick script to activate controllers

cd /home/nadafouad/Downloads/Autonomous-Wheelchair-master-branch/autonomous_wheelchair
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "🔧 Activating Controllers..."
echo ""

# Activate both controllers
ros2 service call /controller_manager/switch_controller \
  controller_manager_msgs/srv/SwitchController \
  "{activate_controllers: ['joint_state_broadcaster', 'skid_steer_controller'], deactivate_controllers: [], strictness: 2, activate_asap: true}" \
  2>&1 | grep -E "ok=|ERROR" | head -3

sleep 2

echo ""
echo "📊 Controller Status:"
ros2 service call /controller_manager/list_controllers controller_manager_msgs/srv/ListControllers {} 2>&1 | grep -o "state='[^']*'" | sort | uniq -c

echo ""
echo "✅ If you see '2 state='active'' then controllers are working!"
