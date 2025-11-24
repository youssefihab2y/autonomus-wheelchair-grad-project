#!/bin/bash
# Debug why diff drive plugin isn't working

cd /home/nadafouad/Downloads/Autonomous-Wheelchair-master-branch
source /opt/ros/humble/setup.bash

echo "=========================================="
echo "🔍 DEBUGGING PLUGIN LOADING"
echo "=========================================="
echo ""

echo "[1] Checking if plugin file exists..."
if [ -f "/opt/ros/humble/lib/libignition-gazebo-diff-drive-system.so" ]; then
    echo "✅ Plugin file exists"
else
    echo "❌ Plugin file NOT found!"
    echo "   Looking for: libignition-gazebo-diff-drive-system.so"
    echo "   Check: ls /opt/ros/humble/lib/ | grep diff"
    ls /opt/ros/humble/lib/ | grep -i diff || echo "No diff drive plugin found"
fi
echo ""

echo "[2] Checking plugin reference in URDF..."
echo "Plugin should be in <gazebo> tag (not <gazebo reference=...>)"
grep -A 5 "DiffDrive" autonomous_wheelchair/src/wheelchair_description/urdf/wheelchair.gazebo.xacro | head -8
echo ""

echo "[3] Checking joint names in spawned model..."
echo "Plugin expects: wheel1_joint, wheel2_joint"
echo "Check Gazebo Entity Tree - joints should be listed"
echo ""

echo "[4] Plugin Configuration Check:"
echo "  - left_joint: wheel2_joint"
echo "  - right_joint: wheel1_joint"
echo "  - wheel_separation: 0.52"
echo "  - wheel_radius: 0.15"
echo "  - odom_frame_id: odom"
echo "  - robot_base_frame: chassis"
echo ""

echo "[5] CRITICAL: Plugin must be in <gazebo> tag (not <gazebo reference=...>)"
echo "   Current plugin is in standalone <gazebo> tag - this is CORRECT"
echo ""

echo "[6] What to check in Gazebo:"
echo "   1. Open Gazebo console (bottom panel)"
echo "   2. Look for errors about 'DiffDrive' or 'diff-drive'"
echo "   3. Look for 'plugin' or 'system' loading messages"
echo "   4. Check if plugin loaded successfully"
echo ""

echo "[7] Alternative: Check if plugin needs to be attached to a link"
echo "   Some plugins need <gazebo reference='chassis'> instead of standalone"
echo ""

echo "=========================================="
echo "Next: Check Gazebo console for plugin errors!"
echo "=========================================="

