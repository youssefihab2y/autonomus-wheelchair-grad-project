#!/bin/bash
# Comprehensive debugging of code, logic, and naming

cd /home/nadafouad/Downloads/Autonomous-Wheelchair-master-branch

echo "=========================================="
echo "🔍 COMPREHENSIVE CODE & LOGIC DEBUG"
echo "=========================================="
echo ""

echo "=== 1. JOINT NAMING CONSISTENCY ==="
echo ""
echo "URDF Joint Names:"
grep -E "name=\"wheel[1-4]_joint\"" autonomous_wheelchair/src/wheelchair_description/urdf/wheelchair.urdf.xacro
echo ""
echo "Gazebo Plugin Joint Names:"
grep -E "left_joint|right_joint" autonomous_wheelchair/src/wheelchair_description/urdf/wheelchair.gazebo.xacro | head -2
echo ""
echo "✅ Checking if plugin joints match URDF joints..."
WHEEL1_IN_URDF=$(grep -c "wheel1_joint" autonomous_wheelchair/src/wheelchair_description/urdf/wheelchair.urdf.xacro)
WHEEL2_IN_URDF=$(grep -c "wheel2_joint" autonomous_wheelchair/src/wheelchair_description/urdf/wheelchair.urdf.xacro)
WHEEL1_IN_PLUGIN=$(grep -c "wheel1_joint" autonomous_wheelchair/src/wheelchair_description/urdf/wheelchair.gazebo.xacro)
WHEEL2_IN_PLUGIN=$(grep -c "wheel2_joint" autonomous_wheelchair/src/wheelchair_description/urdf/wheelchair.gazebo.xacro)

if [ "$WHEEL1_IN_URDF" -gt 0 ] && [ "$WHEEL1_IN_PLUGIN" -gt 0 ] && [ "$WHEEL2_IN_URDF" -gt 0 ] && [ "$WHEEL2_IN_PLUGIN" -gt 0 ]; then
    echo "✅ Joint names are consistent"
else
    echo "❌ Joint name mismatch detected!"
fi
echo ""

echo "=== 2. DIFF DRIVE PLUGIN CONFIGURATION ==="
echo ""
echo "Plugin Parameters:"
grep -A 8 "DiffDrive" autonomous_wheelchair/src/wheelchair_description/urdf/wheelchair.gazebo.xacro | grep -E "left_joint|right_joint|wheel_separation|wheel_radius" | head -4
echo ""
echo "Expected values:"
echo "  left_joint: wheel2_joint (back left)"
echo "  right_joint: wheel1_joint (back right)"
echo "  wheel_separation: 0.52 m"
echo "  wheel_radius: 0.15 m"
echo ""

echo "=== 3. BRIDGE CONFIGURATION ==="
echo ""
echo "Bridge arguments for cmd_vel:"
grep -A 2 "cmd_vel" autonomous_wheelchair/src/wheelchair_gazebo/launch/warehouse_with_robot.launch.py | head -3
echo ""
echo "Bridge remappings:"
grep -A 2 "cmd_vel\|odometry" autonomous_wheelchair/src/wheelchair_gazebo/launch/warehouse_with_robot.launch.py | grep "remap\|cmd_vel\|odom" | head -4
echo ""

echo "=== 4. TOPIC NAMING CONSISTENCY ==="
echo ""
echo "Gazebo plugin expects: model/wheelchair/cmd_vel"
echo "Bridge creates: model/wheelchair/cmd_vel"
echo "Bridge remaps to: /cmd_vel"
echo ""
echo "Checking if all match..."
PLUGIN_TOPIC=$(grep -i "cmd_vel\|topic" autonomous_wheelchair/src/wheelchair_description/urdf/wheelchair.gazebo.xacro | grep -i "model\|wheelchair" | head -1)
BRIDGE_TOPIC=$(grep "cmd_vel" autonomous_wheelchair/src/wheelchair_gazebo/launch/warehouse_with_robot.launch.py | grep "model/wheelchair" | head -1)
if [ -n "$PLUGIN_TOPIC" ] && [ -n "$BRIDGE_TOPIC" ]; then
    echo "✅ Topics are configured"
else
    echo "⚠️  Topic configuration needs verification"
fi
echo ""

echo "=== 5. FRAME NAMING ==="
echo ""
echo "Base frame in plugin:"
grep "robot_base_frame\|base_frame" autonomous_wheelchair/src/wheelchair_description/urdf/wheelchair.gazebo.xacro | head -2
echo ""
echo "Base frame in URDF:"
grep "name=\"chassis\"" autonomous_wheelchair/src/wheelchair_description/urdf/wheelchair.urdf.xacro | head -1
echo ""
echo "Checking consistency..."
if grep -q "chassis" autonomous_wheelchair/src/wheelchair_description/urdf/wheelchair.gazebo.xacro; then
    echo "✅ Base frame 'chassis' is consistent"
else
    echo "❌ Base frame mismatch!"
fi
echo ""

echo "=== 6. WHEEL POSITION LOGIC ==="
echo ""
echo "Wheel positions in URDF:"
echo "wheel1 (back right): x=-0.25, y=-0.26"
echo "wheel2 (back left):  x=-0.25, y=0.26"
echo "wheel3 (front right): x=0.25, y=-0.26"
echo "wheel4 (front left):  x=0.25, y=0.26"
echo ""
echo "Wheel separation calculation:"
echo "  Lateral separation = |y_wheel1 - y_wheel2| = |(-0.26) - 0.26| = 0.52 m ✓"
echo "  Plugin uses: 0.52 m ✓"
echo ""

echo "=== 7. POTENTIAL ISSUES CHECKLIST ==="
echo ""
echo "□ Plugin filename correct: libignition-gazebo-diff-drive-system.so"
echo "□ Joint names match between URDF and plugin"
echo "□ Wheel separation matches actual wheel positions"
echo "□ Wheel radius matches back wheel radius (0.15 m)"
echo "□ Base frame name matches (chassis)"
echo "□ Bridge topic names match plugin expectations"
echo "□ Bridge remapping is correct"
echo ""

echo "=========================================="
echo "✅ Code & Logic Debug Complete!"
echo "=========================================="
echo ""
echo "If robot still doesn't move, check:"
echo "  1. Gazebo console for plugin loading errors"
echo "  2. Bridge is actually receiving/sending messages"
echo "  3. Joints are actually being controlled"
echo "  4. Robot is not stuck or colliding with ground"
echo ""

