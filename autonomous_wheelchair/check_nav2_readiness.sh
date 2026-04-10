#!/bin/bash
# Nav2 Readiness Check Script for Autonomous Wheelchair

echo "=========================================="
echo "🔍 NAV2 READINESS CHECK"
echo "=========================================="
echo ""

# Source ROS 2
source /opt/ros/humble/setup.bash
cd /home/nadafouad/Downloads/Autonomous-Wheelchair-master-branch/autonomous_wheelchair
source install/setup.bash 2>/dev/null

ERRORS=0
WARNINGS=0

# 1. Check Required Topics
echo "[1/8] Checking Required Topics..."
REQUIRED_TOPICS=("/cmd_vel" "/odom" "/scan" "/tf" "/tf_static" "/robot_description")
for topic in "${REQUIRED_TOPICS[@]}"; do
    if ros2 topic list 2>/dev/null | grep -q "^${topic}$"; then
        echo "  ✅ $topic exists"
    else
        echo "  ❌ $topic MISSING"
        ((ERRORS++))
    fi
done
echo ""

# 2. Check Topic Publishers
echo "[2/8] Checking Topic Publishers..."
if ros2 topic info /cmd_vel 2>/dev/null | grep -q "Publisher count: [1-9]"; then
    echo "  ✅ /cmd_vel has publisher"
else
    echo "  ⚠️  /cmd_vel has no publisher (will need teleop or Nav2)"
    ((WARNINGS++))
fi

if ros2 topic info /odom 2>/dev/null | grep -q "Publisher count: [1-9]"; then
    echo "  ✅ /odom has publisher"
    ODOM_RATE=$(timeout 2 ros2 topic hz /odom 2>&1 | grep "average rate" | awk '{print $3}')
    if [ -n "$ODOM_RATE" ]; then
        echo "     Publishing at ~${ODOM_RATE} Hz"
    fi
else
    echo "  ❌ /odom has no publisher"
    ((ERRORS++))
fi

if ros2 topic info /scan 2>/dev/null | grep -q "Publisher count: [1-9]"; then
    echo "  ✅ /scan has publisher"
    SCAN_RATE=$(timeout 2 ros2 topic hz /scan 2>&1 | grep "average rate" | awk '{print $3}')
    if [ -n "$SCAN_RATE" ]; then
        echo "     Publishing at ~${SCAN_RATE} Hz"
    fi
else
    echo "  ❌ /scan has no publisher"
    ((ERRORS++))
fi
echo ""

# 3. Check TF Frames
echo "[3/8] Checking TF Frames..."
# Check if odom -> base_link chain exists (direct or via base_uplayer_link)
if timeout 2 ros2 run tf2_ros tf2_echo odom base_link 2>&1 | grep -q "Translation:"; then
    echo "  ✅ odom -> base_link chain exists"
else
    # Try alternative path: odom -> base_uplayer_link
    if timeout 2 ros2 run tf2_ros tf2_echo odom base_uplayer_link 2>&1 | grep -q "Translation:"; then
        echo "  ✅ odom -> base_uplayer_link exists"
        # Check if base_link is reachable via base_uplayer_link
        if timeout 2 ros2 run tf2_ros tf2_echo base_uplayer_link base_link 2>&1 | grep -q "Translation:"; then
            echo "  ✅ base_link reachable via base_uplayer_link (full chain: odom -> base_uplayer_link -> base_link)"
        else
            echo "  ⚠️  base_link not reachable, but odom -> base_uplayer_link works (Nav2 can use base_uplayer_link as base_frame)"
            ((WARNINGS++))
        fi
    else
        echo "  ❌ odom frame NOT in TF tree (odom_to_tf may not be receiving messages)"
        ((ERRORS++))
    fi
fi

# Check chassis connectivity (for robot model)
if timeout 2 ros2 run tf2_ros tf2_echo base_link chassis 2>&1 | grep -q "Translation:"; then
    echo "  ✅ chassis reachable from base_link"
elif timeout 2 ros2 run tf2_ros tf2_echo base_uplayer_link chassis 2>&1 | grep -q "Translation:"; then
    echo "  ✅ chassis reachable via base_uplayer_link"
else
    echo "  ⚠️  chassis not found (check robot_state_publisher)"
    ((WARNINGS++))
fi
echo ""

# 4. Check TF Chain (odom -> base_link)
echo "[4/8] Checking TF Chain (odom -> base_link)..."
if timeout 3 ros2 run tf2_ros tf2_echo odom base_link 2>&1 | grep -q "Translation:"; then
    echo "  ✅ TF chain odom -> base_link exists"
    TRANSFORM=$(timeout 2 ros2 run tf2_ros tf2_echo odom base_link 2>&1 | grep "Translation:" | head -1)
    echo "     $TRANSFORM"
else
    echo "  ❌ TF chain odom -> base_link MISSING"
    ((ERRORS++))
fi
echo ""

# 5. Check Odometry Message Format
echo "[5/8] Checking Odometry Message Format..."
ODOM_CHECK=$(timeout 2 ros2 topic echo /odom --once 2>&1 | grep -E "frame_id|child_frame_id" | head -2)
if echo "$ODOM_CHECK" | grep -q "frame_id.*odom"; then
    echo "  ✅ Odometry frame_id is 'odom'"
else
    echo "  ⚠️  Odometry frame_id: $(echo "$ODOM_CHECK" | grep frame_id)"
    ((WARNINGS++))
fi

if echo "$ODOM_CHECK" | grep -q "child_frame_id.*base"; then
    echo "  ✅ Odometry child_frame_id contains 'base'"
else
    echo "  ⚠️  Odometry child_frame_id: $(echo "$ODOM_CHECK" | grep child_frame_id)"
    ((WARNINGS++))
fi
echo ""

# 6. Check LaserScan Format
echo "[6/8] Checking LaserScan Format..."
SCAN_CHECK=$(timeout 2 ros2 topic echo /scan --once 2>&1 | grep -E "frame_id|angle_min|angle_max|range_min|range_max" | head -5)
if echo "$SCAN_CHECK" | grep -q "frame_id"; then
    SCAN_FRAME=$(echo "$SCAN_CHECK" | grep frame_id | awk '{print $2}' | tr -d '"')
    echo "  ✅ LaserScan frame_id: $SCAN_FRAME"
    
    # Check if frame exists in TF
    if timeout 2 ros2 run tf2_ros tf2_echo base_link "$SCAN_FRAME" 2>&1 | grep -q "Translation:"; then
        echo "     ✅ LaserScan frame exists in TF tree"
    else
        echo "     ⚠️  LaserScan frame may need TF bridge"
        ((WARNINGS++))
    fi
else
    echo "  ❌ Cannot read LaserScan message"
    ((ERRORS++))
fi
echo ""

# 7. Check Robot Description
echo "[7/8] Checking Robot Description..."
if ros2 param get /robot_state_publisher robot_description 2>/dev/null | grep -q "robot"; then
    echo "  ✅ Robot description available"
    URDF_SIZE=$(ros2 param get /robot_state_publisher robot_description 2>/dev/null | wc -l)
    echo "     URDF size: ~$URDF_SIZE lines"
else
    echo "  ❌ Robot description not found"
    ((ERRORS++))
fi
echo ""

# 8. Check Node Status
echo "[8/8] Checking Required Nodes..."
REQUIRED_NODES=("robot_state_publisher" "odom_to_tf")
for node in "${REQUIRED_NODES[@]}"; do
    if ros2 node list 2>/dev/null | grep -q "$node"; then
        echo "  ✅ $node is running"
    else
        echo "  ❌ $node is NOT running"
        ((ERRORS++))
    fi
done
echo ""

# Summary
echo "=========================================="
echo "📊 SUMMARY"
echo "=========================================="
echo "Errors: $ERRORS"
echo "Warnings: $WARNINGS"
echo ""

if [ $ERRORS -eq 0 ]; then
    echo "✅ SYSTEM IS READY FOR NAV2!"
    echo ""
    echo "Next steps:"
    echo "1. Install Nav2 packages:"
    echo "   sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup"
    echo ""
    echo "2. Create Nav2 configuration files:"
    echo "   - nav2_params.yaml (navigation parameters)"
    echo "   - map.yaml (if using static map)"
    echo "   - nav2_bringup.launch.py (Nav2 launch file)"
    echo ""
    echo "3. Test with Nav2:"
    echo "   ros2 launch nav2_bringup bringup_launch.py"
    exit 0
else
    echo "❌ SYSTEM NOT READY - Fix $ERRORS error(s) first"
    exit 1
fi

