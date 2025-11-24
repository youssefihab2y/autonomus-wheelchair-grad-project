# 🔍 Complete Debugging Guide for Gazebo and RViz

## 🎯 Quick Debugging Checklist

### Step 1: Check Topics in Terminal
```bash
# Check if cmd_vel topic exists
ros2 topic list | grep cmd_vel

# Check if bridge is forwarding messages
ros2 topic echo /cmd_vel
# In another terminal, publish a command:
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}"

# Check odometry
ros2 topic echo /odom

# Check joint states
ros2 topic echo /joint_states
```

### Step 2: Debug in Gazebo

#### A. Check Plugin Status
1. **Select Robot in Entity Tree:**
   - Right panel → Entity Tree
   - Find `autonomous_wheelchair` or `wheelchair`
   - Click to select

2. **Check Component Inspector:**
   - Right panel → Component Inspector
   - Look for `DiffDrive` plugin
   - Check if it's loaded and active
   - Look for any error messages

3. **Check Console Output:**
   - Bottom panel in Gazebo
   - Look for plugin loading messages
   - Look for errors about:
     - `DiffDrive`
     - `libignition-gazebo6-diff-drive-system.so`
     - Joint names (wheel1_joint, wheel2_joint)

#### B. Test with Gazebo Teleop Panel
1. **Open Teleop Panel:**
   - Right panel → Teleop
   - Set topic to: `/cmd_vel` or `model/wheelchair/cmd_vel`
   - Try both if one doesn't work

2. **Test Movement:**
   - Use arrow buttons to move
   - If Gazebo teleop works → Plugin is fine, bridge issue
   - If Gazebo teleop doesn't work → Plugin issue

#### C. Check Robot Position
1. **Select Robot:**
   - Click robot in 3D view
   - Check position in Component Inspector
   - Verify robot is on ground (z should be ~0.1-0.2, not 2.0)

2. **Check Joint States:**
   - In Component Inspector, look for joint components
   - Check wheel1_joint, wheel2_joint
   - Verify they exist and are active

#### D. Check Console for Errors
Look for these error messages:
- `Plugin not found` → Plugin filename wrong
- `Joint not found` → Joint names don't match
- `Topic not found` → Bridge not working
- `Failed to load` → Plugin installation issue

### Step 3: Debug in RViz

#### A. Check TF Tree
1. **Add TF Display:**
   - Left panel → Add → TF
   - Check all frames

2. **Check Wheel Transforms:**
   - Look for wheel1_link, wheel2_link, wheel3_link, wheel4_link
   - Should show "Transform OK" (green)
   - If "No transform" (red) → joint_state_publisher issue

3. **Check Fixed Frame:**
   - Global Options → Fixed Frame
   - Set to `chassis` or `odom`
   - If `odom` doesn't exist → plugin not publishing odometry

#### B. Check Robot Model
1. **Add RobotModel Display:**
   - Left panel → Add → RobotModel
   - Topic: `/robot_description`
   - Robot should appear complete

2. **Check for Missing Parts:**
   - All wheels visible?
   - Camera stand touching base?
   - All sensors visible?

#### C. Check Odometry
1. **Add Odometry Display:**
   - Left panel → Add → Odometry
   - Topic: `/odom`
   - Should show arrow indicating position
   - If no data → plugin not publishing or bridge issue

#### D. Check ROS Time
- Bottom panel → Time
- ROS Time should NOT be 0.00
- If 0.00 → `/clock` bridge not working

### Step 4: Check Bridge Configuration

#### A. Verify Bridge is Running
```bash
ros2 node list | grep bridge
ros2 node info /gz_bridge
```

#### B. Check Bridge Topics
```bash
# Check what topics bridge is bridging
ros2 topic list | grep -E "cmd_vel|odom|joint_states|clock"
```

#### C. Test Bridge Bidirectionally
```bash
# Test ROS -> Gazebo
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}"

# Check if message reached Gazebo
# In Gazebo, check if robot moves or check topic:
gz topic -e -t /model/wheelchair/cmd_vel
```

### Step 5: Check Plugin Configuration

#### A. Verify Plugin File Exists
```bash
ls -la /usr/lib/x86_64-linux-gnu/ign-gazebo-6/plugins/ | grep diff
# Should see: libignition-gazebo6-diff-drive-system.so
```

#### B. Check Plugin Parameters in URDF
```bash
grep -A 10 "DiffDrive" autonomous_wheelchair/src/wheelchair_description/urdf/wheelchair.gazebo.xacro
```

Verify:
- `left_joint`: wheel2_joint
- `right_joint`: wheel1_joint
- `wheel_separation`: 0.52
- `wheel_radius`: 0.15
- `odom_frame_id`: odom
- `robot_base_frame`: chassis

#### C. Check Joint Names Match
```bash
# In URDF
grep "wheel.*_joint" autonomous_wheelchair/src/wheelchair_description/urdf/wheelchair.urdf.xacro

# In Plugin
grep "left_joint\|right_joint" autonomous_wheelchair/src/wheelchair_description/urdf/wheelchair.gazebo.xacro
```

## 🐛 Common Issues and Solutions

### Issue 1: Robot Doesn't Move
**Symptoms:**
- Commands sent but robot doesn't move
- No errors in console

**Debug Steps:**
1. Check Gazebo Teleop panel (test directly)
2. Check plugin is loaded (Component Inspector)
3. Check joint names match (URDF vs Plugin)
4. Check robot position (not floating)
5. Check wheel transforms in RViz

### Issue 2: "No transform" in RViz
**Symptoms:**
- Wheels show "No transform from [wheelX_link] to [chassis]"

**Debug Steps:**
1. Check `/joint_states` topic exists
2. Check `joint_state_publisher` is running
3. Check `robot_state_publisher` is running
4. Restart Gazebo with new launch file

### Issue 3: ROS Time = 0.00
**Symptoms:**
- RViz shows ROS Time: 0.00
- TF not publishing

**Debug Steps:**
1. Check `/clock` topic exists
2. Check bridge is running
3. Check `use_sim_time: true` in launch file

### Issue 4: No Odometry
**Symptoms:**
- `/odom` topic exists but no data
- No `odom` frame in TF tree

**Debug Steps:**
1. Check plugin is loaded
2. Check `odom_frame_id` parameter
3. Check bridge configuration
4. Check Gazebo console for plugin errors

## 📊 What to Report Back

When debugging, report:

1. **Gazebo:**
   - [ ] Plugin loaded? (Component Inspector)
   - [ ] Any errors in console?
   - [ ] Gazebo Teleop works?
   - [ ] Robot position (z coordinate)?

2. **RViz:**
   - [ ] Wheel transforms OK?
   - [ ] ROS Time advancing?
   - [ ] Odometry data?
   - [ ] Robot model complete?

3. **Topics:**
   - [ ] `/cmd_vel` exists?
   - [ ] `/odom` exists and has data?
   - [ ] `/joint_states` exists and has data?
   - [ ] `/clock` exists?

4. **Bridge:**
   - [ ] Bridge node running?
   - [ ] Bridge topics correct?

5. **Plugin:**
   - [ ] Plugin file exists?
   - [ ] Joint names match?
   - [ ] Parameters correct?

## 🔧 Quick Fixes

### Restart Everything
```bash
# Stop all processes
pkill -f gazebo
pkill -f ros2

# Rebuild
cd autonomous_wheelchair
colcon build

# Restart
source install/setup.bash
ros2 launch wheelchair_gazebo warehouse_with_robot.launch.py
```

### Check Plugin Filename
```bash
# Verify plugin exists
ls /usr/lib/x86_64-linux-gnu/ign-gazebo-6/plugins/libignition-gazebo6-diff-drive-system.so

# Check URDF uses correct name
grep "libignition-gazebo.*diff-drive" autonomous_wheelchair/src/wheelchair_description/urdf/wheelchair.gazebo.xacro
```

### Test Bridge Manually
```bash
# Publish to ROS topic
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}"

# Check Gazebo topic
gz topic -e -t /model/wheelchair/cmd_vel
```

