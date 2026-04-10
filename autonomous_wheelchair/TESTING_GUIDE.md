# Testing Guide - Autonomous Wheelchair

## Quick Test Checklist

### 1. Build and Source
```bash
cd autonomous_wheelchair
colcon build --symlink-install
source install/setup.bash
```

### 2. Launch Gazebo Simulation
```bash
ros2 launch wheelchair_gazebo warehouse_with_robot.launch.py
```

**What to check:**
- ✅ Gazebo window opens
- ✅ Warehouse world loads
- ✅ Wheelchair robot spawns (pink/orange body, 4 wheels)
- ✅ Robot sits on ground (wheels touching floor)
- ✅ No error messages in terminal

**Wait ~10-15 seconds** for Gazebo to fully initialize.

### 3. Launch RViz (in new terminal)
```bash
cd autonomous_wheelchair
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch wheelchair_description view_robot_rviz2.launch.py
```

**What to check:**
- ✅ RViz window opens
- ✅ RobotModel shows **"Status: OK"** (not "Error")
- ✅ All wheel links show **"Transform OK"**:
  - `wheel1_link`, `wheel2_link` (back wheels)
  - `wheel3_caster_link`, `wheel3_link` (front right)
  - `wheel4_caster_link`, `wheel4_link` (front left)
- ✅ Robot model visible in 3D view
- ✅ All 4 wheels visible (not floating)

### 4. Test TF Tree
```bash
# In a new terminal
source /opt/ros/humble/setup.bash
source install/setup.bash

# Check TF tree
ros2 run tf2_tools view_frames
# Opens frames.pdf - should show complete tree

# Check specific transforms
ros2 run tf2_ros tf2_echo base_uplayer_link wheel1_link
# Should show transform (not "ERROR")

ros2 run tf2_ros tf2_echo odom base_link
# Should show odometry transform
```

### 5. Test Topics
```bash
# List all topics
ros2 topic list

# Should see:
# ✅ /cmd_vel
# ✅ /joint_states
# ✅ /odom
# ✅ /scan (lidar)
# ✅ /imu_data
# ✅ /camera/image
# ✅ /camera/depth_image
# ✅ /tf
# ✅ /tf_static
```

### 6. Test Sensors

#### Lidar Scan
```bash
ros2 topic echo /scan --once
# Should show LaserScan message with ranges array
```

**In RViz:**
- Add "LaserScan" display
- Topic: `/scan`
- Should see red/yellow point cloud around robot

#### Camera
```bash
ros2 topic echo /camera/image --once
# Should show Image message
```

**In RViz:**
- Add "Image" display
- Topic: `/camera/image`
- Should see camera feed

#### IMU
```bash
ros2 topic echo /imu_data --once
# Should show Imu message with orientation, angular_velocity, linear_acceleration
```

### 7. Test Robot Movement

#### Keyboard Control (in new terminal)
```bash
cd autonomous_wheelchair
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**Controls:**
- `i` - Move forward
- `k` - Move backward
- `j` - Turn left
- `l` - Turn right
- `,` - Increase speed
- `.` - Decrease speed
- `q` - Quit

**What to check:**
- ✅ Robot moves in Gazebo
- ✅ Back wheels spin
- ✅ Front wheels swivel (caster yaw) and roll
- ✅ Odometry updates: `ros2 topic echo /odom`
- ✅ Robot moves in RViz (if running)

#### Test with Direct Command
```bash
# Move forward
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" \
  --once

# Turn in place
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}" \
  --once
```

### 8. Test Joint States
```bash
ros2 topic echo /joint_states
# Should show all joints:
# - wheel1_joint, wheel2_joint (back wheels)
# - wheel3_caster_yaw_joint, wheel3_joint (front right)
# - wheel4_caster_yaw_joint, wheel4_joint (front left)
```

### 9. Test Odometry
```bash
ros2 topic echo /odom
# Should show:
# - pose (position and orientation)
# - twist (linear and angular velocity)
# - frame_id: "odom"
# - child_frame_id: "base_uplayer_link"
```

## Common Issues and Fixes

### ❌ Wheels show "No transform" in RViz
**Fix:** Rebuild and restart Gazebo:
```bash
colcon build --symlink-install
source install/setup.bash
# Kill and restart Gazebo
```

### ❌ Robot doesn't move
**Check:**
- Is `/cmd_vel` being published? `ros2 topic hz /cmd_vel`
- Are back wheels spinning in Gazebo?
- Check diff drive plugin is active

### ❌ No lidar scan
**Check:**
- Is `/scan` topic published? `ros2 topic list | grep scan`
- Check frame_id in scan: `ros2 topic echo /scan --once | grep frame_id`
- Should be `autonomous_wheelchair/chassis/lidar`

### ❌ TF errors
**Check:**
- Is `robot_state_publisher` running? `ros2 node list | grep robot_state`
- Is `/joint_states` being published? `ros2 topic hz /joint_states`
- Check TF tree: `ros2 run tf2_tools view_frames`

## Success Criteria

✅ **All systems working if:**
1. Gazebo launches without errors
2. Robot spawns correctly with all 4 wheels visible
3. RViz shows robot model with "Status: OK"
4. All wheel transforms are "OK" (no blue errors)
5. Lidar scan visible in RViz
6. Camera feed visible in RViz
7. Robot moves when commanded
8. Odometry updates during movement
9. All topics publishing at expected rates

## Performance Checks

```bash
# Check topic rates
ros2 topic hz /scan        # Should be ~5 Hz
ros2 topic hz /odom        # Should be ~5 Hz
ros2 topic hz /joint_states # Should be ~30-50 Hz
ros2 topic hz /imu_data    # Should be ~30 Hz
ros2 topic hz /camera/image # Should be ~10 Hz
```

---

**Quick Test Script:**
```bash
#!/bin/bash
echo "Testing Autonomous Wheelchair..."
echo "1. Checking topics..."
ros2 topic list | grep -E "(cmd_vel|odom|scan|joint_states|imu|camera)" && echo "✅ Topics OK" || echo "❌ Missing topics"

echo "2. Checking TF..."
ros2 run tf2_ros tf2_echo base_uplayer_link wheel1_link 2>&1 | head -5 && echo "✅ TF OK" || echo "❌ TF Error"

echo "3. Testing movement..."
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}, angular: {z: 0.0}}" --once && echo "✅ Command sent"

echo "Test complete!"
```

