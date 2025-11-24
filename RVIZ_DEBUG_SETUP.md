# RViz Debugging Setup Guide

## 🎯 Quick RViz Setup for Movement Debugging

### Step 1: Launch RViz
```bash
cd autonomous_wheelchair
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch wheelchair_description view_robot_rviz2.launch.py
```

### Step 2: Add These Displays in RViz

1. **RobotModel**
   - Topic: `/robot_description`
   - Shows the URDF model
   - Check if robot appears correctly

2. **TF**
   - Shows transform tree
   - **CRITICAL**: Look for `odom -> chassis` transform
   - Red frames = missing transforms (BAD)
   - Green frames = valid transforms (GOOD)

3. **Odometry**
   - Topic: `/odom`
   - Shows robot position and orientation
   - Add arrow to visualize movement
   - If arrow doesn't move → robot not moving

4. **LaserScan** (optional)
   - Topic: `/scan`
   - Verify LiDAR is working

5. **Marker** (for debugging)
   - Can add custom markers to visualize issues

### Step 3: Check TF Tree

**Expected TF Tree:**
```
odom
 └── chassis
     ├── base_uplayer_link
     │   ├── wheel1_link (back right)
     │   ├── wheel2_link (back left)
     │   ├── wheel3_link (front right)
     │   └── wheel4_link (front left)
     ├── imu_sensor_link
     ├── camera_stand_link
     │   └── camera_link
     │       └── kinect_optical
     ├── base_laser
     └── seat_link
         └── back_link
```

**If `odom -> chassis` is RED:**
- Diff drive plugin not publishing odometry
- Plugin might not be loaded
- Check Gazebo console for errors

### Step 4: Monitor Topics in RViz

1. **Check `/cmd_vel` topic:**
   - Add "Topic" display or use `ros2 topic echo /cmd_vel`
   - Verify commands are being sent

2. **Check `/odom` topic:**
   - Add Odometry display
   - Watch position.x, position.y values
   - If they don't change → robot not moving

3. **Check `/joint_states` (if available):**
   - Shows joint velocities
   - wheel1_joint, wheel2_joint should have non-zero velocity when moving

## 🔍 Debugging Checklist in RViz

- [ ] Robot model appears correctly
- [ ] TF tree shows all frames (no red frames)
- [ ] `odom -> chassis` transform exists
- [ ] Odometry position changes when sending cmd_vel
- [ ] `/cmd_vel` topic shows messages being published
- [ ] Robot model moves in RViz (if TF is correct)

## 🐛 Common Issues

### Issue 1: No `odom` frame in TF
**Problem:** Diff drive plugin not publishing odometry
**Solution:** Check Gazebo console for plugin errors

### Issue 2: `odom` exists but position doesn't change
**Problem:** Plugin receiving commands but robot not moving
**Possible causes:**
- Joint names don't match
- Robot stuck/colliding
- Plugin parameters wrong

### Issue 3: Robot model doesn't appear
**Problem:** robot_description not publishing
**Solution:** Check robot_state_publisher is running

## 📊 What to Look For

1. **TF Tree:**
   - All frames should be green
   - `odom` should be at the root
   - `chassis` should be child of `odom`

2. **Odometry Display:**
   - Arrow should point in direction of movement
   - Position should change when robot moves
   - If static → robot not moving

3. **RobotModel:**
   - Should match Gazebo view
   - If different → URDF issue

