# SLAM Mapping Guide

This guide walks you through creating a map of the warehouse environment using SLAM (Simultaneous Localization and Mapping).

## Quick Start

### Option 1: Combined Launch (Easiest)

**Single Terminal:**
```bash
cd autonomous_wheelchair
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch wheelchair_gazebo warehouse_with_slam.launch.py
```

This launches Gazebo + Robot + SLAM together.

### Option 2: Separate Launches

**Terminal 1: Launch Gazebo + Robot**
```bash
cd autonomous_wheelchair
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch wheelchair_gazebo warehouse_with_robot.launch.py
```

**Terminal 2: Launch SLAM**
```bash
cd autonomous_wheelchair
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch wheelchair_gazebo slam_mapping.launch.py
```

**Terminal 3: Launch RViz (for visualization)**
```bash
cd autonomous_wheelchair
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run rviz2 rviz2
```

## Step-by-Step Mapping Process

### Step 1: Launch Everything

Use Option 1 or Option 2 above to start Gazebo, robot, and SLAM.

### Step 2: Configure RViz

In RViz:
1. Set **Fixed Frame** to `map`
2. Add **Map** display:
   - Topic: `/map`
   - Set **Durability** to `Transient Local` (important!)
3. Add **RobotModel** display
4. Add **LaserScan** display (topic: `/scan`)
5. Add **TF** display
6. Add **PoseArray** display (topic: `/slam_toolbox/particlecloud`) - shows SLAM particles

### Step 3: Drive the Robot

**Terminal 4: Keyboard Control**
```bash
cd autonomous_wheelchair
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**Drive the robot around:**
- Use `i` to move forward
- Use `j`/`l` to turn left/right
- Use `k` to move backward
- Drive slowly and cover the entire warehouse
- Make sure to loop back to close loops (helps SLAM accuracy)

### Step 4: Monitor Mapping Progress

**Watch the map build in RViz:**
- The map will appear as you drive
- Gray areas = unknown
- White areas = free space
- Black areas = obstacles

**Check SLAM status:**
```bash
# View SLAM topics
ros2 topic list | grep slam

# Check map topic
ros2 topic hz /map

# View map info
ros2 topic echo /map_metadata --once
```

### Step 5: Save the Map

Once you've mapped the entire warehouse:

**Option A: Using map_saver (Recommended)**
```bash
cd autonomous_wheelchair
source /opt/ros/humble/setup.bash
source install/setup.bash

# Save map to home directory
ros2 run nav2_map_server map_saver_cli -f ~/wheelchair_warehouse_map

# Or save to a specific location
ros2 run nav2_map_server map_saver_cli -f $(pwd)/maps/warehouse_map
```

This creates:
- `~/wheelchair_warehouse_map.pgm` (map image)
- `~/wheelchair_warehouse_map.yaml` (map metadata)

**Option B: Using SLAM Toolbox service**
```bash
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: 'wheelchair_warehouse_map'}}"
```

### Step 6: Use the Saved Map with Nav2

After saving the map, you can use it with Nav2:

```bash
ros2 launch wheelchair_gazebo nav2_bringup.launch.py \
  map:=~/wheelchair_warehouse_map.yaml
```

## Tips for Better Mapping

1. **Drive Slowly**: Move at moderate speed (0.3-0.5 m/s) for better scan matching
2. **Cover All Areas**: Make sure to visit all corners and rooms
3. **Close Loops**: Return to previously visited areas to improve map accuracy
4. **Avoid Fast Turns**: Smooth movements help SLAM match scans better
5. **Check Map Quality**: In RViz, verify walls are straight and obstacles are clear

## Troubleshooting

### Issue: Map not appearing in RViz

**Solution:**
- Check Map display durability is set to "Transient Local"
- Verify `/map` topic is publishing: `ros2 topic hz /map`
- Check SLAM node is running: `ros2 node list | grep slam`

### Issue: Map looks distorted

**Solutions:**
- Drive slower
- Make sure odometry is working: `ros2 topic echo /odom`
- Check TF tree: `ros2 run tf2_tools view_frames`
- Verify scan is publishing: `ros2 topic hz /scan`

### Issue: SLAM not updating map

**Solutions:**
- Check minimum travel distance in `slam_params.yaml` (currently 0.5m)
- Verify robot is actually moving: `ros2 topic echo /odom`
- Check for SLAM errors: `ros2 topic echo /rosout | grep ERROR`

## Configuration Files

- **SLAM Parameters:** `src/wheelchair_gazebo/config/slam_params.yaml`
  - Resolution: 0.05m (5cm)
  - Max laser range: 10.0m
  - Base frame: `base_link`
  - Scan topic: `/scan`

## Next Steps After Mapping

1. ✅ Save the map
2. ✅ Test localization with saved map
3. ✅ Use map with Nav2 for navigation
4. ✅ Fine-tune SLAM parameters if needed

## Useful Commands

```bash
# View map metadata
ros2 topic echo /map_metadata --once

# Check map topic
ros2 topic hz /map

# View SLAM status
ros2 service list | grep slam

# Save map interactively
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: 'my_map'}}"

# View TF tree
ros2 run tf2_tools view_frames
```

