# Nav2 Setup Guide - Step by Step

This guide walks you through setting up and testing Nav2 with the autonomous wheelchair.

## Prerequisites

✅ **Step 1: Verify Nav2 Installation**

```bash
# Check if Nav2 is installed
ros2 pkg list | grep nav2

# If not installed, install it:
sudo apt install -y \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox
```

✅ **Step 2: Verify System Readiness**

```bash
cd autonomous_wheelchair
source /opt/ros/humble/setup.bash
source install/setup.bash
./check_nav2_readiness.sh
```

All checks should pass (0 errors).

## Step-by-Step Nav2 Setup

### Step 1: Launch Gazebo with Robot

**Terminal 1:**
```bash
cd autonomous_wheelchair
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch wheelchair_gazebo warehouse_with_robot.launch.py
```

Wait ~15-20 seconds for Gazebo to fully initialize.

**Verify:**
- Robot appears in Gazebo
- Check topics: `ros2 topic list | grep -E "(odom|scan|cmd_vel)"`
- Check TF: `ros2 run tf2_ros tf2_echo odom base_link`

### Step 2: Launch Nav2 (Without Map - Localization Only)

**Terminal 2:**
```bash
cd autonomous_wheelchair
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch wheelchair_gazebo nav2_bringup.launch.py
```

**Verify Nav2 is running:**
```bash
# Check Nav2 nodes
ros2 node list | grep nav2

# Check Nav2 topics
ros2 topic list | grep nav2

# Check if map frame exists
ros2 run tf2_ros tf2_echo map odom
```

### Step 3: Launch RViz for Visualization

**Terminal 3:**
```bash
cd autonomous_wheelchair
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run rviz2 rviz2
```

**In RViz:**
1. Set **Fixed Frame** to `map`
2. Add **RobotModel** display
3. Add **Map** display (if using a map)
4. Add **LaserScan** display (topic: `/scan`)
5. Add **TF** display
6. Add **Path** display (topic: `/plan`)
7. Add **PoseArray** display (topic: `/particlecloud`) - for AMCL

### Step 4: Set Initial Pose (Required for Localization)

**In RViz:**
1. Click **"2D Pose Estimate"** button (toolbar)
2. Click on the map where the robot is located
3. Drag to set the robot's orientation

**Or via command line:**
```bash
ros2 topic pub /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "
{
  header: {frame_id: 'map'},
  pose: {
    pose: {
      position: {x: 0.0, y: 0.0, z: 0.0},
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
    }
  }
}" --once
```

### Step 5: Send Navigation Goal

**In RViz:**
1. Click **"2D Nav Goal"** button (toolbar)
2. Click on the map where you want the robot to go
3. Drag to set the goal orientation

**Or via command line:**
```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "
{
  pose: {
    header: {frame_id: 'map'},
    pose: {
      position: {x: 2.0, y: 1.0, z: 0.0},
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
    }
  }
}"
```

## Alternative: Combined Launch (Gazebo + Nav2)

**Single Terminal:**
```bash
cd autonomous_wheelchair
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch wheelchair_gazebo warehouse_with_nav2.launch.py
```

This launches both Gazebo and Nav2 together.

## Creating a Map (Optional - For SLAM)

### Step 1: Launch SLAM Toolbox

**Terminal 4:**
```bash
cd autonomous_wheelchair
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch slam_toolbox online_async_launch.py \
  slam_params_file:=$(ros2 pkg prefix wheelchair_gazebo)/share/wheelchair_gazebo/config/slam_params.yaml
```

### Step 2: Drive Robot to Map Environment

Use keyboard teleop to drive the robot around:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Step 3: Save the Map

```bash
ros2 run nav2_map_server map_saver_cli -f ~/wheelchair_map
```

This creates:
- `~/wheelchair_map.pgm` (map image)
- `~/wheelchair_map.yaml` (map metadata)

### Step 4: Use Saved Map with Nav2

```bash
ros2 launch wheelchair_gazebo nav2_bringup.launch.py \
  map:=~/wheelchair_map.yaml
```

## Troubleshooting

### Issue: "No transform from [map] to [odom]"

**Solution:** Set initial pose in RViz using "2D Pose Estimate" tool.

### Issue: "No path found"

**Solutions:**
- Check if goal is in an obstacle
- Verify costmap is updating: `ros2 topic echo /local_costmap/costmap`
- Check robot radius in `nav2_params.yaml` (should match wheelchair size)

### Issue: Robot not moving

**Solutions:**
- Check `/cmd_vel` topic: `ros2 topic echo /cmd_vel`
- Verify controller is running: `ros2 node list | grep controller`
- Check for errors: `ros2 topic echo /rosout | grep ERROR`

### Issue: Costmap not updating

**Solutions:**
- Verify `/scan` topic is publishing: `ros2 topic hz /scan`
- Check lidar frame in TF: `ros2 run tf2_ros tf2_echo base_link base_laser`
- Verify scan frame_id matches costmap configuration

## Configuration Files

- **Nav2 Parameters:** `src/wheelchair_gazebo/config/nav2_params.yaml`
  - Robot radius: 0.3m (wheelchair size)
  - Max velocity: 1.0 m/s
  - Max rotation: 0.5 rad/s
  - Base frame: `base_link`

## Next Steps

1. ✅ Test basic navigation without map (localization only)
2. ✅ Create map using SLAM
3. ✅ Test navigation with saved map
4. ✅ Tune parameters for better performance
5. ✅ Add waypoint following

## Useful Commands

```bash
# View TF tree
ros2 run tf2_tools view_frames

# Check Nav2 status
ros2 service call /lifecycle_manager_navigation/manage_nodes lifecycle_msgs/srv/GetState

# Monitor navigation goal
ros2 action list
ros2 action info /navigate_to_pose

# View costmap
ros2 topic echo /local_costmap/costmap --once
```

