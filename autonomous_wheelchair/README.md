# Autonomous Wheelchair Project

ROS 2 Humble + Gazebo Fortress project for autonomous wheelchair simulation with obstacle avoidance and navigation capabilities.

> ℹ️ **Latest updates:**
> - **SLAM Integration** - Full SLAM toolbox support for map creation
> - **Nav2 Integration** - Complete navigation stack with wheelchair-specific parameters
> - **Lidar on back bar** - Mounted for optimal 360° scanning without self-detection
> - **Optimized turning** - Low-friction front wheels with caster mechanism for smooth rotation
> - **Complete TF tree** - Proper odometry integration (`odom → base_link → chassis`)
> - **Frame ID fixes** - Automatic conversion of Gazebo prefixed frames to URDF frames

## 📋 Requirements

- **Ubuntu 22.04 LTS**
- **ROS 2 Humble Desktop**
- **Gazebo Fortress** (Gazebo Sim 6.17.0)
- **Python 3.10+**

### Required ROS 2 Packages
```bash
sudo apt update
sudo apt install -y \
  ros-humble-desktop \
  ros-humble-ros-gz-sim \
  ros-humble-ros-gz-bridge \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  ros-humble-teleop-twist-keyboard \
  ros-humble-rviz2 \
  ros-humble-tf2-ros \
  ros-humble-tf2-tools \
  ros-humble-nav-msgs \
  ros-humble-sensor-msgs \
  ros-humble-geometry-msgs \
  python3-colcon-common-extensions
```

### Optional: Navigation and SLAM Packages
```bash
# For Nav2 navigation
sudo apt install -y \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup

# For SLAM mapping
sudo apt install -y \
  ros-humble-slam-toolbox \
  ros-humble-nav2-map-server
```

## 🚀 Quick Start

1. **Source ROS 2:**
   ```bash
   source /opt/ros/humble/setup.bash
   ```

2. **Build the workspace:**
   ```bash
   cd autonomous_wheelchair
   colcon build --symlink-install
   source install/setup.bash
   ```

3. **Launch simulation (choose one):**
   
   **Basic simulation:**
   ```bash
   ros2 launch wheelchair_gazebo warehouse_with_robot.launch.py
   ```
   
   **With SLAM mapping (build a new map):**
   ```bash
   ros2 launch wheelchair_gazebo warehouse_with_slam_rviz.launch.py
   ```
   
   **With localization using a saved map (AMCL only):**
   ```bash
   # Example using the example map committed in this repo
   ros2 launch wheelchair_gazebo warehouse_with_amcl_rviz.launch.py \
     map_file:=$(pwd)/src/wheelchair_gazebo/maps/mymap.yaml
   ```
   
   **With Nav2 navigation (full navigation stack + Gazebo + RViz):**
   ```bash
   # Single command launches everything: Gazebo + Robot + Nav2 + RViz
   # RViz opens automatically after ~15 seconds
   ros2 launch wheelchair_gazebo warehouse_with_nav2.launch.py \
     map:=$(pwd)/src/wheelchair_gazebo/maps/mymap.yaml
   
   # Or use your own saved map:
   ros2 launch wheelchair_gazebo warehouse_with_nav2.launch.py \
     map:=~/wheelchair_warehouse_map.yaml
   ```
   
   **What opens automatically:**
   - **Gazebo** - Simulation window (appears first, ~5 seconds)
   - **RViz** - Visualization window (appears after ~15 seconds with Nav2 default view)
   - **Nav2 nodes** - All navigation components start automatically
   
   **Note:** If RViz doesn't appear, you can launch it manually:
   ```bash
   ros2 run rviz2 rviz2 -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/rviz/nav2_default_view.rviz
   ```

4. **(Optional) Clear stale processes before relaunching:**
   ```bash
   ./STOP_ALL.sh
   ```

5. **Control the robot (in a new terminal):**
   ```bash
   source /opt/ros/humble/setup.bash
   source install/setup.bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```

6. **Smoke-test movement (publishes one forward command):**
   ```bash
   ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
     "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" \
     --once
   ```
   You should immediately see the wheelchair roll forward in Gazebo and `/odom` update.

For detailed instructions, see [QUICK_START.md](./QUICK_START.md).

## 📁 Project Structure

```
autonomous_wheelchair/
├── src/
│   ├── wheelchair_description/    # Robot URDF/Xacro model and configs
│   │   ├── urdf/                   # Robot model files
│   │   ├── launch/                 # Launch files for visualization
│   │   ├── config/                 # Controller configurations
│   │   └── rviz/                   # RViz configuration files
│   └── wheelchair_gazebo/         # Gazebo simulation
│       ├── launch/                 # Launch files (simulation, SLAM, Nav2, AMCL)
│       ├── config/                 # Configuration files (Nav2, SLAM)
│       ├── maps/                   # Example maps shipped with the project (e.g. mymap.{pgm,yaml})
│       ├── scripts/                # Python nodes (TF, odometry, lidar filtering)
│       └── worlds/                 # Gazebo world files
├── build/                          # Build files (generated, gitignored)
├── install/                        # Install files (generated, gitignored)
├── log/                            # Log files (generated, gitignored)
├── maps/                           # User-generated maps you save (gitignored)
└── Documentation files:
    ├── README.md                   # This file
    ├── QUICK_START.md              # Quick start guide
    ├── NAV2_READINESS.md           # Nav2 setup checklist
    ├── NAV2_SETUP_GUIDE.md         # Nav2 integration guide
    ├── SLAM_GUIDE.md               # SLAM mapping guide
    ├── DRIVING_GUIDE.md            # Driving instructions
    └── SPEED_SETTINGS.md           # Speed reference guide
   ```

## ✨ Features

- ✅ **4-direction movement control** - Forward, backward, left, right
- ✅ **Gazebo simulation** - Warehouse environment with obstacles
- ✅ **RViz visualization** - Real-time robot state visualization
- ✅ **Keyboard teleoperation** - Control robot with keyboard
- ✅ **Sensor integration** - LIDAR (mounted on back bar) and depth camera support
- ✅ **Odometry** - Real-time position and velocity tracking with proper TF transforms
- ✅ **Caster wheels** - Front wheels with optimized friction for smooth turning
- ✅ **SLAM mapping** - Create maps of the environment using SLAM toolbox
- ✅ **Nav2 navigation** - Complete navigation stack with wheelchair-specific parameters
- ✅ **Self-filtering lidar** - Automatic filtering of robot's own body from scans
- ✅ **Frame ID conversion** - Automatic conversion of Gazebo frames to URDF frames

## 📦 Packages

- **`wheelchair_description`** - Robot URDF/Xacro model, launch files, and configurations
- **`wheelchair_gazebo`** - Gazebo simulation launch files and world files

## 🎮 Usage

### Launch Options

**1. Basic Simulation:**
```bash
# Terminal 1: Launch Gazebo + Robot 
ros2 launch wheelchair_gazebo warehouse_with_robot.launch.py


# Terminal 2: Launch RViz (optional)
ros2 launch wheelchair_description view_robot_rviz2.launch.py

# Terminal 3: Keyboard control
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**2. SLAM Mapping (Recommended for first-time setup):**
```bash
# Single command launches everything (Gazebo + Robot + SLAM + RViz)
ros2 launch wheelchair_gazebo warehouse_with_slam_rviz.launch.py

# Then in new terminal: Drive to create map
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Save map when done (for example into your home directory):
ros2 run nav2_map_server map_saver_cli -f ~/wheelchair_warehouse_map
```

**3. Nav2 Navigation (using a saved map):**
```bash
# Single command: Launch Gazebo + Robot + Nav2 + RViz (all-in-one)
# RViz opens automatically after ~15 seconds
ros2 launch wheelchair_gazebo warehouse_with_nav2.launch.py \
  map:=$(pwd)/src/wheelchair_gazebo/maps/mymap.yaml

# Or use your own saved map:
ros2 launch wheelchair_gazebo warehouse_with_nav2.launch.py \
  map:=~/wheelchair_warehouse_map.yaml

# Or launch Nav2 separately (if Gazebo + Robot are already running):
# Note: This does NOT include RViz - launch it manually if needed
ros2 launch wheelchair_gazebo nav2_bringup.launch.py \
  map:=~/wheelchair_warehouse_map.yaml
```

See [SLAM_GUIDE.md](./SLAM_GUIDE.md) and [NAV2_SETUP_GUIDE.md](./NAV2_SETUP_GUIDE.md) for detailed instructions.

### Keyboard Controls
- **i**: Move forward
- **k**: Move backward
- **j**: Turn left
- **l**: Turn right
- **,**: Increase linear speed
- **.**: Decrease linear speed
- **u**: Increase angular speed
- **o**: Decrease angular speed
- **Space**: Emergency stop
- **q**: Quit

## 🛠️ Helper Scripts

- `STOP_ALL.sh` – Terminates lingering Gazebo/ROS 2 processes before a fresh launch
- `START_KEYBOARD_CONTROL.sh` – Sources the workspace and starts `teleop_twist_keyboard`
- `check_nav2_readiness.sh` – Automated script to verify system is ready for Nav2 integration

## 📚 Documentation

- **[QUICK_START.md](./QUICK_START.md)** - Quick start guide and troubleshooting
- **[NAV2_READINESS.md](./NAV2_READINESS.md)** - Nav2 readiness checklist
- **[NAV2_SETUP_GUIDE.md](./NAV2_SETUP_GUIDE.md)** - Step-by-step Nav2 integration guide
- **[SLAM_GUIDE.md](./SLAM_GUIDE.md)** - SLAM mapping instructions
- **[DRIVING_GUIDE.md](./DRIVING_GUIDE.md)** - Driving techniques for mapping
- **[SPEED_SETTINGS.md](./SPEED_SETTINGS.md)** - Speed parameter reference

## 🧭 Navigation & Mapping

### SLAM Mapping
Create maps of your environment using SLAM toolbox:
```bash
# Launch with SLAM
ros2 launch wheelchair_gazebo warehouse_with_slam_rviz.launch.py

# Drive around to map the environment
# Save map when complete:
ros2 run nav2_map_server map_saver_cli -f ~/wheelchair_warehouse_map
```

See [SLAM_GUIDE.md](./SLAM_GUIDE.md) for detailed mapping instructions.

### Nav2 Navigation
Navigate autonomously using saved maps:
```bash
# Check system readiness
./check_nav2_readiness.sh

# Launch with Nav2 (includes Gazebo + Robot + Nav2 + RViz)
# RViz opens automatically after ~15 seconds
ros2 launch wheelchair_gazebo warehouse_with_nav2.launch.py \
  map:=$(pwd)/src/wheelchair_gazebo/maps/mymap.yaml
```

**What to expect:**
- **Gazebo window** opens first (~5 seconds) - shows the robot in the warehouse
- **RViz window** opens automatically (~15 seconds) - pre-configured with Nav2 default view
- **Nav2 nodes** start automatically - ready for navigation

**After launching:**
1. **Wait for RViz to open** (should appear automatically after ~15 seconds)
2. **Set initial pose in RViz**: Use the "2D Pose Estimate" tool (top toolbar) to set the robot's starting position on the map
3. **Send navigation goal**: Use the "2D Nav Goal" tool (top toolbar) to command the robot to navigate to a location
4. **Monitor in RViz**: Watch the global/local costmaps, planned path, and robot movement

**If RViz doesn't open automatically:**
```bash
# Launch RViz manually with Nav2 configuration
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/rviz/nav2_default_view.rviz
```

See [NAV2_SETUP_GUIDE.md](./NAV2_SETUP_GUIDE.md) for complete Nav2 setup guide.

## 🔧 Troubleshooting

See [QUICK_START.md](./QUICK_START.md) for detailed troubleshooting steps, including RViz/Gazebo debugging tips.

## 🔧 Nav2 Parameter Customization Guide

> **For Nav2 developers/partners**: This section explains how to modify Nav2 parameters for enhancements and tuning.

### Configuration File Location
All Nav2 parameters are stored in:
```
src/wheelchair_gazebo/config/nav2_params.yaml
```

### Key Parameter Sections

#### 1. **AMCL (Localization) Parameters**
Located under `amcl:` section:
- `min_particles: 500` / `max_particles: 2000` - Particle filter size (more = more accurate but slower)
- `laser_model_type: "likelihood_field"` - Laser scan matching model
- `update_min_d: 0.25` - Minimum distance (m) before updating localization
- `update_min_a: 0.2` - Minimum angle (rad) before updating localization

**To improve localization accuracy:**
- Increase `max_particles` (e.g., 3000-5000) for better accuracy in complex environments
- Adjust `alpha1-alpha5` (0.2 default) for odometry noise models
- Tune `laser_max_range: 100.0` to match your lidar's actual range

#### 2. **DWB Local Planner (Controller) Parameters**
Located under `controller_server:` → `FollowPath:` section:
- `max_vel_x: 1.0` - Maximum forward speed (m/s)
- `max_vel_theta: 0.5` - Maximum rotation speed (rad/s)
- `acc_lim_x: 0.5` - Forward acceleration limit (m/s²)
- `acc_lim_theta: 0.5` - Angular acceleration limit (rad/s²)
- `vx_samples: 20` - Number of velocity samples for forward motion
- `vtheta_samples: 40` - Number of velocity samples for rotation

**To tune navigation behavior:**
- **Faster navigation**: Increase `max_vel_x` and `max_vel_theta` (but respect wheelchair limits)
- **Smoother motion**: Increase `acc_lim_x` and `acc_lim_theta` for gentler acceleration
- **Better obstacle avoidance**: Increase `vx_samples` and `vtheta_samples` (more trajectory options)
- **Critic weights**: Adjust `critic_scales` to prioritize different behaviors (path following vs. obstacle avoidance)

#### 3. **Global Planner Parameters**
Located under `planner_server:` → `GridBased:` section:
- `tolerance: 0.5` - Goal tolerance (m)
- `use_astar: false` - Uses Dijkstra by default (set to `true` for A* algorithm)
- `allow_unknown: true` - Allow planning through unknown space

**To change planner algorithm:**
- **Dijkstra** (current): Guaranteed shortest path, slower
- **A***: Faster, heuristic-based, may not be optimal
- To switch to A*: Set `use_astar: true` in the `GridBased:` section

#### 4. **Costmap Parameters**
Located under `global_costmap:` and `local_costmap:` sections:

**Global Costmap:**
- `robot_radius: 0.4` - Robot footprint radius (m) - **IMPORTANT**: Match your wheelchair size
- `inflation_radius: 0.55` - How far obstacles are "inflated" (safety margin)
- `resolution: 0.05` - Map resolution (m/pixel) - lower = more detailed but slower

**Local Costmap:**
- `width: 3.0` / `height: 3.0` - Size of local planning window (m)
- `update_frequency: 5.0` - How often to update (Hz)

**To improve obstacle avoidance:**
- Increase `inflation_radius` for larger safety margins
- Adjust `robot_radius` to match actual wheelchair dimensions
- Increase `update_frequency` for more responsive obstacle detection

#### 5. **Layer Configuration**
Costmaps use layers for different obstacle sources:

**StaticLayer** - Loads obstacles from the map
**ObstacleLayer** - Detects obstacles from lidar scans
**InflationLayer** - Expands obstacles for safety

**To add/remove layers:**
- Modify the `plugins:` list in `global_costmap:` or `local_costmap:`
- Example: Add `RangeSensorLayer` for additional sensor data

### Making Changes

1. **Edit the YAML file:**
   ```bash
   nano src/wheelchair_gazebo/config/nav2_params.yaml
   ```

2. **Rebuild the package:**
   ```bash
   colcon build --packages-select wheelchair_gazebo
   source install/setup.bash
   ```

3. **Test your changes:**
   ```bash
   ros2 launch wheelchair_gazebo warehouse_with_nav2.launch.py \
     map:=$(pwd)/src/wheelchair_gazebo/maps/mymap.yaml
   ```

### Recommended Tuning Workflow

1. **Start with localization (AMCL)**: Ensure robot knows where it is accurately
2. **Tune global planner**: Adjust tolerance and algorithm for path planning
3. **Tune local planner (DWB)**: Adjust speeds, accelerations, and critic weights
4. **Fine-tune costmaps**: Adjust inflation, resolution, and update frequencies
5. **Test in simulation**: Use Gazebo to validate changes before real-world testing

### Important Notes for Wheelchair

- **Max speeds**: Wheelchair typically maxes at ~1.0 m/s forward, ~0.5 rad/s rotation
- **Acceleration limits**: Wheelchairs have lower acceleration than robots - keep `acc_lim_x` ≤ 0.5
- **Robot radius**: Current setting (0.4m) assumes wheelchair width ~0.8m - adjust if different
- **Turning behavior**: DWB controller is tuned for differential drive (no lateral movement)

### Additional Resources

- **Nav2 Documentation**: https://navigation.ros.org/
- **DWB Controller**: https://github.com/ros-planning/navigation2/tree/main/nav2_dwb_controller
- **Parameter Reference**: See inline comments in `nav2_params.yaml` for detailed explanations

---

## 📝 Notes

- Build directories (`build/`, `install/`, `log/`) are automatically generated and gitignored
- Wait ~15-20 seconds after launch for Gazebo to fully initialize
- Keyboard teleop must run in a separate terminal (requires interactive terminal)
- **Lidar** is mounted on the back bar to avoid self-detection issues
- **Front wheels** use optimized caster mechanism with low friction for smooth turning
- **Recommended speeds** for SLAM: Forward 0.3-0.5 m/s, Yaw 0.3-0.5 rad/s
- **Camera** starts positioned behind the wheelchair for better view
- User-generated maps are stored in `maps/` directory (gitignored)
- **Nav2 launch** now includes RViz automatically - no need to launch separately

---

**ROS 2 Version:** Humble
**Gazebo Version:** Fortress (6.17.0)
**License:** MIT

