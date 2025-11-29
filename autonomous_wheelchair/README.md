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
   
   **With Nav2 navigation (full navigation stack):**
   ```bash
   ros2 launch wheelchair_gazebo warehouse_with_nav2.launch.py \
     map:=~/wheelchair_warehouse_map.yaml
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
# Launch Gazebo + Robot + Nav2 
ros2 launch wheelchair_gazebo warehouse_with_nav2.launch.py \
  map:=~/wheelchair_warehouse_map.yaml

# Or launch Nav2 separately (if Gazebo + Robot are already running):
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

# Launch with Nav2
ros2 launch wheelchair_gazebo warehouse_with_nav2.launch.py map:=~/wheelchair_warehouse_map.yaml
```

See [NAV2_SETUP_GUIDE.md](./NAV2_SETUP_GUIDE.md) for complete Nav2 setup guide.

## 🔧 Troubleshooting

See [QUICK_START.md](./QUICK_START.md) for detailed troubleshooting steps, including RViz/Gazebo debugging tips.

## 📝 Notes

- Build directories (`build/`, `install/`, `log/`) are automatically generated and gitignored
- Wait ~15-20 seconds after launch for Gazebo to fully initialize
- Keyboard teleop must run in a separate terminal (requires interactive terminal)
- **Lidar** is mounted on the back bar to avoid self-detection issues
- **Front wheels** use optimized caster mechanism with low friction for smooth turning
- **Recommended speeds** for SLAM: Forward 0.3-0.5 m/s, Yaw 0.3-0.5 rad/s
- **Camera** starts positioned behind the wheelchair for better view
- User-generated maps are stored in `maps/` directory (gitignored)

---

**ROS 2 Version:** Humble
**Gazebo Version:** Fortress (6.17.0)
**License:** MIT

