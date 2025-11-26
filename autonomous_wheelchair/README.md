# Autonomous Wheelchair Project

ROS 2 Humble + Gazebo Fortress project for autonomous wheelchair simulation with obstacle avoidance.

> ℹ️ **Latest update:** the Gazebo launch file now spawns the wheelchair at `z=0.15 m`, so the wheels always touch the ground immediately—no extra tweaking required before testing movement. The Gazebo diff-drive plugin now uses explicit `cmd_vel` / `odom` topics, so ROS commands and odometry always line up with Gazebo.

## 📋 Requirements

- **Ubuntu 22.04 LTS**
- **ROS 2 Humble Desktop**
- **Gazebo Fortress** (Gazebo Sim 6.17.0)
- **Python 3.10+**

### Required ROS 2 Packages
```bash
sudo apt install -y \
  ros-humble-desktop \
  ros-humble-ros-gz-sim \
  ros-humble-ros-gz-bridge \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  ros-humble-teleop-twist-keyboard \
  ros-humble-rviz2 \
  python3-colcon-common-extensions
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

3. **Launch simulation:**
   ```bash
   ros2 launch wheelchair_gazebo warehouse_with_robot.launch.py
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
│       ├── launch/                 # Launch files for simulation
│       └── worlds/                  # Gazebo world files
├── build/                          # Build files (generated, gitignored)
├── install/                        # Install files (generated, gitignored)
└── log/                            # Log files (generated, gitignored)
   ```

## ✨ Features

- ✅ **4-direction movement control** - Forward, backward, left, right
- ✅ **Gazebo simulation** - Warehouse environment with obstacles
- ✅ **RViz visualization** - Real-time robot state visualization
- ✅ **Keyboard teleoperation** - Control robot with keyboard
- ✅ **Sensor integration** - LIDAR and depth camera support
- ✅ **Odometry** - Real-time position and velocity tracking

## 📦 Packages

- **`wheelchair_description`** - Robot URDF/Xacro model, launch files, and configurations
- **`wheelchair_gazebo`** - Gazebo simulation launch files and world files

## 🎮 Usage

### Launch Simulation and Visualization
```bash
# Terminal 1: Launch Gazebo
ros2 launch wheelchair_gazebo warehouse_with_robot.launch.py

# Terminal 2: Launch RViz (optional)
ros2 launch wheelchair_description view_robot_rviz2.launch.py

# Terminal 3: Keyboard control
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

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

## 🔧 Troubleshooting

See [QUICK_START.md](./QUICK_START.md) for detailed troubleshooting steps, including RViz/Gazebo debugging tips.

## 📝 Notes

- Build directories (`build/`, `install/`, `log/`) are automatically generated and gitignored
- Wait ~15-20 seconds after launch for Gazebo to fully initialize
- Keyboard teleop must run in a separate terminal (requires interactive terminal)

---

**ROS 2 Version:** Humble
**Gazebo Version:** Fortress (6.17.0)
**License:** MIT

