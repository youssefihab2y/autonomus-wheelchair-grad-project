# 🦽 Autonomous Wheelchair - Quick Start Guide

## 🚀 Quick Start

### 1. Clean Everything (Fresh Start)
```bash
cd /home/nadafouad/Downloads/Autonomous-Wheelchair-master-branch/autonomous_wheelchair
rm -rf build install log
```

### 2. Build the Workspace
```bash
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

### 3. Launch Simulation
```bash
ros2 launch wheelchair_gazebo warehouse_with_robot.launch.py
```

**Wait ~45 seconds** for everything to initialize (controllers, bridges, etc.)

### 4. Launch Keyboard Control (In a NEW Terminal)
```bash
cd /home/nadafouad/Downloads/Autonomous-Wheelchair-master-branch/autonomous_wheelchair
source /opt/ros/humble/setup.bash
source install/setup.bash
./START_KEYBOARD_CONTROL.sh
```

Or manually:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## ⌨️ Keyboard Controls

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

## 🧪 Testing

### Manual Testing
```bash
# Send a forward command
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# Check joint states
ros2 topic echo /joint_states

# Check odometry
ros2 topic echo /odom
```

## 🔧 Troubleshooting

### Robot Not Moving?

1. **Check Topics:**
   ```bash
   ros2 topic list | grep cmd_vel
   ros2 topic list | grep joint_states
   ```

2. **Check Gazebo:**
   - Is simulation paused? (Press Play)
   - Are wheels touching the ground?
   - Check for collision penetrations

### Keyboard Control Not Working?

1. **Check if /cmd_vel topic exists:**
   ```bash
   ros2 topic list | grep cmd_vel
   ```

2. **Test with manual command:**
   ```bash
   ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
     "{linear: {x: 0.3}}"
   ```

### Clean Everything and Restart

```bash
rm -rf build install log
./STOP_ALL.sh
# Then rebuild and relaunch
colcon build
source install/setup.bash
```

## 📁 Project Structure

```
autonomous_wheelchair/
├── src/
│   ├── wheelchair_description/    # Robot URDF and configs
│   └── wheelchair_gazebo/         # Gazebo launch files
├── STOP_ALL.sh                    # Stop all processes
└── START_KEYBOARD_CONTROL.sh     # Launch keyboard control
```

## 🔗 Key Topics

- `/cmd_vel` - Velocity commands (geometry_msgs/Twist)
- `/odom` - Odometry (nav_msgs/Odometry)
- `/joint_states` - Joint positions and velocities
- `/scan` - LiDAR scan data

## 📝 Notes

- **Keyboard teleop must run in a separate terminal** (it needs an interactive terminal)
- **Wait ~15-20 seconds** after launch for Gazebo to fully initialize
- **Direct Gazebo diff drive plugin** is used (ros2_control disabled for simplicity)

## 🆘 Emergency Stop

```bash
./STOP_ALL.sh
```

Or manually:
```bash
killall -9 gz
pkill -9 -f "ros2 launch"
```



