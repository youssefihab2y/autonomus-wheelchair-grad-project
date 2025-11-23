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
./launch_keyboard_teleop.sh
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

### Test Robot Movement
```bash
./TEST_MOVEMENT.sh
```

This script will:
- Check if simulation is running
- Verify controllers are active
- Check hardware interfaces
- Test movement commands
- Verify joint velocities

### Manual Testing
```bash
# Send a forward command
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# Check controller status
ros2 service call /controller_manager/list_controllers \
  controller_manager_msgs/srv/ListControllers {}

# Check joint states
ros2 topic echo /joint_states
```

## 🔧 Troubleshooting

### Robot Not Moving?

1. **Check Controllers:**
   ```bash
   ./ACTIVATE_CONTROLLERS.sh
   ```

2. **Check Topics:**
   ```bash
   ros2 topic list | grep cmd_vel
   ros2 topic list | grep joint_states
   ```

3. **Check Controller Status:**
   ```bash
   ros2 service call /controller_manager/list_controllers \
     controller_manager_msgs/srv/ListControllers {} | grep state
   ```

4. **Verify cmd_vel Relay:**
   ```bash
   ros2 node list | grep cmd_vel_relay
   ```

5. **Check Gazebo:**
   - Is simulation paused? (Press Play)
   - Are wheels touching the ground?
   - Check for collision penetrations

### Controllers Not Activating?

1. **Wait longer** - Controllers activate at 25 seconds after launch
2. **Manually activate:**
   ```bash
   ./ACTIVATE_CONTROLLERS.sh
   ```

### Keyboard Control Not Working?

1. **Check if /cmd_vel topic exists:**
   ```bash
   ros2 topic list | grep cmd_vel
   ```

2. **Check if relay is running:**
   ```bash
   ros2 node list | grep relay
   ```

3. **Test with manual command:**
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
├── TEST_MOVEMENT.sh               # Test robot movement
├── launch_keyboard_teleop.sh      # Launch keyboard control
└── ACTIVATE_CONTROLLERS.sh        # Activate controllers manually
```

## 🔗 Key Topics

- `/cmd_vel` - Velocity commands (geometry_msgs/Twist)
- `/joint_states` - Joint positions and velocities
- `/skid_steer_controller/cmd_vel_unstamped` - Controller command topic
- `/skid_steer_controller/odom` - Odometry

## 📝 Notes

- **Keyboard teleop must run in a separate terminal** (it needs an interactive terminal)
- **Wait ~45 seconds** after launch for everything to initialize
- **Controllers activate automatically** at 25 seconds
- **cmd_vel relay** bridges `/cmd_vel` to controller's expected topic

## 🆘 Emergency Stop

```bash
./STOP_ALL.sh
```

Or manually:
```bash
killall -9 gz
killall -9 ros2_control_node
pkill -9 -f "ros2 launch"
```



