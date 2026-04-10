# Wheelchair Description Package

ROS 2 package containing the URDF/Xacro description of the autonomous wheelchair robot.

## Package Structure

```
wheelchair_description/
├── urdf/
│   ├── wheelchair.urdf.xacro          # Main robot description with parameters
│   ├── wheelchair.ros2_control.xacro   # ros2_control configuration
│   └── wheelchair.gazebo.xacro        # Gazebo Fortress plugins
├── config/
│   └── controllers.yaml               # ros2_control controller configuration
├── launch/
│   └── view_robot_rviz2.launch.py     # Launch file for RViz visualization
└── meshes/                            # 3D mesh files (if needed)
```

## Features

- **4-Wheel Skid Steer Drive**: Four wheels (2 front, 2 back) for omnidirectional movement
- **Parameterized Design**: All dimensions and properties are parameterized using xacro properties
- **ROS 2 Control**: Integrated with `ros2_control` for hardware abstraction
- **Sensors**:
  - IMU sensor for orientation
  - RGBD camera (Kinect-like) for depth perception
  - 360° LiDAR for mapping and obstacle detection
- **Gazebo Fortress Compatible**: All plugins updated for Gazebo Sim 6.17.0

## Robot Parameters

### Base Dimensions
- Length: 0.50 m
- Width: 0.44 m
- Height: 0.035 m
- Mass: 10.0 kg

### Wheels
- Back wheels: Radius 0.15 m, Width 0.04 m
- Front wheels: Radius 0.095 m, Width 0.03 m
- Wheel separation: 0.52 m (left-right)
- Wheel base: 0.50 m (front-back)

## Usage

### Build the Package

```bash
cd ~/autonomous_wheelchair
colcon build --packages-select wheelchair_description
source install/setup.bash
```

### Spawn Robot in Gazebo

1. **Start Gazebo:**
   ```bash
   ign gazebo empty.sdf
   ```

2. **In another terminal, spawn the robot:**
   ```bash
   source install/setup.bash
   ros2 launch wheelchair_gazebo warehouse_with_robot.launch.py
   ```

### View Robot in RViz2

```bash
ros2 run rviz2 rviz2
```

Add:
- RobotModel (topic: `/robot_description`)
- TF
- LaserScan (topic: `/scan`)
- Image (topic: `/camera/image`)
- IMU (topic: `/imu_data`)

## ROS 2 Control

The robot uses `ros2_control` with:
- **Controller**: `diff_drive_controller` configured for skid steer
- **Joint State Broadcaster**: Publishes joint states
- **Hardware Interface**: `gz_ros2_control` for Gazebo simulation

### Controller Configuration

The controller is configured in `config/controllers.yaml`:
- Left wheels: `wheel2_joint`, `wheel4_joint`
- Right wheels: `wheel1_joint`, `wheel3_joint`
- Max linear velocity: 1.0 m/s
- Max angular velocity: 2.0 rad/s

## Topics

- `/cmd_vel` (geometry_msgs/Twist): Velocity commands
- `/odom` (nav_msgs/Odometry): Odometry data
- `/scan` (sensor_msgs/LaserScan): LiDAR scan data
- `/imu_data` (sensor_msgs/Imu): IMU sensor data
- `/camera/image` (sensor_msgs/Image): RGB camera image
- `/camera/depth_image` (sensor_msgs/Image): Depth image
- `/camera/points` (sensor_msgs/PointCloud2): Point cloud data

## TF Frames

- `chassis`: Base frame
- `base_uplayer_link`: Main platform
- `wheel1_link` to `wheel4_link`: Wheel frames
- `imu_sensor_link`: IMU frame
- `camera_link`: Camera frame
- `base_laser`: LiDAR frame

## Migration Notes

This package was migrated from ROS 1 to ROS 2:
- **ROS 1 → ROS 2**: Updated all plugins and launch files
- **Gazebo Classic → Gazebo Fortress**: Updated sensor plugins
- **gazebo_ros → ros2_control**: Replaced with ros2_control architecture
- **Parameterized**: Added xacro properties for easy customization

## Next Steps

1. Test robot spawning in Gazebo
2. Verify all sensors are publishing data
3. Test basic movement with teleop
4. Integrate with Navigation2 stack

