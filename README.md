# 🦽 Self-Driving Wheelchair (Graduation Project)

This project is my **graduation project** as part of a **team of five members**. It focuses on the development of an autonomous self-driving wheelchair, which uses **ROS (Robot Operating System)**, **SLAM (Simultaneous Localization and Mapping)**, and various sensors (Kinect and LIDAR) to achieve autonomous navigation and obstacle avoidance. The project has been tested in both **simulation** (using Gazebo) and **real-world** environments.

The project is proudly sponsored by the **Information Technology Industry Development Agency (ITIDA)**.

Our project was part of **three competitions**:

- **Made In Egypt (MIE)**
- **Egypt Industry 4.0 Challenge**
- **IEEE IC-SIT 2024**

We reached the **finals** in the **IEEE IC-SIT 2024** competition. The project received an **A\* Excellent** evaluation for its outstanding implementation and performance.

---

## Project Structure

The project is divided into the following main components:

### 1. **Robotics (Robot Control & Navigation)**

This section involves the development and deployment of the robot's control system, including the following key components:

- **ROS (Robot Operating System)** for managing communication between various sensors and actuators.
- **SLAM** for localization and mapping, allowing the wheelchair to navigate autonomously.
- **Sensor Integration**: LIDAR and Kinect sensors are used for obstacle detection and mapping.
- **Navigation Algorithms**: The robot uses algorithms like **A\* (global planner)** and **DWA (local planner)** for path planning and obstacle avoidance.
- **Robot State**: The robot continuously updates its position (x, y, orientation) and velocity data for real-time tracking.

### 2. **Web App (User Interface for Control)**

The web app serves as the interface for controlling and monitoring the robot remotely:

- **Connection Component**: Displays if the robot is connected or disconnected using the ROSSerial WebSocket.
- **Teleoperation Component**: Provides joystick controls for moving the robot.
- **RobotState Component**: Displays the real-time status of the robot, including location, velocity, and orientation.
- **Map Component**: Displays the environment map generated from the robot's sensors (Kinect/LIDAR) and allows users to specify navigation goals.
- **Footer**: Contains information like copyright and team details.

### 3. **Simulations (Gazebo)**

The project has been tested in simulation using **Gazebo**. This allows for safe and scalable testing of the robot's navigation and obstacle avoidance capabilities before deployment in the real world.

---

## Requirements

### System Requirements
- **Ubuntu 22.04 LTS** (Jammy Jellyfish)
- **ROS 2 Humble Desktop** - [Installation Guide](https://docs.ros.org/en/humble/Installation.html)
- **Gazebo Fortress** (Gazebo Sim 6.17.0) - [Installation Guide](https://gazebosim.org/docs/fortress/install)
- **Python 3.10+**
- **colcon** build tool (included with ROS 2)

### ROS 2 Packages
The following ROS 2 packages are required (install via apt):
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
  python3-colcon-common-extensions
```

### Python Dependencies
```bash
pip install -r requirements.txt
```

## Setup & Installation

### 1. Clone the Repository
```bash
git clone <repository-url>
cd Autonomous-Wheelchair-master-branch
```

### 2. Install ROS 2 Dependencies
```bash
source /opt/ros/humble/setup.bash
```

### 3. Build the Workspace
```bash
cd autonomous_wheelchair
colcon build --symlink-install
source install/setup.bash
```

### 4. Launch Simulation
```bash
# Terminal 1: Launch Gazebo simulation
ros2 launch wheelchair_gazebo warehouse_with_robot.launch.py

# Terminal 2: Launch RViz visualization (optional)
ros2 launch wheelchair_description view_robot_rviz2.launch.py

# Terminal 3: Control the robot with keyboard
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

For detailed usage instructions, see [QUICK_START.md](./autonomous_wheelchair/QUICK_START.md).

### Hardware Requirements (for Real-World Deployment)
- **LIDAR** sensor for mapping and obstacle detection
- **Kinect** sensor for depth perception
- **Wheelchair base** with motor controllers

---

## Demo

### **Live Demo**

For a live demo of the **Self-Driving Wheelchair** system:  
[Watch on LinkedIn](https://www.linkedin.com/posts/yousif-adel-a601641b1_apexdriveinnovators-ros-selfdrivingwheelchair-activity-7219724350126514176-LP5P?utm_source=share&utm_medium=member_desktop&rcm=ACoAADFSougBbplLvCFvoq2oVcM3uoEe_eK2zig)

## 📦 Installation

See the [Requirements](#requirements) section above for detailed installation instructions.

### Quick Installation
```bash
# Install ROS 2 Humble (if not already installed)
sudo apt update
sudo apt install -y ros-humble-desktop

# Install required ROS 2 packages
sudo apt install -y \
  ros-humble-ros-gz-sim \
  ros-humble-ros-gz-bridge \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  ros-humble-teleop-twist-keyboard \
  ros-humble-rviz2

# Clone and build
git clone <repository-url>
cd Autonomous-Wheelchair-master-branch/autonomous_wheelchair
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 🚀 Quick Start

For detailed usage instructions, see [QUICK_START.md](./autonomous_wheelchair/QUICK_START.md).

## 🤝 Contributing

Feel free to contribute by:

- **Reporting bugs**
- **Submitting feature requests**
- **Improving documentation**
- **Creating pull requests for improvements**

---

## Acknowledgements

This project was part of my **graduation project** as a team member. We thank everyone who supported us in completing this project, including faculty members and competition organizers.

## I would like to thank The engineer **Eng. Hesham Gamal** for his constant support for us throughout the year

---

## 🔗 Related Projects

- [🦽 Autonomous Mobile Robot (Simulation)](https://github.com/YousifAdel170/Autonomous-Mobile-Robot)

---

## License

## This project is licensed under the MIT License - see the [LICENSE](./LICENSE) file for details.

---

## 🙋‍♂️ Author

**Yousif Adel** – [LinkedIn](https://www.linkedin.com/in/yousif-adel-a601641b1/)
