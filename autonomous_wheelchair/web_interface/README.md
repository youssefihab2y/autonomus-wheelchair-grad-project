# Web Interface for Autonomous Wheelchair

This folder contains a web-based interface to visualize and control your autonomous wheelchair simulation. It integrates with **Gazebo**, **RViz**, and **ROS 2** through **rosbridge** (WebSocket bridge).

## Architecture

```
┌─────────────────┐     ┌──────────────────┐     ┌─────────────────┐
│  Gazebo Sim     │     │   ROS 2 Core     │     │  RViz2          │
│  (Physics,      │◄───►│   Topics, TF,    │◄───►│  (Visualization)│
│   Sensors)      │     │   Services       │     │                 │
└────────┬────────┘     └────────┬─────────┘     └─────────────────┘
         │                       │
         │              ros_gz_bridge
         │                       │
         │                       │  rosbridge_suite (WebSocket)
         │                       │
         │                       ▼
         │              ┌──────────────────┐
         └─────────────►│  Web Interface   │
                        │  (Browser)       │
                        │  - Map view      │
                        │  - Robot pose    │
                        │  - Sensor data   │
                        │  - Send goals    │
                        └──────────────────┘
```

## Where This Folder Lives

```
autonomous_wheelchair/
├── src/                    # ROS 2 packages (wheelchair_gazebo, etc.)
├── build/
├── install/
├── log/
└── web_interface/          # ← You are here (web app, not a ROS package)
    ├── README.md
    ├── index.html
    └── package.json        # For npm-based tools (optional)
```

## Prerequisites

1. **Install rosbridge_suite** (ROS 2 WebSocket bridge):
   ```bash
   sudo apt update
   sudo apt install ros-humble-rosbridge-suite
   ```

2. **Install npm** (if using the optional Node.js tools):
   ```bash
   # For serving the web app locally
   sudo apt install npm
   ```

## Quick Start

### Step 1: Launch your simulation (Gazebo + Nav2 + RViz)

In one terminal:
```bash
cd ~/.../autonomous_wheelchair
source install/setup.bash
ros2 launch wheelchair_gazebo warehouse_with_nav2.launch.py
# Or for SLAM: ros2 launch wheelchair_gazebo warehouse_with_slam_rviz.launch.py
```

### Step 2: Launch rosbridge (WebSocket server)

In a second terminal:
```bash
source install/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```
This starts a WebSocket server at `ws://localhost:9090`.

### Step 3: Serve and open the web interface

In a third terminal:
```bash
cd autonomous_wheelchair/web_interface
# Option A: Python (no npm needed)
python3 -m http.server 8080

# Option B: npx (if you have Node.js)
# npx serve .
```

Then open in your browser: **http://localhost:8080**

## Key ROS 2 Topics (for your web app)

| Topic | Type | Description |
|-------|------|-------------|
| `/map` | nav_msgs/OccupancyGrid | Map for localization/navigation |
| `/scan` | sensor_msgs/LaserScan | Lidar data |
| `/odom` | nav_msgs/Odometry | Robot pose and velocity |
| `/tf` | tf2_msgs/TFMessage | Transform tree |
| `/cmd_vel` | geometry_msgs/Twist | Velocity commands (to control robot) |
| `/goal_pose` | geometry_msgs/PoseStamped | Nav2 goal (via action) |
| `/camera/image` | sensor_msgs/Image | Camera feed |

## One-Command Launch (Recommended)

Use the **warehouse_with_nav2_web** launch to start Gazebo + Nav2 + RViz + rosbridge together:

```bash
source install/setup.bash
ros2 launch wheelchair_gazebo warehouse_with_nav2_web.launch.py
```

Then serve the web app (`python3 -m http.server 8080` in `web_interface/`) and open http://localhost:8080.

## Troubleshooting: Map not appearing

1. **Ensure `/map` is published** – The map is only published when using Nav2 with a pre-built map (AMCL):
   ```bash
   ros2 launch wheelchair_gazebo warehouse_with_nav2_web.launch.py map:=$(pwd)/src/wheelchair_gazebo/maps/mymap.yaml
   ```
   If you launch without a map file, map_server will not start and `/map` stays empty.

2. **Check that `/map` is active**:
   ```bash
   ros2 topic list | grep map
   ros2 topic echo /map --once
   ```
   If nothing appears, the map server has not loaded a map.

3. **Wait for startup** – Map_server starts ~10–15 seconds after launch. Connect the web dashboard after the simulation is fully up.

## Alternative: Foxglove Studio

For a ready-made web visualization (maps, 3D, plots) without coding:

1. Install: `sudo apt install ros-humble-foxglove-bridge`
2. Launch: `ros2 launch foxglove_bridge foxglove_bridge_launch.xml`
3. Open: https://app.foxglove.dev → Connect → Foxglove WebSocket → `ws://localhost:8765`

Foxglove provides map, TF, laser scan, and more out of the box.
