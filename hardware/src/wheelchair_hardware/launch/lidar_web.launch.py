#!/usr/bin/env python3
"""
LiDAR → SLAM → Website pipeline (no ESP32, no Nav2 needed).

Starts:
  1. RPLidar   — publishes /scan
  2. SLAM Toolbox — builds live /map from /scan
  3. map_bridge — serves /map.json at http://localhost:5000
  4. rosbridge  — WebSocket at ws://localhost:9090

Then in a second terminal:
  python3 -m http.server 8080 --directory \
    ~/autonomus-wheelchair-grad-project-nav2/autonomous_wheelchair/web_interface
  → open http://localhost:8080
"""

import glob
import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _find_lidar():
    ports = sorted(glob.glob('/dev/ttyUSB*'))
    port = ports[0] if ports else '/dev/ttyUSB0'
    print(f'[lidar_web] LiDAR port → {port}')
    return port


def generate_launch_description():
    pkg        = FindPackageShare('wheelchair_hardware')
    auto_port  = _find_lidar()
    slam_params = PathJoinSubstitution([pkg, 'config', 'slam_params.yaml'])

    port_arg = DeclareLaunchArgument(
        'serial_port', default_value=auto_port,
        description='RPLidar USB port (auto-detected)',
    )

    # ── LiDAR ─────────────────────────────────────────────────────────────────
    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg, 'launch', 'rplidar.launch.py'])
        ),
        launch_arguments={'serial_port': LaunchConfiguration('serial_port')}.items(),
    )

    # ── SLAM Toolbox — 2 s delay so /scan is publishing first ─────────────────
    slam = TimerAction(period=2.0, actions=[
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_params, {'use_sim_time': False}],
        )
    ])

    # ── Map bridge — HTTP :5000 for web dashboard ─────────────────────────────
    map_bridge = Node(
        package='wheelchair_hardware', executable='map_bridge.py',
        name='map_bridge', output='screen',
    )

    # ── rosbridge — WebSocket :9090 for web dashboard topics ──────────────────
    rosbridge = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
        parameters=[{'port': 9090, 'ssl': False, 'authenticate': False}],
    )

    return LaunchDescription([
        port_arg,
        lidar,
        slam,
        map_bridge,
        rosbridge,
    ])
