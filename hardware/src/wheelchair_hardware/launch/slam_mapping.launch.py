#!/usr/bin/env python3
"""
SLAM mapping with real RPLidar A1M8.

Drives around with keyboard and builds a map using pure scan matching
(no wheel encoders needed — odom_frame == base_frame in slam_params.yaml).

Usage:
  ~/slam.sh          ← wrapper that sources setup and passes correct port

  In a separate terminal, drive with keyboard:
  source hardware/install/setup.bash
  ros2 run wheelchair_hardware keyboard_drive.py

  When the map looks complete, save it (while this launch is still running):
  ~/save_map.sh
"""

import glob
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def find_lidar_port():
    # ESP32 is USB0, LiDAR is USB1 (plugged second)
    ports = sorted(glob.glob('/dev/ttyUSB*'))
    return ports[1] if len(ports) > 1 else (ports[0] if ports else '/dev/ttyUSB1')


def generate_launch_description():
    pkg = FindPackageShare('wheelchair_hardware')

    auto_port   = find_lidar_port()
    serial_port = LaunchConfiguration('serial_port', default=auto_port)
    slam_params = PathJoinSubstitution([pkg, 'config', 'slam_params.yaml'])

    # --- RPLidar (includes base_link→laser static TF) ---
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg, 'launch', 'rplidar.launch.py'])
        ]),
        launch_arguments={'serial_port': serial_port}.items(),
    )

    # --- SLAM Toolbox: 2 s delay lets lidar publish first ---
    slam_node = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='slam_toolbox',
                executable='async_slam_toolbox_node',
                name='slam_toolbox',
                output='screen',
                parameters=[
                    slam_params,
                    {'use_sim_time': False},
                ],
            )
        ]
    )

    # --- RViz: 3 s delay lets SLAM initialise first ---
    rviz_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', PathJoinSubstitution([
                    FindPackageShare('nav2_bringup'), 'rviz', 'nav2_default_view.rviz'
                ])],
                parameters=[{'use_sim_time': False}],
            )
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_port',
            default_value=auto_port,
            description='USB port for RPLidar (auto-detected at launch time)',
        ),
        rplidar_launch,
        slam_node,
        rviz_node,
    ])
