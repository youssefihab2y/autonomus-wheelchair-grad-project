#!/usr/bin/env python3
"""
Autonomous navigation launch — slam_toolbox localization + Nav2 stack.

Starts:
  1. ESP32 bridge       — motor control + odometry + IMU
  2. RPLidar            — laser scanner
  3. Static TFs         — base_link → imu_link / laser / odom (identity)
  4. Map server         — publishes /map from saved YAML (for costmaps)
  5. slam_toolbox       — localization via scan matching (publishes map→odom TF)
  6. Nav2 stack         — planner + DWB controller + costmaps (no AMCL)

NOTE: slam_toolbox handles ALL localization. AMCL is NOT used.
TF chain: map → odom (slam_toolbox) → base_link (static identity)

Prerequisites:
  # Build a map first:
  ros2 launch wheelchair_hardware slam_mapping.launch.py
  ~/save_map.sh     ← saves both .yaml/.pgm AND .posegraph

Usage:
  source hardware/install/setup.bash
  ros2 launch wheelchair_hardware navigation.launch.py map:=~/maps/my_map.yaml
"""

import glob
import os
import ament_index_python.packages as aip

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# ── port auto-detection ───────────────────────────────────────────────────────

def _find_ports():
    # ESP32 plugged first → lowest ttyUSB index; LiDAR plugged second → next index
    usb = sorted(glob.glob('/dev/ttyUSB*'))
    acm = sorted(glob.glob('/dev/ttyACM*'))
    esp32 = usb[0] if usb else (acm[0] if acm else '/dev/ttyUSB0')
    lidar = usb[1] if len(usb) > 1 else '/dev/ttyUSB1'
    print(f'[navigation] ESP32  port → {esp32}')
    print(f'[navigation] LiDAR  port → {lidar}')
    return lidar, esp32


# ── launch description ────────────────────────────────────────────────────────

def generate_launch_description():
    pkg      = aip.get_package_share_directory('wheelchair_hardware')
    nav2_pkg = aip.get_package_share_directory('nav2_bringup')

    lidar_auto, esp32_auto = _find_ports()

    # ── launch arguments ─────────────────────────────────────────────────────
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.expanduser('~/maps/my_map.yaml'),
        description='Full path to map YAML saved by map_saver_cli',
    )
    lidar_port_arg = DeclareLaunchArgument(
        'lidar_port',
        default_value=lidar_auto,
        description='RPLidar serial port (auto-detected)',
    )
    esp32_port_arg = DeclareLaunchArgument(
        'esp32_port',
        default_value=esp32_auto,
        description='ESP32 serial port (auto-detected)',
    )

    map_yaml   = LaunchConfiguration('map')
    lidar_port = LaunchConfiguration('lidar_port')
    esp32_port = LaunchConfiguration('esp32_port')

    nav2_params = os.path.join(pkg, 'config', 'nav2_hardware_params.yaml')

    # ── static TF publishers ─────────────────────────────────────────────────
    imu_tf = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='static_tf_imu',
        arguments=['--x', '0.0', '--y', '0.0', '--z', '0.1',
                   '--roll', '0.0', '--pitch', '0.0', '--yaw', '3.14159',
                   '--frame-id', 'base_link', '--child-frame-id', 'imu_link'],
    )
    # NOTE: base_link→laser TF is published by rplidar.launch.py (static_tf_lidar)
    # Do NOT add a second publisher here — TF_REPEATED_DATA warnings corrupt tf2.

    # Identity odom→base_link — slam_toolbox publishes map→odom dynamically
    odom_tf = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='static_tf_odom',
        arguments=['--x', '0.0', '--y', '0.0', '--z', '0.0',
                   '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
                   '--frame-id', 'odom', '--child-frame-id', 'base_link'],
    )

    # ── ESP32 bridge ─────────────────────────────────────────────────────────
    bridge = Node(
        package='wheelchair_hardware', executable='esp32_bridge.py',
        name='esp32_bridge', output='screen',
        parameters=[{
            'serial_port':      esp32_port,
            'wheel_radius':     0.27,
            'wheel_separation': 0.52,
            'encoder_ppr':      20,
            'max_pwm':          150,
            'max_linear_vel':   0.2,
        }],
    )

    # ── RPLidar ──────────────────────────────────────────────────────────────
    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'rplidar.launch.py')
        ),
        launch_arguments={'serial_port': lidar_port}.items(),
    )

    # ── Map server — loads saved .yaml/.pgm map for Nav2 costmaps ────────────
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            nav2_params,
            {'yaml_filename': map_yaml},
        ],
    )

    # Lifecycle manager activates map_server automatically
    map_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart':    True,
            'node_names':   ['map_server'],
        }],
    )

    # ── slam_toolbox localization — scan matching only, no odometry needed ───
    slam_loc = Node(
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            os.path.join(pkg, 'config', 'slam_localization.yaml'),
            {'use_sim_time': False},
        ],
    )

    # ── Nav2 navigation stack — NO AMCL, slam_toolbox handles localization ───
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_pkg, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file':  nav2_params,
        }.items(),
    )

    # ── RViz ─────────────────────────────────────────────────────────────────
    rviz_cfg = os.path.join(nav2_pkg, 'rviz', 'nav2_default_view.rviz')
    rviz = Node(
        package='rviz2', executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_cfg],
        output='screen',
    )

    return LaunchDescription([
        # Arguments
        map_arg, lidar_port_arg, esp32_port_arg,

        # Hardware — start immediately
        imu_tf, odom_tf,
        bridge,
        lidar,

        # Map server — start early so costmaps have /map
        map_server, map_lifecycle,

        # slam_toolbox localization — 2s after lidar starts
        TimerAction(period=2.0, actions=[slam_loc]),

        # Nav2 navigation stack — 4s (slam_toolbox must publish map→odom first)
        TimerAction(period=4.0, actions=[nav2]),

        # RViz — optional, comment out if running headless
        TimerAction(period=5.0, actions=[rviz]),
    ])
