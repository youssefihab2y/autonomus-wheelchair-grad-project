#!/usr/bin/env python3
"""
Launch RPLidar A1M8 with auto USB port detection.
Scans /dev/ttyUSB0..9 and picks the first one that exists.
"""

import glob
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def find_lidar_port():
    # ESP32 is USB0 (plugged first), LiDAR is USB1 (plugged second)
    ports = sorted(glob.glob('/dev/ttyUSB*'))
    port = ports[1] if len(ports) > 1 else (ports[0] if ports else '/dev/ttyUSB1')
    print(f'[rplidar] Auto-detected port: {port}')
    return port


def generate_launch_description():
    auto_port = find_lidar_port()

    channel_type     = LaunchConfiguration('channel_type',     default='serial')
    serial_port      = LaunchConfiguration('serial_port',      default=auto_port)
    serial_baudrate  = LaunchConfiguration('serial_baudrate',  default='115200')
    frame_id         = LaunchConfiguration('frame_id',         default='laser')
    inverted         = LaunchConfiguration('inverted',         default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')

    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        parameters=[{
            'channel_type':    channel_type,
            'serial_port':     serial_port,
            'serial_baudrate': serial_baudrate,
            'frame_id':        frame_id,
            'inverted':        inverted,
            'angle_compensate': angle_compensate,
        }],
        output='screen',
        respawn=True,
        respawn_delay=2.0,
    )

    # Static TF: base_link -> laser
    # Update x/z once you measure the actual lidar position on the wheelchair
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_lidar',
        arguments=['--x', '-0.30', '--y', '0.0', '--z', '0.2',
                   '--roll', '0.0', '--pitch', '0.0', '--yaw', '3.14159',
                   '--frame-id', 'base_link',
                   '--child-frame-id', 'laser'],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('channel_type',     default_value='serial'),
        DeclareLaunchArgument('serial_port',      default_value=auto_port),
        DeclareLaunchArgument('serial_baudrate',  default_value='115200'),
        DeclareLaunchArgument('frame_id',         default_value='laser'),
        DeclareLaunchArgument('inverted',         default_value='false'),
        DeclareLaunchArgument('angle_compensate', default_value='true'),
        rplidar_node,
        static_tf,
    ])
