#!/usr/bin/env python3
"""
Manual control launch — ESP32 bridge + keyboard teleop.

Usage:
  ros2 launch wheelchair_hardware manual_control.launch.py
  ros2 launch wheelchair_hardware manual_control.launch.py serial_port:=/dev/ttyUSB0

Then use arrow keys / WASD in the teleop terminal to drive.

NOTE: Do NOT run keyboard_controller.py at the same time — both need the serial port.
"""

import glob
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def find_esp32_port():
    # ESP32 always plugged first → lowest ttyUSB index
    ports = sorted(glob.glob('/dev/ttyUSB*'))
    if ports:
        return ports[0]
    acm = sorted(glob.glob('/dev/ttyACM*'))
    return acm[0] if acm else '/dev/ttyUSB0'


def generate_launch_description():
    auto_port = find_esp32_port()
    serial_port = LaunchConfiguration('serial_port', default=auto_port)

    # ESP32 bridge — reads sensors, converts /cmd_vel to motor commands
    bridge_node = Node(
        package='wheelchair_hardware',
        executable='esp32_bridge.py',
        name='esp32_bridge',
        output='screen',
        parameters=[{
            'serial_port':      serial_port,
            'wheel_radius':     0.27,
            'wheel_separation': 0.52,
            'encoder_ppr':      20,    # TODO: update with real PPR
        }],
    )

    # Keyboard teleop — publishes /cmd_vel
    # Use WASD or arrow keys in the terminal where this node runs
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_keyboard',
        output='screen',
        prefix='xterm -e',    # opens in its own terminal window
        remappings=[('/cmd_vel', '/cmd_vel')],
    )

    # Static TF: base_link -> imu_link
    imu_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_imu',
        arguments=['--x', '0.0', '--y', '0.0', '--z', '0.1',
                   '--roll', '0.0', '--pitch', '0.0', '--yaw', '3.14159',
                   '--frame-id', 'base_link',
                   '--child-frame-id', 'imu_link'],
        output='screen',
    )

    # Static TF: base_link -> laser (lidar position on wheelchair)
    laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_laser',
        arguments=['--x', '-0.30', '--y', '0.0', '--z', '0.2',
                   '--roll', '0.0', '--pitch', '0.0', '--yaw', '3.14159',
                   '--frame-id', 'base_link',
                   '--child-frame-id', 'laser'],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_port',
            default_value=auto_port,
            description='ESP32 USB serial port (auto-detected)',
        ),
        imu_tf,
        laser_tf,
        bridge_node,
        teleop_node,
    ])
