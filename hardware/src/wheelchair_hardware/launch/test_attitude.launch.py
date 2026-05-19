#!/usr/bin/env python3
"""
Attitude test launch — ESP32 bridge + complementary-filter attitude node.

Starts:
  1. esp32_bridge      — serial → /imu/data  (raw accel + gyro)
  2. imu_attitude_test — complementary filter → /imu/attitude, /imu/rpy
  3. Static TFs        — base_link → imu_link,  base_link → laser

Usage:
  # Auto-detect ESP32 port (first /dev/ttyUSB* or /dev/ttyACM*)
  ros2 launch wheelchair_hardware test_attitude.launch.py

  # Specify port explicitly if two USB devices are connected
  ros2 launch wheelchair_hardware test_attitude.launch.py esp32_port:=/dev/ttyUSB0

IMPORTANT — two USB devices (ESP32 + LiDAR):
  If both are plugged in, they appear as /dev/ttyUSB0 and /dev/ttyUSB1.
  The ESP32 bridge auto-picks the FIRST port.
  Unplug the LiDAR before this test, OR pass the correct port manually.

Monitor roll/pitch/yaw:
  Terminal shows live values.
  Or: ros2 topic echo /imu/rpy
  Or: ros2 run rqt_plot rqt_plot /imu/rpy/vector/x /imu/rpy/vector/y
"""

import glob
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def find_esp32_port():
    ports = sorted(glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*'))
    if ports:
        print(f'[test_attitude] Auto-detected port: {ports[0]}')
        return ports[0]
    print('[test_attitude] WARNING: no serial port found, defaulting to /dev/ttyUSB0')
    return '/dev/ttyUSB0'


def generate_launch_description():
    auto_port = find_esp32_port()
    esp32_port = LaunchConfiguration('esp32_port', default=auto_port)

    # 1 — ESP32 bridge: serial JSON → /imu/data + /odom + TF odom→base_link
    bridge_node = Node(
        package='wheelchair_hardware',
        executable='esp32_bridge.py',
        name='esp32_bridge',
        output='screen',
        parameters=[{
            'serial_port':      esp32_port,
            'baud_rate':        115200,
            'wheel_radius':     0.15,
            'wheel_separation': 0.52,
            'encoder_ppr':      20,
        }],
    )

    # 2 — Complementary filter attitude estimator
    attitude_node = Node(
        package='wheelchair_hardware',
        executable='imu_attitude_test.py',
        name='imu_attitude_test',
        output='screen',
    )

    # 3 — Static TF: base_link → imu_link
    #     Adjust x/y/z/roll/pitch/yaw to match the actual IMU mounting position.
    imu_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_imu',
        arguments=['--x', '0.0', '--y', '0.0', '--z', '0.1',
                   '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
                   '--frame-id', 'base_link',
                   '--child-frame-id', 'imu_link'],
        output='screen',
    )

    # 4 — Static TF: base_link → laser (needed if RViz is opened later)
    laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_laser',
        arguments=['--x', '0.0', '--y', '0.0', '--z', '0.2',
                   '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
                   '--frame-id', 'base_link',
                   '--child-frame-id', 'laser'],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'esp32_port',
            default_value=auto_port,
            description='Serial port for the ESP32 (auto-detected)',
        ),
        imu_tf,
        laser_tf,
        bridge_node,
        attitude_node,
    ])
