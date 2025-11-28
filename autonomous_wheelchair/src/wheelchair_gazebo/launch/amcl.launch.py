#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('wheelchair_gazebo')
    default_map = os.path.join(pkg_share, 'maps', 'my_map.yaml')

    map_file_arg = DeclareLaunchArgument('map_file', default_value=default_map,
                                         description='Full path to map yaml file')
    scan_topic_arg = DeclareLaunchArgument('scan_topic', default_value='/scan',
                                           description='LIDAR scan topic')

    map_file = LaunchConfiguration('map_file')
    scan_topic = LaunchConfiguration('scan_topic')

    return LaunchDescription([
        map_file_arg,
        scan_topic_arg,

        # Map server (Nav2)
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': map_file, 'use_sim_time': True}]
        ),

        # AMCL (Nav2)
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'base_frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'scan_topic': scan_topic,
                'min_particles': 500,
                'max_particles': 2000,
            }]
        ),

        # lifecycle manager to autostart map_server and amcl
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'autostart': True,
                'node_names': ['map_server', 'amcl']
            }]
        ),
    ])
