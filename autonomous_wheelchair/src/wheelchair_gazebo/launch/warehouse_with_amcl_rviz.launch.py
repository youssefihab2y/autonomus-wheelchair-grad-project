#!/usr/bin/env python3
"""
Launch file for Gazebo simulation with AMCL localization and RViz.
This combines everything: robot simulation, localization, and visualization.
Use this when you have a saved map and want to navigate.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # Package directories
    pkg_wheelchair_gazebo = FindPackageShare('wheelchair_gazebo')
    pkg_nav2_bringup = FindPackageShare('nav2_bringup')
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_file = LaunchConfiguration('map_file')
    scan_topic = LaunchConfiguration('scan_topic', default='/scan')
    
    # Default map path - use FindPackageShare to get correct path
    pkg_share = PathJoinSubstitution([
        pkg_wheelchair_gazebo,
        'maps',
        'mymap.yaml'
    ])
    
    # Launch arguments
    map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value=pkg_share,
        description='Full path to map yaml file'
    )
    
    scan_topic_arg = DeclareLaunchArgument(
        'scan_topic',
        default_value='/scan',
        description='LIDAR scan topic'
    )
    
    # Include the base warehouse launch (Gazebo + Robot)
    warehouse_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                pkg_wheelchair_gazebo,
                'launch',
                'warehouse_with_robot.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items()
    )
    
    # Map Server Node (delayed to ensure robot is spawned)
    # Note: map_server needs absolute path to YAML file, and YAML must have absolute path to PGM
    map_server_node = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                parameters=[{
                    'yaml_filename': map_file,
                    'use_sim_time': True
                }]
            )
        ]
    )
    
    # AMCL Node (delayed to ensure robot is spawned)
    amcl_node = TimerAction(
        period=10.0,
        actions=[
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
            )
        ]
    )
    
    # Lifecycle Manager (delayed to ensure map_server and amcl are launched)
    lifecycle_manager = TimerAction(
        period=10.0,
        actions=[
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
            )
        ]
    )
    
    # Launch RViz (delayed to ensure everything is running)
    # Note: nav2_default_view.rviz should include Map display, but if map doesn't appear:
    # 1. In RViz, set Fixed Frame to 'map'
    # 2. Add Map display manually: Topic: /map, Durability: Transient Local
    rviz_launch = TimerAction(
        period=15.0,  # Wait 15 seconds for everything to initialize
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', PathJoinSubstitution([
                    pkg_nav2_bringup,
                    'rviz',
                    'nav2_default_view.rviz'
                ])],
                parameters=[{'use_sim_time': use_sim_time}]
            )
        ]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        map_file_arg,
        scan_topic_arg,
        warehouse_launch,
        map_server_node,
        amcl_node,
        lifecycle_manager,
        rviz_launch,
    ])
