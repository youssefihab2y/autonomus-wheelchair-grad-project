#!/usr/bin/env python3
"""
Launch file for Gazebo simulation with Nav2 navigation stack.
This combines the robot simulation with Nav2 for autonomous navigation.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # Package directories
    pkg_wheelchair_gazebo = FindPackageShare('wheelchair_gazebo')
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_nav2 = LaunchConfiguration('use_nav2', default='true')
    map_file = LaunchConfiguration('map', default='')
    
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
    
    # Include Nav2 bringup (delayed to ensure robot is spawned)
    nav2_launch = TimerAction(
        period=10.0,  # Wait 10 seconds for robot to spawn
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        pkg_wheelchair_gazebo,
                        'launch',
                        'nav2_bringup.launch.py'
                    ])
                ]),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'map': map_file,
                }.items()
            )
        ]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'use_nav2',
            default_value='true',
            description='Launch Nav2 navigation stack'
        ),
        DeclareLaunchArgument(
            'map',
            default_value='',
            description='Full path to map yaml file to load (optional, for localization only)'
        ),
        warehouse_launch,
        nav2_launch,
    ])

