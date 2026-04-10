#!/usr/bin/env python3
"""
Launch file for Gazebo simulation with SLAM mapping.
This combines the robot simulation with SLAM toolbox for map creation.
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
    
    # Include SLAM launch (delayed to ensure robot is spawned)
    slam_launch = TimerAction(
        period=10.0,  # Wait 10 seconds for robot to spawn
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        pkg_wheelchair_gazebo,
                        'launch',
                        'slam_mapping.launch.py'
                    ])
                ]),
                launch_arguments={
                    'use_sim_time': use_sim_time,
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
        warehouse_launch,
        slam_launch,
    ])

