#!/usr/bin/env python3
"""
Launch file for Gazebo simulation with SLAM mapping and RViz.
This combines everything: robot simulation, SLAM, and visualization.
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
    
    # Launch RViz (delayed to ensure everything is running)
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
        warehouse_launch,
        slam_launch,
        rviz_launch,
    ])

