#!/usr/bin/env python3
"""
Launch file for Gazebo + Nav2 + RViz + rosbridge (web interface).
Use this when you want to connect the web dashboard to the simulation.
Requires: sudo apt install ros-humble-rosbridge-suite
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_wheelchair_gazebo = FindPackageShare('wheelchair_gazebo')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_file = LaunchConfiguration('map', default='')

    # Base warehouse + Nav2 + RViz launch
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                pkg_wheelchair_gazebo,
                'launch',
                'warehouse_with_nav2.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_file,
        }.items()
    )

    # Rosbridge WebSocket server (for web interface)
    rosbridge_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='rosbridge_server',
                executable='rosbridge_websocket',
                name='rosbridge_websocket',
                output='screen',
                parameters=[{'port': 9090}],
            )
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock'
        ),
        DeclareLaunchArgument(
            'map',
            default_value='',
            description='Full path to map yaml file (optional)'
        ),
        nav2_launch,
        rosbridge_node,
    ])
