#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
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
    params_file = LaunchConfiguration('params_file')
    map_file = LaunchConfiguration('map', default='')
    
    # Nav2 parameters file
    nav2_params_file = PathJoinSubstitution([
        pkg_wheelchair_gazebo,
        'config',
        'nav2_params.yaml'
    ])
    
    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=nav2_params_file,
        description='Full path to the Nav2 parameters file'
    )
    
    declare_map_file_cmd = DeclareLaunchArgument(
        'map',
        default_value='',
        description='Full path to map yaml file to load'
    )
    
    # Include Nav2 bringup launch file
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                pkg_nav2_bringup,
                'launch',
                'bringup_launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'map': map_file,
        }.items()
    )
    
    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_params_file_cmd,
        declare_map_file_cmd,
        nav2_bringup_launch,
    ])

