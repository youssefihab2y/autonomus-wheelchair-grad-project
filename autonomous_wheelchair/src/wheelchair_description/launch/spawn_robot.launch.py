#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Package directories
    pkg_share = FindPackageShare('wheelchair_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    z_pose = LaunchConfiguration('z_pose', default='0.1')
    
    # URDF file path
    urdf_file = PathJoinSubstitution([
        pkg_share,
        'urdf',
        'wheelchair.urdf.xacro'
    ])
    
    # Launch Gazebo with empty world (or specify a world file)
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': 'empty.sdf'  # Use empty world, or specify a world file path
        }.items()
    )
    
    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'robot_description': ParameterValue(
                Command([
                    FindExecutable(name='xacro'),
                    ' ',
                    urdf_file
                ]), value_type=str
            )}
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )
    
    # Joint State Publisher (for visualization) - Optional, only needed for RViz
    # joint_state_publisher_node = Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     name='joint_state_publisher',
    #     output='screen',
    #     parameters=[{'use_sim_time': use_sim_time}]
    # )
    
    # Spawn robot in Gazebo (with delay to ensure world is loaded)
    spawn_entity_node = TimerAction(
        period=5.0,  # Wait 5 seconds for Gazebo world to fully load
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                name='spawn_wheelchair',
                arguments=[
                    '-entity', 'wheelchair',
                    '-topic', 'robot_description',
                    '-x', x_pose,
                    '-y', y_pose,
                    '-z', z_pose
                ],
                output='screen'
            )
        ]
    )
    
    # Bridge for ROS 2 <-> Gazebo communication
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/imu_data@sensor_msgs/msg/Imu@gz.msgs.IMU',
            '/camera/image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
            '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
        ],
        output='screen'
    )
    
    # Controller Manager - Required for ros2_control
    # Start with delay to ensure robot is spawned and plugin is loaded first
    controller_manager_node = TimerAction(
        period=8.0,  # Wait 8 seconds for robot to spawn and hardware plugin to initialize
        actions=[
            Node(
                package='controller_manager',
                executable='ros2_control_node',
                name='controller_manager',
                parameters=[
                    PathJoinSubstitution([pkg_share, 'config', 'controllers.yaml']),
                    {'use_sim_time': use_sim_time}
                ],
                output='screen'
            )
        ]
    )
    
    # Load and activate controllers (after controller manager starts)
    load_controllers = TimerAction(
        period=12.0,  # Wait 12 seconds for controller manager and hardware to be fully ready
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                name='controller_spawner',
                arguments=[
                    'joint_state_broadcaster',
                    'skid_steer_controller',
                    '--controller-manager', 'controller_manager',
                    '--controller-manager-timeout', '60',  # Increased timeout for activation
                    '--switch-timeout', '30',  # Timeout for switch/activation operations
                    '--activate-as-group'  # Activate all controllers as a group
                ],
                output='screen'
            )
        ]
    )
    
    # Topic relay: Bridge /cmd_vel to /skid_steer_controller/cmd_vel_unstamped
    cmd_vel_relay = TimerAction(
        period=14.0,  # Wait for controllers to load first
        actions=[
            Node(
                package='topic_tools',
                executable='relay',
                name='cmd_vel_relay',
                arguments=[
                    '/cmd_vel',
                    '/skid_steer_controller/cmd_vel_unstamped'  # Controller subscribes to _unstamped version
                ],
                output='screen'
            )
        ]
    )
    
    return LaunchDescription([
        # Set plugin path for Gazebo to find ros2_control plugin
        SetEnvironmentVariable(
            'GZ_SIM_SYSTEM_PLUGIN_PATH',
            '/opt/ros/humble/lib'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'x_pose',
            default_value='0.0',
            description='Initial x position of the robot'
        ),
        DeclareLaunchArgument(
            'y_pose',
            default_value='0.0',
            description='Initial y position of the robot'
        ),
        DeclareLaunchArgument(
            'z_pose',
            default_value='0.1',
            description='Initial z position of the robot'
        ),
        gz_sim,  # Launch Gazebo first
        robot_state_publisher_node,
        # joint_state_publisher_node,  # Optional - uncomment if you have joint_state_publisher installed
        spawn_entity_node,  # Spawn robot after Gazebo is ready
        bridge_node,
        controller_manager_node,
        load_controllers,  # Load and activate controllers automatically
        cmd_vel_relay,  # Bridge /cmd_vel to controller's expected topic
    ])

