#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Package directories
    pkg_wheelchair_desc = FindPackageShare('wheelchair_description')
    pkg_wheelchair_gazebo = get_package_share_directory('wheelchair_gazebo')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    # World file path - use pre-made tugbot warehouse world (no edits)
    # Copied to workspace to avoid path issues with spaces
    world_file = os.path.join(pkg_wheelchair_gazebo, 'worlds', 'tugbot_warehouse.sdf')
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    z_pose = LaunchConfiguration('z_pose', default='0.15')
    
    # URDF file path - using original wheelchair model
    urdf_file = PathJoinSubstitution([
        pkg_wheelchair_desc,
        'urdf',
        'wheelchair.urdf.xacro'  # Original wheelchair model
    ])
    
    # Set plugin path for Gazebo
    set_plugin_path = SetEnvironmentVariable(
        'GZ_SIM_SYSTEM_PLUGIN_PATH',
        '/opt/ros/humble/lib'
    )
    
    # Launch Gazebo with warehouse world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': world_file
        }.items()
    )
    
    # Joint State Publisher - publishes joint states from URDF for robot_state_publisher
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time}
        ]
    )
    
    # Robot State Publisher - publishes TF transforms from URDF
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
        ]
    )
    
    # Static TF transforms - bridge Gazebo's prefixed frame names to URDF frame names
    static_tf_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_lidar',
        arguments=[
            '0', '0', '0',  # x, y, z (no offset - same frame)
            '0', '0', '0',  # roll, pitch, yaw (no rotation)
            'base_laser',  # parent frame (from TF tree)
            'autonomous_wheelchair/chassis/lidar'  # child frame (from Gazebo messages)
        ],
        output='screen'
    )
    
    static_tf_depth = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_depth',
        arguments=[
            '0', '0', '0',  # x, y, z (no offset - same frame)
            '0', '0', '0',  # roll, pitch, yaw (no rotation)
            'camera_link',  # parent frame (from TF tree)
            'autonomous_wheelchair/chassis/depth_camera'  # child frame (from Gazebo messages)
        ],
        output='screen'
    )
    
    # Spawn robot in Gazebo (delayed to ensure world is loaded)
    spawn_entity_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                name='spawn_wheelchair',
                arguments=[
                    '-world', 'world_demo',
                    '-entity', 'wheelchair',
                    '-topic', 'robot_description',
                    '-x', x_pose,
                    '-y', y_pose,
                    '-z', z_pose,
                    '-allow_renaming', 'false'
                ],
                output='screen'
            )
        ]
    )
    
    # Bridge for ROS 2 <-> Gazebo communication
    bridge_node = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='gz_bridge',
                arguments=[
                    '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                    'model/wheelchair/joint_state@sensor_msgs/msg/JointState[gz.msgs.ModelJointState',
                    'cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                    'odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                    '/wheelchair/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                    '/wheelchair/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
                    '/wheelchair/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
                    '/wheelchair/camera/depth@sensor_msgs/msg/Image[gz.msgs.Image',
                    '/wheelchair/camera/depth/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
                ],
                remappings=[
                    ('model/wheelchair/joint_state', '/joint_states'),
                    ('cmd_vel', '/cmd_vel'),
                    ('odom', '/odom'),
                    ('/wheelchair/scan', '/scan'),
                    ('/wheelchair/imu', '/imu_data'),
                    ('/wheelchair/camera/image_raw', '/camera/image'),
                    ('/wheelchair/camera/depth', '/camera/depth_image'),
                    ('/wheelchair/camera/depth/points', '/camera/points'),
                ],
                output='screen'
            )
        ]
    )
    
    # Note: Using direct Gazebo diff drive plugin (ros2_control disabled)
    # To re-enable ros2_control, see wheelchair.gazebo.xacro and controllers.yaml
    
    return LaunchDescription([
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
            default_value='0.15',
            description='Initial z position of the robot (wheels touch ground)'
        ),
        set_plugin_path,
        gz_sim,
        joint_state_publisher_node,  # CRITICAL: Provides joint states for robot_state_publisher
        robot_state_publisher_node,
        static_tf_lidar,  # Bridge Gazebo's prefixed frame name to TF tree frame
        static_tf_depth,  # Bridge Gazebo's prefixed frame name to TF tree frame
        spawn_entity_node,
        bridge_node,
    ])

