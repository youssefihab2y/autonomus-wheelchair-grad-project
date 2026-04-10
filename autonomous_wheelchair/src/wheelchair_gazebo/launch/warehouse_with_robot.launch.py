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
    # Spawn height chosen so that, with the current URDF geometry, all wheels
    # just touch the ground at t=0 (no initial “drop” or penetration).
    # See wheel_height calculations in wheelchair.urdf.xacro:
    #   z_spawn = 2*wheel_radius_back - base_height_offset = 0.09 m
    z_pose = LaunchConfiguration('z_pose', default='0.09')
    
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
    # NOTE: If Gazebo joint states aren't available, this provides fallback joint states
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
    # Uses joint states from either Gazebo (via bridge) or joint_state_publisher
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
    
    # Static TF: Add base_link alias for ROS 2 conventions (base_link -> base_uplayer_link)
    # Many ROS 2 tools expect base_link as the base frame
    static_tf_base_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_link',
        arguments=[
            '0', '0', '0',  # x, y, z (no offset)
            '0', '0', '0',  # roll, pitch, yaw (no rotation)
            'base_uplayer_link',  # parent frame
            'base_link'  # child frame (alias)
        ],
        output='screen'
    )
    
    # Static TF: Bridge Gazebo's prefixed sensor frame names to URDF frame names
    # Gazebo Fortress automatically prefixes sensor frames with model name
    static_tf_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_lidar',
        arguments=[
            '0', '0', '0',  # x, y, z (no offset - same frame)
            '0', '0', '0',  # roll, pitch, yaw (no rotation)
            'base_laser',  # parent frame (URDF name)
            'autonomous_wheelchair/chassis/lidar'  # child frame (Gazebo prefixed name)
        ],
        output='screen'
    )
    
    static_tf_depth_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_depth_camera',
        arguments=[
            '0', '0', '0',  # x, y, z (no offset - same frame)
            '0', '0', '0',  # roll, pitch, yaw (no rotation)
            'camera_link',  # parent frame (URDF name)
            'autonomous_wheelchair/chassis/depth_camera'  # child frame (Gazebo prefixed name)
        ],
        output='screen'
    )
    
    # Spawn robot in Gazebo (delayed to ensure world is loaded)
    # Note: If duplicates appear, ensure Gazebo is fully closed (pkill -9 -f "gz") before relaunching
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
                    '-allow_renaming', 'false'  # Will fail if entity exists - ensures no duplicates
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
                    '/scan_raw@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                    '/wheelchair/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
                    '/wheelchair/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
                    '/wheelchair/camera/depth@sensor_msgs/msg/Image[gz.msgs.Image',
                    '/wheelchair/camera/depth/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
                ],
                remappings=[
                    ('model/wheelchair/joint_state', '/joint_states'),
                    ('cmd_vel', '/cmd_vel'),
                    ('odom', '/odom_raw'),  # Bridge to /odom_raw (with prefixed frames)
                    ('/wheelchair/scan', '/scan_raw'),  # Bridge to /scan_raw (with Gazebo frame names)
                    ('/wheelchair/imu', '/imu_data'),
                    ('/wheelchair/camera/image_raw', '/camera/image'),
                    ('/wheelchair/camera/depth', '/camera/depth_image'),
                    ('/wheelchair/camera/depth/points', '/camera/points'),
                ],
                output='screen'
            )
        ]
    )
    
    # Fix odometry frame names (remove Gazebo prefixes)
    odom_frame_fixer_node = TimerAction(
        period=7.0,
        actions=[
            Node(
                package='wheelchair_gazebo',
                executable='fix_odom_frames.py',
                name='odom_frame_fixer',
                remappings=[
                    ('/odom', '/odom_raw'),  # Subscribe to raw odom with prefixed frames
                    ('/odom_fixed', '/odom'),  # Publish fixed odom to standard /odom topic
                ],
                output='screen'
            )
        ]
    )
    
    # Convert odometry to TF transform (prevents RViz queue overflow)
    odom_to_tf_node = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='wheelchair_gazebo',
                executable='odom_to_tf.py',
                name='odom_to_tf',
                output='screen'
            )
        ]
    )
    
    # Filter lidar scan to remove robot's own body (back bar, seat, etc.)
    lidar_filter_node = TimerAction(
        period=9.0,
        actions=[
            Node(
                package='wheelchair_gazebo',
                executable='filter_lidar_self.py',
                name='lidar_self_filter',
                remappings=[
                    ('/scan', '/scan_raw'),  # Subscribe to raw scan from bridge
                    ('/scan_filtered', '/scan'),  # Publish filtered scan to /scan (replaces original)
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
        default_value='0.09',
            description='Initial z position of the robot (wheels touch ground)'
        ),
        set_plugin_path,
        gz_sim,
        joint_state_publisher_node,  # Provides joint states for robot_state_publisher
        robot_state_publisher_node,  # Publishes TF from URDF using joint_states
        static_tf_base_link,  # Add base_link alias for ROS 2 conventions
        static_tf_lidar,  # Bridge Gazebo's prefixed lidar frame to URDF frame
        static_tf_depth_camera,  # Bridge Gazebo's prefixed depth camera frame to URDF frame
        spawn_entity_node,
        bridge_node,
        odom_frame_fixer_node,  # Fix odometry frame names (remove Gazebo prefixes)
        odom_to_tf_node,  # Convert odometry to TF (prevents RViz queue overflow)
        lidar_filter_node,  # Filter lidar to remove robot's own body (back bar, seat, etc.)
    ])

