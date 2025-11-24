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
    z_pose = LaunchConfiguration('z_pose', default='0.1')
    
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
    
    # Joint State Publisher
    # Publishes joint states for robot_state_publisher
    # CRITICAL: robot_state_publisher needs /joint_states for continuous joints (wheels)
    # If Gazebo doesn't publish joint_states, this node will provide default values
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'source_list': ['/joint_states']}  # Try to use Gazebo's joint_states if available
        ]
    )
    
    # Robot State Publisher
    # Publishes TF transforms for all links and joints from URDF
    # Requires /joint_states topic for continuous joints (wheels)
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
        # Removed remappings - they were causing issues
        # robot_state_publisher should publish to standard /tf and /tf_static topics
    )
    
    # Static TF transforms to bridge Gazebo's auto-prefixed frame names
    # Gazebo prefixes sensor frames with model name (autonomous_wheelchair/chassis/...)
    # but TF tree uses simple URDF link names (base_laser, camera_link)
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
    
    # Spawn robot in Gazebo (with delay to ensure world is loaded)
    # NOTE: Always stop simulation before restarting to prevent duplicates
    # Use ./STOP_ALL.sh before launching to clean up existing robots
    spawn_entity_node = TimerAction(
        period=5.0,  # Wait 5 seconds for world to fully load
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                name='spawn_wheelchair',
                arguments=[
                    '-world', 'world_demo',
                    '-entity', 'wheelchair',  # Match working model name exactly
                    '-topic', 'robot_description',
                    '-x', x_pose,
                    '-y', y_pose,
                    '-z', '2.0',  # Match working model height (z=2.0)
                    '-allow_renaming', 'false'  # Prevent automatic renaming (wheelchair_0, etc.)
                ],
                output='screen'
            )
        ]
    )
    
    # Bridge for ROS 2 <-> Gazebo communication
    # Simplified: Direct diff drive plugin handles cmd_vel and odometry
    # Using TimerAction to delay bridge startup until Gazebo is ready
    bridge_node = TimerAction(
        period=6.0,  # Wait 6 seconds for Gazebo to initialize
        actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='gz_bridge',
                arguments=[
                    # ==================== CLOCK (CRITICAL - One-way: Gazebo -> ROS) ====================
                    # Bridge simulation clock - REQUIRED for use_sim_time=true
                    # Without this, ROS Time = 0.00 and TF won't publish
                    '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                    
                    # ==================== JOINT STATES (CRITICAL - One-way: Gazebo -> ROS) ====================
                    # Bridge joint states - REQUIRED for robot_state_publisher to publish wheel transforms
                    # Continuous joints (wheels) need joint positions from Gazebo
                    # Note: Gazebo publishes joint states to model/wheelchair/joint_state topic
                    'model/wheelchair/joint_state@sensor_msgs/msg/JointState[gz.msgs.ModelJointState',
                    
                    # ==================== MOVEMENT TOPICS (Bidirectional) ====================
                    # Bridge cmd_vel: ROS <-> Gazebo
                    # Syntax: TOPIC@ROS_MSG_TYPE@GAZEBO_MSG_TYPE
                    # @ means bidirectional (both directions)
                    # Note: Bridge creates the SAME topic name on both ROS and Gazebo sides
                    # We bridge model/wheelchair/cmd_vel, then remap ROS side to /cmd_vel
                    'model/wheelchair/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                    # Bridge odometry: ROS <-> Gazebo
                    'model/wheelchair/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                    
                    # ==================== SENSOR TOPICS (One-way: Gazebo -> ROS) ====================
                    # Syntax: ROS_TOPIC@ROS_MSG_TYPE[GAZEBO_MSG_TYPE
                    # [ means one-way (Gazebo to ROS only)
                    # LiDAR: Try multiple possible topic paths
                    '/wheelchair/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                    # IMU: Try multiple possible topic paths
                    '/wheelchair/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
                    # RGB Camera
                    '/wheelchair/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
                    # Depth Camera
                    '/wheelchair/camera/depth@sensor_msgs/msg/Image[gz.msgs.Image',
                    # Depth Point Cloud
                    '/wheelchair/camera/depth/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
                ],
                remappings=[
                    # Remap topic names to standard ROS topic names
                    # Format: (from_topic, to_topic)
                    # Clock - bridge publishes to /clock (standard ROS topic)
                    # Joint states - bridge publishes to model/wheelchair/joint_state, remap to /joint_states
                    ('model/wheelchair/joint_state', '/joint_states'),
                    # Movement topics - bridge creates 'model/wheelchair/cmd_vel', remap to '/cmd_vel'
                    ('model/wheelchair/cmd_vel', '/cmd_vel'),
                    ('model/wheelchair/odometry', '/odom'),
                    # Sensor topics
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
    
    # ==================== ROS 2 CONTROL DISABLED ====================
    # Using direct Gazebo diff drive plugin instead (like ros2-wheelchair-main)
    # This eliminates:
    # - Controller manager initialization delays
    # - Controller activation timing issues
    # - Topic relay complexity
    # - Hardware interface claiming problems
    #
    # The direct plugin approach:
    # - Works immediately (no delays needed)
    # - Simpler (fewer components)
    # - More reliable (no activation failures)
    # - Faster startup (no controller manager)
    #
    # If you need ros2_control features, you can re-enable by:
    # 1. Commenting out the diff drive plugin in wheelchair.gazebo.xacro
    # 2. Uncommenting the ros2_control plugin
    # 3. Uncommenting the controller_manager_node, load_and_activate_controllers, and cmd_vel_relay below
    #
    # controller_manager_node = TimerAction(...)  # DISABLED
    # load_and_activate_controllers = TimerAction(...)  # DISABLED
    # cmd_vel_relay = TimerAction(...)  # DISABLED
    
    # Teleop Keyboard - Control wheelchair with keyboard
    # NOTE: Teleop needs interactive terminal, so run manually in separate terminal:
    # ros2 run teleop_twist_keyboard teleop_twist_keyboard
    # Removed from launch file because it fails in non-interactive context
    
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
            default_value='0.1',
            description='Initial z position of the robot'
        ),
        set_plugin_path,
        gz_sim,
        joint_state_publisher_node,  # CRITICAL: Provides joint states for robot_state_publisher
        robot_state_publisher_node,
        static_tf_lidar,  # Bridge Gazebo's prefixed frame name to TF tree frame
        static_tf_depth,  # Bridge Gazebo's prefixed frame name to TF tree frame
        spawn_entity_node,
        bridge_node,  # Simplified bridge - direct diff drive plugin handles cmd_vel/odom
        # ros2_control components removed - using direct Gazebo plugin instead
        # controller_manager_node,  # DISABLED
        # load_and_activate_controllers,  # DISABLED
        # cmd_vel_relay,  # DISABLED
        # teleop_keyboard_node removed - run manually: ros2 run teleop_twist_keyboard teleop_twist_keyboard
    ])

