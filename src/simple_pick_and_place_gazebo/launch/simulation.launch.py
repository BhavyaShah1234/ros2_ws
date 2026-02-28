#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, RegisterEventHandler, ExecuteProcess, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
import xacro


def generate_launch_description():
    
    # Get package directories
    pkg_gazebo = get_package_share_directory('simple_pick_and_place_gazebo')
    pkg_description = get_package_share_directory('simple_pick_and_place_description')
    pkg_franka_description = get_package_share_directory('franka_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    # Set Gazebo resource path to include our models and Franka meshes
    # For model:// protocol to work, we need parent directory of franka_description
    models_path = os.path.join(pkg_gazebo, 'models')
    franka_resource = os.path.dirname(pkg_franka_description)  # Go up one level to 'share'
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=f'{models_path}:{franka_resource}:{pkg_description}:{os.environ.get("GZ_SIM_RESOURCE_PATH", "")}'
    )
    
    # Gazebo launch using ros_gz_sim (with GUI and auto-run)
    world_file = os.path.join(pkg_gazebo, 'worlds', 'pick_and_place.world')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'{world_file} -r -v 4'}.items(),
    )
    
    # Robot description - process URDF directly with xacro
    robot_urdf_path = os.path.join(
        pkg_description,
        'urdf',
        'robot',
        'franka_pick_and_place.urdf.xacro'
    )
    
    robot_description_content = xacro.process_file(
        robot_urdf_path,
        mappings={
            'robot_type': 'fr3',
            'hand': 'true',
            'use_fake_hardware': 'false',  # Use Gazebo hardware, not mock
            'ros2_control': 'true',
            'gazebo': 'true',  # Load Gazebo plugin (gz_ros2_control)
        }
    ).toxml()
    
    robot_description = {'robot_description': robot_description_content}
    
    # Robot state publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[
            robot_description,
            {'use_sim_time': True},  # Use boolean True directly
        ],
    )
    
    # Static transform: Connect Gazebo's world frame to robot base
    # Required because gazebo="true" disables URDF world link creation
    static_tf_world_to_robot = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_world_to_robot_base',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'fr3_link0'],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )
    
    # Static transform: Overhead camera frame (1.5m in front, 1.5m high, 45° down angle)
    static_tf_world_to_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_world_to_camera',
        arguments=['1.5', '0', '1.5', '3.14159', '-0.785398', '0', 'world', 'overhead_camera/link/camera'],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )
    
    # Spawn robot on ground
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', '/robot_description',
            '-name', 'franka',
            '-allow_renaming', 'true',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.0',  # Ground level for floor-mounted robot
        ],
        output='screen',
    )
    
    # CRITICAL: Clock bridge - Gazebo simulation time to ROS /clock topic
    bridge_clock = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )
    
    # ROS-Gazebo bridge for camera image (one-way: Gazebo -> ROS)
    bridge_camera = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='camera_bridge',
        arguments=[
            '/overhead_camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
        ],
        remappings=[
            ('/overhead_camera/image', '/overhead_camera/image_raw'),
        ],
        parameters=[
            {'use_sim_time': True},
        ],
        output='screen',
    )
    
    # ROS-Gazebo bridge for camera_info (one-way: Gazebo -> ROS)
    bridge_camera_info = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='camera_info_bridge',
        arguments=[
            '/overhead_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        ],
        remappings=[
            ('/overhead_camera/camera_info', '/overhead_camera/camera_info'),
        ],
        parameters=[
            {'use_sim_time': True},
        ],
        output='screen',
    )
    
    # ROS-Gazebo bridge for depth image (one-way: Gazebo -> ROS)
    bridge_depth = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='depth_bridge',
        arguments=[
            '/overhead_camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
        ],
        remappings=[
            ('/overhead_camera/depth_image', '/overhead_camera/depth/image_raw'),
        ],
        parameters=[
            {'use_sim_time': True},
        ],
        output='screen',
    )
    
    # NOTE: DO NOT start ros2_control_node when using Gazebo!
    # The gz_ros2_control Gazebo plugin provides the controller_manager internally.
    # Controllers will connect to the controller_manager running inside Gazebo.
    
    # Load joint state broadcaster using spawner (with automatic activation)
    load_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    
    # Load arm controller using spawner
    load_arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    
    # Load gripper controller using spawner
    load_gripper_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    
    # RViz
    rviz_config = os.path.join(pkg_description, 'rviz', 'pick_and_place.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[
            {'use_sim_time': True},
        ],
        output='screen',
    )
    
    # Build launch description with proper timing
    return LaunchDescription([
        use_sim_time_arg,
        set_gz_resource_path,
        
        # t=0s: Start Gazebo (gz_sim.launch.py handles clock bridge automatically)
        gazebo,
        
        # t=1s: CRITICAL - Start clock bridge immediately after Gazebo
        TimerAction(
            period=1.0,
            actions=[bridge_clock]
        ),
        
        # t=2s: Start robot state publisher and static TF (Gazebo plugin provides controller_manager)
        TimerAction(
            period=2.0,
            actions=[robot_state_publisher_node, static_tf_world_to_robot, static_tf_world_to_camera]
        ),
        
        # t=3s: Start camera bridges
        TimerAction(
            period=3.0,
            actions=[bridge_camera, bridge_camera_info, bridge_depth]
        ),
        
        # t=4s: Spawn robot
        TimerAction(
            period=4.0,
            actions=[spawn_robot]
        ),
        
        # t=10s: Load joint state broadcaster first (critical for TF and /joint_states)
        TimerAction(
            period=10.0,
            actions=[load_joint_state_broadcaster]
        ),
        
        # t=11s: Load arm controller (after joint_state_broadcaster is active)
        TimerAction(
            period=11.0,
            actions=[load_arm_controller]
        ),
        
        # t=12s: Load gripper controller
        TimerAction(
            period=12.0,
            actions=[load_gripper_controller]
        ),
        
        # t=15s: Start RViz (after all controllers loaded to avoid resource contention)
        TimerAction(
            period=15.0,
            actions=[rviz_node]
        ),
    ])
