#!/usr/bin/env python3
"""
Simple workaround launch for visualization-only mode.
Uses joint_state_publisher_gui instead of controllers for testing.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


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
   
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='false',
        description='Start joint_state_publisher_gui for manual control'
    )
    
    # Set Gazebo resource path
    models_path = os.path.join(pkg_gazebo, 'models')
    franka_resource = os.path.dirname(pkg_franka_description)
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=f'{models_path}:{franka_resource}:{pkg_description}:{os.environ.get("GZ_SIM_RESOURCE_PATH", "")}'
    )
    
    # Gazebo launch
    world_file = os.path.join(pkg_gazebo, 'worlds', 'pick_and_place.world')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'{world_file} -r'}.items(),
    )
    
    # Robot description WITHOUT ros2_control
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([
            FindPackageShare('simple_pick_and_place_description'),
            'urdf',
            'robot',
            'franka_pick_and_place.urdf.xacro'
        ]),
        ' robot_type:=fr3',
        ' hand:=true',
        ' use_fake_hardware:=true',
        ' ros2_control:=false',  # Disable ros2_control for viz-only mode
        ' gazebo:=false',
    ])
    
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}
    
    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[
            robot_description,
            {'use_sim_time': True},
        ],
    )
    
    # Joint state publisher (publishes fixed joint states)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[
            {'use_sim_time': True},
        ],
        condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui'))
    )
    
    # Joint state publisher GUI (manual control)
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        parameters=[
            {'use_sim_time': True},
        ],
        condition=launch.conditions.IfCondition(LaunchConfiguration('gui'))
    )
    
    # Spawn robot
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', '/robot_description',
            '-name', 'franka',
            '-allow_renaming', 'true',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.05',
        ],
        output='screen',
    )
    
    # Camera bridges
    bridge_camera = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/overhead_camera/image_raw'],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )
    
    bridge_camera_info = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/overhead_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        ],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )
    
    # RViz
    rviz_config = os.path.join(pkg_description, 'rviz', 'pick_and_place.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        gui_arg,
        set_gz_resource_path,
        
        gazebo,
        
        TimerAction(period=2.0, actions=[robot_state_publisher_node, joint_state_publisher]),
        TimerAction(period=3.0, actions=[bridge_camera, bridge_camera_info]),
        TimerAction(period=4.0, actions=[spawn_robot]),
        TimerAction(period=8.0, actions=[rviz_node]),
    ])
