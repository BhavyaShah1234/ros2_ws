#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    
    # Declare launch arguments
    robot_type_arg = DeclareLaunchArgument(
        'robot_type',
        default_value='fr3',
        description='Type of Franka robot (fr3, fer, fp3, etc.)'
    )
    
    hand_arg = DeclareLaunchArgument(
        'hand',
        default_value='true',
        description='Whether to attach the gripper/hand'
    )
    
    use_fake_hardware_arg = DeclareLaunchArgument(
        'use_fake_hardware',
        default_value='false',
        description='Use fake hardware for testing'
    )
    
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='',
        description='IP address of the real robot'
    )
    
    arm_prefix_arg = DeclareLaunchArgument(
        'arm_prefix',
        default_value='',
        description='Prefix for robot links and joints'
    )
    
    # Get URDF via xacro
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([
            FindPackageShare('simple_pick_and_place_description'),
            'urdf',
            'robot',
            'franka_pick_and_place.urdf.xacro'
        ]),
        ' robot_type:=', LaunchConfiguration('robot_type'),
        ' hand:=', LaunchConfiguration('hand'),
        ' use_fake_hardware:=', LaunchConfiguration('use_fake_hardware'),
        ' robot_ip:=', LaunchConfiguration('robot_ip'),
        ' arm_prefix:=', LaunchConfiguration('arm_prefix'),
    ])
    
    robot_description = {'robot_description': robot_description_content}
    
    # Robot state publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description],
    )
    
    return LaunchDescription([
        robot_type_arg,
        hand_arg,
        use_fake_hardware_arg,
        robot_ip_arg,
        arm_prefix_arg,
        robot_state_publisher_node,
    ])
