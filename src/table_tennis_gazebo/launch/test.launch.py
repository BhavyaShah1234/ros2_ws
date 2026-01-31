#!/usr/bin/env python3
"""
Test launch file for single Franka robot with ros2_control in empty world.
Includes Rviz for visualization. Robot is controlled via ros2_control.

To move the robot, send trajectory commands to arm_controller.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    """Generate launch description with single robot in empty world"""
    
    # Package directories
    pkg_gazebo = get_package_share_directory('table_tennis_gazebo')
    pkg_description = get_package_share_directory('table_tennis_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_franka_description = get_package_share_directory('franka_description')
    
    # File paths
    world_file = os.path.join(pkg_gazebo, 'worlds', 'empty.world')
    robot_urdf_path = os.path.join(pkg_description, 'urdf', 'robots', 'franka_with_paddle.urdf.xacro')
    models_path = os.path.join(pkg_gazebo, 'models')
    rviz_config = os.path.join(pkg_gazebo, 'rviz', 'single_robot.rviz')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Set Gazebo resource path
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=f'{models_path}:{pkg_franka_description}:{pkg_description}:{os.environ.get("GZ_SIM_RESOURCE_PATH", "")}'
    )
    
    # Process URDF
    robot_description = xacro.process_file(
        robot_urdf_path,
        mappings={
            'robot_type': 'fr3',
            'hand': 'false',
            'ros2_control': 'true',
            'gazebo': 'true',
            'use_fake_hardware': 'false',
        }
    ).toxml()
    
    # Launch Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'{world_file} -r'}.items(),
    )
    
    # Unified bridge using YAML config for clock and camera topics
    bridge_config_file = os.path.join(
        get_package_share_directory('table_tennis_gazebo'),
        'config',
        'ros_gz_bridge.yaml'
    )
    
    unified_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'config_file': bridge_config_file
        }]
    )
    
    delayed_unified_bridge = TimerAction(
        period=6.0,
        actions=[unified_bridge]
    )
    
    # Robot state publisher - required for TF tree
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_description}
        ],
    )
    
    delayed_robot_state_pub = TimerAction(
        period=3.0,
        actions=[robot_state_pub]
    )
    
    # Spawn robot at origin
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'test_robot',
            '-topic', '/robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.0',
        ],
        output='screen'
    )
    
    delayed_spawn = TimerAction(
        period=5.0,
        actions=[spawn_robot]
    )
    
    # Rviz for visualization
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    
    delayed_rviz = TimerAction(
        period=15.0,
        actions=[rviz]
    )
    
    # Load and activate controllers
    load_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    load_arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '--controller-manager', '/controller_manager'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    delayed_load_joint_state = TimerAction(
        period=7.0,
        actions=[load_joint_state_broadcaster]
    )
    
    delayed_load_arm = TimerAction(
        period=8.0,
        actions=[load_arm_controller]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        set_gz_resource_path,
        gazebo,
        delayed_unified_bridge,
        delayed_robot_state_pub,
        delayed_spawn,
        delayed_rviz,
        delayed_load_joint_state,
        delayed_load_arm,
    ])
