#!/usr/bin/env python3
"""
Launch file for table tennis Gazebo simulation with dual Franka FR3 robots.

This launch file properly sequences:
1. Start Gazebo with world
2. Start robot state publishers  
3. Spawn robots in Gazebo
4. Load and activate controllers
5. Launch ROS-Gazebo bridges

Based on best practices from:
https://automaticaddison.com/how-to-simulate-a-robotic-arm-in-gazebo-ros-2-jazzy/
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    """Generate launch description with proper sequencing"""
    
    # Package directories
    pkg_gazebo = get_package_share_directory('table_tennis_gazebo')
    pkg_description = get_package_share_directory('table_tennis_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_franka_description = get_package_share_directory('franka_description')
    
    # File paths
    world_file = os.path.join(pkg_gazebo, 'worlds', 'arena.world')
    robot_urdf_path = os.path.join(pkg_description, 'urdf', 'robots', 'franka_with_paddle.urdf.xacro')
    models_path = os.path.join(pkg_gazebo, 'models')
    rviz_config_red = os.path.join(pkg_gazebo, 'rviz', 'red_robot.rviz')
    rviz_config_green = os.path.join(pkg_gazebo, 'rviz', 'green_robot.rviz')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Set Gazebo resource path to include Franka meshes and table tennis description
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=f'{models_path}:{pkg_franka_description}:{pkg_description}:{os.environ.get("GZ_SIM_RESOURCE_PATH", "")}'
    )
    
    # Process URDF for red robot (ros2_control enabled)
    robot_description_red = xacro.process_file(
        robot_urdf_path,
        mappings={
            'robot_type': 'fr3',
            'hand': 'false',
            'ros2_control': 'true',
            'gazebo': 'true',
            'use_fake_hardware': 'false',
            'robot_namespace': 'red',
        }
    ).toxml()
    
    # Process URDF for green robot (ros2_control enabled)
    robot_description_green = xacro.process_file(
        robot_urdf_path,
        mappings={
            'robot_type': 'fr3',
            'hand': 'false',
            'ros2_control': 'true',
            'gazebo': 'true',
            'use_fake_hardware': 'false',
            'robot_namespace': 'green',
        }
    ).toxml()
    
    # 1. Launch Gazebo (first thing that starts)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'{world_file} -r'}.items(),
    )
    
    # 2. Robot state publishers (start after small delay to let Gazebo initialize)
    robot_state_pub_red = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace='red',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_description_red},
            {'frame_prefix': 'red/'},  # Avoid TF frame collisions
        ],
    )
    
    robot_state_pub_green = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace='green',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_description_green},
            {'frame_prefix': 'green/'},  # Avoid TF frame collisions
        ],
    )
    
    # Delay robot state publishers slightly to ensure Gazebo is ready
    delayed_robot_state_pub_red = TimerAction(
        period=3.0,
        actions=[robot_state_pub_red]
    )
    
    delayed_robot_state_pub_green = TimerAction(
        period=3.0,
        actions=[robot_state_pub_green]
    )
    
    # 3. Spawn robots (after robot_state_publisher starts)
    spawn_red = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_red_robot',
        output='screen',
        arguments=[
            '-name', 'red_robot',
            '-topic', '/red/robot_description',
            '-x', '-1.57',
            '-y', '0.0',
            '-z', '0.76',
        ],
    )
    
    spawn_green = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_green_robot',
        output='screen',
        arguments=[
            '-name', 'green_robot',
            '-topic', '/green/robot_description',
            '-x', '1.57',
            '-y', '0.0',
            '-z', '0.76',
            '-Y', '3.14159',
        ],
    )
    
    # Delay spawn to happen after robot_state_publisher
    delayed_spawn_red = TimerAction(
        period=5.0,
        actions=[spawn_red]
    )
    
    delayed_spawn_green = TimerAction(
    delayed_spawn_green = TimerAction(
        period=5.5,
        actions=[spawn_green]
    )
    
    # Static transforms to create separate world frames for each robot
    # This creates red/world and green/world frames linked to the main world
    static_tf_red = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_red_world',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'red/world']
    )
    
    static_tf_green = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_green_world',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'green/world']
    )
    
    # 4. Bridges (start after a delay)
    bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, 'launch', 'bridge.launch.py')
        ),
    )
    
    delayed_bridge = TimerAction(
        period=4.0,
        actions=[bridge]
    )
    
    # Ball spawner
    ball_spawner = Node(
        package='table_tennis_gazebo',
        executable='ball_spawner_node',
        name='ball_spawner',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )
    
    delayed_ball_spawner = TimerAction(
        period=8.0,
        actions=[ball_spawner]
    )
    
    # 5. Load controllers for red robot
    load_joint_state_broadcaster_red = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', 
                   '--controller-manager', '/red/controller_manager'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    load_arm_controller_red = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', 
                   '--controller-manager', '/red/controller_manager'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    delayed_load_joint_state_broadcaster_red = TimerAction(
        period=10.0,
        actions=[load_joint_state_broadcaster_red]
    )
    
    delayed_load_arm_controller_red = TimerAction(
        period=11.0,
        actions=[load_arm_controller_red]
    )
    
    # 6. Load controllers for green robot
    load_joint_state_broadcaster_green = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', 
                   '--controller-manager', '/green/controller_manager'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    load_arm_controller_green = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', 
                   '--controller-manager', '/green/controller_manager'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    delayed_load_joint_state_broadcaster_green = TimerAction(
        period=10.5,
        actions=[load_joint_state_broadcaster_green]
    )
    
    delayed_load_arm_controller_green = TimerAction(
        period=11.5,
        actions=[load_arm_controller_green]
    )
    
    # 7. Rviz for visualization (two separate windows)
    rviz_red = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_red',
        namespace='red',
        arguments=['-d', rviz_config_red],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    
    rviz_green = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_green',
        namespace='green',
        arguments=['-d', rviz_config_green],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    
    delayed_rviz_red = TimerAction(
        period=15.0,
        actions=[rviz_red]
    )
    
    delayed_rviz_green = TimerAction(
        period=15.5,
        actions=[rviz_green]
    )
    
    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        
        # Set environment
        set_gz_resource_path,
        
        # 1. Start Gazebo first
        gazebo,
        
        # Static TF publishers for separate world frames
        static_tf_red,
        static_tf_green,
        
        # 2. Start robot state publishers (delayed)
        delayed_robot_state_pub_red,
        delayed_robot_state_pub_green,
        
        # 3. Spawn robots (delayed more)
        delayed_spawn_red,
        delayed_spawn_green,
        
        # 4. Bridges (delayed)
        delayed_bridge,
        
        # 5-6. Load controllers (delayed after spawn)
        delayed_load_joint_state_broadcaster_red,
        delayed_load_arm_controller_red,
        delayed_load_joint_state_broadcaster_green,
        delayed_load_arm_controller_green,
        
        # 7. Ball spawner (delayed)
        delayed_ball_spawner,
        
        # 8. Rviz windows (delayed) - one for each robot
        delayed_rviz_red,
        delayed_rviz_green,
    ])


