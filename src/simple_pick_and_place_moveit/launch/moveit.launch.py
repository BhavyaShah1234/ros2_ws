#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    
    # Get package directories
    pkg_moveit = get_package_share_directory('simple_pick_and_place_moveit')
    pkg_description = get_package_share_directory('simple_pick_and_place_description')
    
    # Robot description
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
        ' ros2_control:=true',
        ' gazebo:=false',
    ])
    
    robot_description = {'robot_description': robot_description_content}
    
    # Robot description semantic (SRDF)
    robot_description_semantic_path = os.path.join(
        pkg_moveit, 'config', 'franka.srdf'
    )
    with open(robot_description_semantic_path, 'r') as f:
        robot_description_semantic = {'robot_description_semantic': f.read()}
    
    # Kinematics
    kinematics_yaml = os.path.join(pkg_moveit, 'config', 'kinematics.yaml')
    
    # Joint limits    
    joint_limits_yaml = os.path.join(pkg_moveit, 'config', 'joint_limits.yaml')
    
    # Planning parameters
    ompl_planning_yaml = {
        'planning_plugin': 'ompl_interface/OMPLPlanner',
        'request_adapters': 'default_planning_request_adapters/AddTimeOptimalParameterization '
                           'default_planning_request_adapters/ResolveConstraintFrames '
                           'default_planning_request_adapters/FixWorkspaceBounds '
                           'default_planning_request_adapters/FixStartStateBounds '
                           'default_planning_request_adapters/FixStartStateCollision '
                           'default_planning_request_adapters/FixStartStatePathConstraints',
        'planning_scene_monitor_options': {
            'name': 'planning_scene_monitor',
            'robot_description': 'robot_description',
            'joint_state_topic': '/joint_states',
            'attached_collision_object_topic': '/moveit_attached_collision_object',
            'publish_planning_scene_topic': '/moveit_published_planning_scene',
            'monitored_planning_scene_topic': '/moveit_monitored_planning_scene',
            'wait_for_initial_state_timeout': 10.0,
        },
    }
    
    # Trajectory execution
    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }
    
    # Controller manager
    moveit_controllers_path = os.path.join(
        pkg_moveit, 'config', 'moveit_controllers.yaml'
    )
    
    # Planning scene monitor parameters
    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }
    
    # Move group node
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            joint_limits_yaml,
            ompl_planning_yaml,
            trajectory_execution,
            moveit_controllers_path,
            planning_scene_monitor_parameters,
            {'use_sim_time': True},
        ],
    )
    
    # RViz with MoveIt plugin
    rviz_config = os.path.join(pkg_moveit, 'config', 'moveit.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
        parameters=[
            robot_description,
            robot_description_semantic,
            {'use_sim_time': True},
        ],
        output='screen',
    )
    
    return LaunchDescription([
        move_group_node,
        rviz_node,
    ])
