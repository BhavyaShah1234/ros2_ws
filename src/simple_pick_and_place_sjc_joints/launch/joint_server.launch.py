#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    
    joint_server = Node(
        package='simple_pick_and_place_sjc_joints',
        executable='joint_action_server.py',
        name='joint_action_server',
        output='screen',
    )
    
    return LaunchDescription([
        joint_server,
    ])
