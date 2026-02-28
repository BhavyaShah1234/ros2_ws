#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    
    cartesian_server = Node(
        package='simple_pick_and_place_sjc_cartesian',
        executable='cartesian_action_server.py',
        name='cartesian_action_server',
        output='screen',
    )
    
    return LaunchDescription([
        cartesian_server,
    ])
