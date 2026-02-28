#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    
    highlevel_server = Node(
        package='simple_pick_and_place_sjc_highlevel',
        executable='highlevel_action_server.py',
        name='highlevel_action_server',
        output='screen',
    )
    
    return LaunchDescription([
        highlevel_server,
    ])
