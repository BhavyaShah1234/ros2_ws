#!/usr/bin/env python3

import os
import random
import math
import uuid
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import rclpy
from rclpy.node import Node as RCLNode
from ros_gz_interfaces.srv import SpawnEntity


def spawn_blocks_function(context, *args, **kwargs):
    """Function to spawn blocks with access to launch context"""
    
    num_blocks = int(LaunchConfiguration('num_blocks').perform(context))
    spawn_radius = float(LaunchConfiguration('spawn_radius').perform(context))
    spawn_height = float(LaunchConfiguration('spawn_height').perform(context))
    
    # Available block types
    colors = ['red', 'green', 'blue']
    shapes = ['cylinder', 'cuboid', 'sphere', 'cone']
    
    # Create spawner node
    spawner_node = Node(
        package='simple_pick_and_place_gazebo',
        executable='block_spawner_node.py',
        name='block_spawner',
        parameters=[{
            'num_blocks': num_blocks,
            'spawn_radius': spawn_radius,
            'spawn_height': spawn_height,
            'colors': colors,
            'shapes': shapes,
        }],
        output='screen',
    )
    
    return [spawner_node]


def generate_launch_description():
    
    # Declare launch arguments
    num_blocks_arg = DeclareLaunchArgument(
        'num_blocks',
        default_value='10',
        description='Number of blocks to spawn'
    )
    
    spawn_radius_arg = DeclareLaunchArgument(
        'spawn_radius',
        default_value='0.4',
        description='Radius around robot base (x=0, y=0) to spawn blocks'
    )
    
    spawn_height_arg = DeclareLaunchArgument(
        'spawn_height',
        default_value='0.85',
        description='Height (z coordinate) at which to spawn blocks'
    )
    
    return LaunchDescription([
        num_blocks_arg,
        spawn_radius_arg,
        spawn_height_arg,
        OpaqueFunction(function=spawn_blocks_function),
    ])
