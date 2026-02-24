#!/usr/bin/env python3
"""
Launch file for ROS-Gazebo bridges for dual robot simulation.
Uses YAML config file for proper Gazebo Harmonic + ROS 2 Jazzy integration.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launch file for ROS-Gazebo bridges"""
    
    # Package directory
    pkg_gazebo = get_package_share_directory('table_tennis_gazebo')
    
    # Bridge config file
    bridge_config_file = os.path.join(
        pkg_gazebo,
        'config',
        'ros_gz_bridge_dual.yaml'
    )
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Unified bridge node for all topics
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
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        unified_bridge,
    ])
    
    return LaunchDescription([
        
        # Clock bridge
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='clock_bridge',
            output='screen',
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
            ]
        ),
        
        # Red robot arm camera - color image
        Node(
            package='ros_gz_image',
            executable='image_bridge',
            name='red_arm_cam_color_bridge',
            output='screen',
            arguments=['/red/panda_arm_cam/image'],
            remappings=[
                ('/red/panda_arm_cam/image', '/red/arm_cam/color/image_raw')
            ]
        ),
        
        # Red robot arm camera - depth image
        Node(
            package='ros_gz_image',
            executable='image_bridge',
            name='red_arm_cam_depth_bridge',
            output='screen',
            arguments=['/red/panda_arm_cam/depth_image'],
            remappings=[
                ('/red/panda_arm_cam/depth_image', '/red/arm_cam/depth/image_rect_raw')
            ]
        ),
        
        # Red robot arm camera - camera info
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='red_arm_cam_info_bridge',
            output='screen',
            arguments=[
                '/red/panda_arm_cam/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'
            ],
            remappings=[
                ('/red/panda_arm_cam/camera_info', '/red/arm_cam/color/camera_info')
            ]
        ),
        
        # Green robot arm camera - color image
        Node(
            package='ros_gz_image',
            executable='image_bridge',
            name='green_arm_cam_color_bridge',
            output='screen',
            arguments=['/green/panda_arm_cam/image'],
            remappings=[
                ('/green/panda_arm_cam/image', '/green/arm_cam/color/image_raw')
            ]
        ),
        
        # Green robot arm camera - depth image
        Node(
            package='ros_gz_image',
            executable='image_bridge',
            name='green_arm_cam_depth_bridge',
            output='screen',
            arguments=['/green/panda_arm_cam/depth_image'],
            remappings=[
                ('/green/panda_arm_cam/depth_image', '/green/arm_cam/depth/image_rect_raw')
            ]
        ),
        
        # Green robot arm camera - camera info
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='green_arm_cam_info_bridge',
            output='screen',
            arguments=[
                '/green/panda_arm_cam/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'
            ],
            remappings=[
                ('/green/panda_arm_cam/camera_info', '/green/arm_cam/color/camera_info')
            ]
        ),
        
        # Overhead camera - color image
        Node(
            package='ros_gz_image',
            executable='image_bridge',
            name='overhead_cam_color_bridge',
            output='screen',
            arguments=['/overhead_cam/image'],
            remappings=[
                ('/overhead_cam/image', '/overhead_cam/color/image_raw')
            ]
        ),
        
        # Overhead camera - depth image
        Node(
            package='ros_gz_image',
            executable='image_bridge',
            name='overhead_cam_depth_bridge',
            output='screen',
            arguments=['/overhead_cam/depth_image'],
            remappings=[
                ('/overhead_cam/depth_image', '/overhead_cam/depth/image_rect_raw')
            ]
        ),
        
        # Overhead camera - camera info
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='overhead_cam_info_bridge',
            output='screen',
            arguments=[
                '/overhead_cam/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'
            ],
            remappings=[
                ('/overhead_cam/camera_info', '/overhead_cam/color/camera_info')
            ]
        ),
        
        # Side camera - color image
        Node(
            package='ros_gz_image',
            executable='image_bridge',
            name='side_cam_color_bridge',
            output='screen',
            arguments=['/side_cam/image'],
            remappings=[
                ('/side_cam/image', '/side_cam/color/image_raw')
            ]
        ),
        
        # Side camera - depth image
        Node(
            package='ros_gz_image',
            executable='image_bridge',
            name='side_cam_depth_bridge',
            output='screen',
            arguments=['/side_cam/depth_image'],
            remappings=[
                ('/side_cam/depth_image', '/side_cam/depth/image_rect_raw')
            ]
        ),
        
        # Side camera - camera info
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='side_cam_info_bridge',
            output='screen',
            arguments=[
                '/side_cam/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'
            ],
            remappings=[
                ('/side_cam/camera_info', '/side_cam/color/camera_info')
            ]
        ),
        
    ])
