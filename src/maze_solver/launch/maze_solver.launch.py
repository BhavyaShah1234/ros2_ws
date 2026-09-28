from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='maze_solver', executable='maze_digitizer', output='screen'),
        Node(package='maze_solver', executable='path_planner', output='screen'),
        Node(package='maze_solver', executable='motion_executor', output='screen'),
    ])
