from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get package paths
    gazebo_ros_pkg = FindPackageShare('gazebo_ros')
    car_description_pkg = FindPackageShare('autonomous_car_description')
    
    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([gazebo_ros_pkg, 'launch', 'gazebo.launch.py'])
        ])
    )

    # URDF file path
    urdf_file = PathJoinSubstitution([
        car_description_pkg,
        'urdf',
        'simple_car.urdf'
    ])

    # Spawn robot
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'simple_car', '-file', urdf_file],
        output='screen'
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': Command(['cat ', urdf_file])
        }]
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity
    ])
