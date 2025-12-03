from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare world argument
    world_file = LaunchConfiguration('world')
    
    # Get world file path
    world_path = PathJoinSubstitution([
        FindPackageShare('gazebo_worlds'),
        'worlds',
        world_file
    ])

    # Launch Gazebo with the specified world
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', world_path],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='empty.world',
            description='World file to load (e.g., empty.world, indoor.world, outdoor.world)'
        ),
        gazebo
    ])
