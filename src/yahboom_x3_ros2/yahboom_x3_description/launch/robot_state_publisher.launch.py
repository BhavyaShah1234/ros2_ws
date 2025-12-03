import os
import launch as l
import pathlib as p
import launch_ros as lr
from launch_ros.parameter_descriptions import ParameterValue

def process_ros2_controllers_config(context):
    # Get the configuration values
    prefix = l.substitutions.LaunchConfiguration('prefix').perform(context)
    robot_name = l.substitutions.LaunchConfiguration('robot_name').perform(context)
    enable_odom_tf = l.substitutions.LaunchConfiguration('enable_odom_tf').perform(context)

    home = str(p.Path.home())

    # Define both source and install paths
    src_config_path = os.path.join(home, 'ros2_ws/src/yahboom_rosmaster/yahboom_rosmaster_description/config', robot_name)
    install_config_path = os.path.join(home, 'ros2_ws/install/yahboom_rosmaster_description/share/yahboom_rosmaster_description/config', robot_name)

    # Read from source template
    template_path = os.path.join(src_config_path, 'ros2_controllers_template.yaml')
    with open(template_path, 'r', encoding='utf-8') as f:
        template_content = f.read()
        f.close()

    # Create processed content (leaving template untouched)
    processed_content = template_content.replace('${prefix}', prefix)
    processed_content = processed_content.replace('enable_odom_tf: true', f'enable_odom_tf: {enable_odom_tf}')

    # Write processed content to both source and install directories
    for config_path in [src_config_path, install_config_path]:
        os.makedirs(config_path, exist_ok=True)
        output_path = os.path.join(config_path, 'ros2_controllers.yaml')
        with open(output_path, 'w', encoding='utf-8') as f:
            f.write(processed_content)
            f.close()
    return []

# Define the arguments for the XACRO file
ARGUMENTS = [
    l.actions.DeclareLaunchArgument('robot_name', default_value='yahboom_x3', description='Name of the robot'),
    l.actions.DeclareLaunchArgument('prefix', default_value='', description='Prefix for robot joints and links'),
    l.actions.DeclareLaunchArgument('use_gazebo', default_value='false', choices=['true', 'false'], description='Whether to use Gazebo simulation'),
    l.actions.DeclareLaunchArgument('enable_odom_tf', default_value='true', choices=['true', 'false'], description='Enable odometry transform broadcasting via ROS 2 Control'),
]

def generate_launch_description():
    # Define filenames
    urdf_package = 'yahboom_x3_description'
    urdf_filename = 'yahboom_x3.urdf.xacro'
    rviz_config_filename = 'yahboom_x3_description.rviz'

    # Set paths to important files
    pkg_share_description = lr.substitutions.FindPackageShare(urdf_package)
    default_urdf_model_path = lr.substitutions.PathJoinSubstitution([pkg_share_description, 'urdf', 'robots', urdf_filename])
    default_rviz_config_path = lr.substitutions.PathJoinSubstitution([pkg_share_description, 'rviz', rviz_config_filename])

    # Launch configuration variables
    jsp_gui = lr.substitutions.LaunchConfiguration('jsp_gui')
    rviz_config_file = lr.substitutions.LaunchConfiguration('rviz_config_file')
    urdf_model = lr.substitutions.LaunchConfiguration('urdf_model')
    use_jsp = lr.substitutions.LaunchConfiguration('use_jsp')
    use_rviz = lr.substitutions.LaunchConfiguration('use_rviz')
    use_sim_time = lr.substitutions.LaunchConfiguration('use_sim_time')

    # Declare the launch arguments
    declare_jsp_gui_cmd = l.actions.DeclareLaunchArgument(
        name='jsp_gui',
        default_value='true',
        choices=['true', 'false'],
        description='Flag to enable joint_state_publisher_gui')

    declare_rviz_config_file_cmd = l.actions.DeclareLaunchArgument(
        name='rviz_config_file',
        default_value=default_rviz_config_path,
        description='Full path to the RVIZ config file to use')

    declare_urdf_model_path_cmd = l.actions.DeclareLaunchArgument(
        name='urdf_model',
        default_value=default_urdf_model_path,
        description='Absolute path to robot urdf file')
    
    declare_use_jsp_cmd = l.actions.DeclareLaunchArgument(
        name='use_jsp',
        default_value='false',
        choices=['true', 'false'],
        description='Enable the joint state publisher')

    declare_use_rviz_cmd = l.actions.DeclareLaunchArgument(
        name='use_rviz',
        default_value='true',
        description='Whether to start RVIZ')

    declare_use_sim_time_cmd = l.actions.DeclareLaunchArgument(
        name='use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    robot_description_content = ParameterValue(
        value=l.substitutions.Command([
            'xacro', ' ', urdf_model, ' ',
            'robot_name:=', l.substitutions.LaunchConfiguration('robot_name'), ' ',
            'prefix:=', l.substitutions.LaunchConfiguration('prefix'), ' ',
            'use_gazebo:=', l.substitutions.LaunchConfiguration('use_gazebo'),
        ]), value_type=str
    )

    # Subscribe to the joint states of the robot, and publish the 3D pose of each link.
    start_robot_state_publisher_cmd = lr.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_description_content}]
    )

    # Publish the joint state values for the non-fixed joints in the URDF file.
    start_joint_state_publisher_cmd = lr.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=l.conditions.IfCondition(use_jsp))

    # Depending on gui parameter, either launch joint_state_publisher or joint_state_publisher_gui
    start_joint_state_publisher_gui_cmd = lr.actions.Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=l.conditions.IfCondition(jsp_gui))

    # Launch RViz
    start_rviz_cmd = lr.actions.Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=l.conditions.IfCondition(use_rviz),
    )

    # Create the launch description and populate
    ld = l.LaunchDescription(ARGUMENTS)

    # Process the controller configuration before starting nodes
    ld.add_action(l.actions.OpaqueFunction(function=process_ros2_controllers_config))

    # Declare the launch options
    ld.add_action(declare_jsp_gui_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_urdf_model_path_cmd)
    ld.add_action(declare_use_jsp_cmd) 
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_sim_time_cmd)

    # Add any actions
    ld.add_action(start_joint_state_publisher_cmd)
    ld.add_action(start_joint_state_publisher_gui_cmd)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_rviz_cmd)

    return ld
