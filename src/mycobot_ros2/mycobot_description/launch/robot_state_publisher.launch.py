import os
import launch as l
import pathlib as p
import launch_ros as lr
from launch_ros.parameter_descriptions import ParameterValue

def process_ros2_controllers_config(context):
    prefix = l.substitutions.LaunchConfiguration('prefix').perform(context)
    flange_link = l.substitutions.LaunchConfiguration('flange_link').perform(context)
    robot_name = l.substitutions.LaunchConfiguration('robot_name').perform(context)

    home = str(p.Path.home())

    src_config_path = os.path.join(home, 'ros2_ws/src/mycobot_ros2/mycobot_moveit_config/config', robot_name)
    install_config_path = os.path.join(home, 'ros2_ws/install/mycobot_moveit_config/share/mycobot_moveit_config/config', robot_name)
    template_path = os.path.join(src_config_path, 'ros2_controllers_template.yaml')

    with open(template_path, 'r', encoding='utf-8') as f:
        template_content = f.read()
        f.close()

    processed_content = template_content.replace('${prefix}', prefix)
    processed_content = processed_content.replace('${flange_link}', flange_link)

    for config_path in [src_config_path, install_config_path]:
        os.makedirs(config_path, exist_ok=True)
        output_path = os.path.join(config_path, 'ros2_controllers.yaml')
        with open(output_path, 'w', encoding='utf-8') as file:
            file.write(processed_content)
    return []

ARGUMENTS = [
    l.actions.DeclareLaunchArgument('robot_name', default_value='mycobot_280', description='Name of the robot'),
    l.actions.DeclareLaunchArgument('prefix', default_value='', description='Prefix for robot joints and links'),
    l.actions.DeclareLaunchArgument('add_world', default_value='true', choices=['true', 'false'], description='Whether to add world link'),
    l.actions.DeclareLaunchArgument('base_link', default_value='base_link', description='Name of the base link'),
    l.actions.DeclareLaunchArgument('base_type', default_value='g_shape', description='Type of base'),
    l.actions.DeclareLaunchArgument('flange_link', default_value='link6_flange', description='Name of the flange link'),
    l.actions.DeclareLaunchArgument('gripper_type', default_value='adaptive_gripper', description='Type of the gripper'),
    l.actions.DeclareLaunchArgument('use_camera', default_value='false', choices=['true', 'false'], description='Whether to use RGBD Gazebo plugin for point cloud'),
    l.actions.DeclareLaunchArgument('use_gazebo', default_value='false', choices=['true', 'false'], description='Whether to use Gazebo simulation'),
    l.actions.DeclareLaunchArgument('use_gripper', default_value='true', choices=['true', 'false'], description='Whether to attach a gripper'),
]

def generate_launch_description():
    urdf_package = 'mycobot_description'
    urdf_filename = 'mycobot_280.urdf.xacro'
    rviz_config_filename = 'mycobot_280_description.rviz'

    package_share_description = lr.substitutions.FindPackageShare(urdf_package)
    default_urdf_model_path = l.substitutions.PathJoinSubstitution([package_share_description, 'urdf', 'robots', urdf_filename])
    default_rviz_config_path = l.substitutions.PathJoinSubstitution([package_share_description, 'rviz', rviz_config_filename])

    joint_state_publisher_gui = l.substitutions.LaunchConfiguration('joint_state_publisher_gui')
    rviz_config_file = l.substitutions.LaunchConfiguration('rviz_config_file')
    urdf_model = l.substitutions.LaunchConfiguration('urdf_model')
    use_joint_state_publisher = l.substitutions.LaunchConfiguration('use_joint_state_publisher')
    use_rviz = l.substitutions.LaunchConfiguration('use_rviz')
    use_sim_time = l.substitutions.LaunchConfiguration('use_sim_time')

    declare_jsp_gui_cmd = l.actions.DeclareLaunchArgument(name='joint_state_publisher_gui', default_value='true', choices=['true', 'false'], description='Flag to enable joint_state_publisher_gui')
    declare_rviz_config_file = l.actions.DeclareLaunchArgument(name='rviz_config_file', default_value=default_rviz_config_path, description='Full path to the RVIZ config file to use')
    declare_urdf_model_path = l.actions.DeclareLaunchArgument(name='urdf_model_path', default_value=default_urdf_model_path, description='Absolute path to robot urdf file')
    declare_use_joint_state_publisher = l.actions.DeclareLaunchArgument(name='use_joint_state_publisher', default_value='false', choices=['true', 'false'], description='Enable the joint state publisher')
    declare_use_rviz = l.actions.DeclareLaunchArgument(name='use_rviz', default_value='true', choices=['true', 'false'], description='Whether to start RVIZ')
    declare_use_sim_time = l.actions.DeclareLaunchArgument(name='use_sim_time', default_value='false', description='Use simulation (Gazebo) clock if true')

    robot_description_content = ParameterValue(
        value=l.substitutions.Command(command=[
            'xacro', ' ', urdf_model, ' ', 'robot_name:=', l.substitutions.LaunchConfiguration('robot_name'), ' ', 'prefix:=', l.substitutions.LaunchConfiguration('prefix'), ' ', 'add_world:=', l.substitutions.LaunchConfiguration('add_world'), ' ', 'base_link:=', l.substitutions.LaunchConfiguration('base_link'), ' ', 'base_type:=', l.substitutions.LaunchConfiguration('base_type'), ' ', 'flange_link:=', l.substitutions.LaunchConfiguration('flange_link'), ' ', 'gripper_type:=', l.substitutions.LaunchConfiguration('gripper_type'), ' ', 'use_camera:=', l.substitutions.LaunchConfiguration('use_camera'), ' ', 'use_gazebo:=', l.substitutions.LaunchConfiguration('use_gazebo'), ' ', 'use_gripper:=', l.substitutions.LaunchConfiguration('use_gripper')
        ]), value_type=str
    )

    start_robot_state_publisher = lr.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_description_content}]
    )
    start_joint_state_publisher = lr.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=l.conditions.IfCondition(use_joint_state_publisher)
    )
    start_joint_state_publisher_gui = lr.actions.Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=l.conditions.IfCondition(joint_state_publisher_gui)
    )
    start_rviz_cmd = lr.actions.Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=l.conditions.IfCondition(use_rviz),
    )

    launch_description = l.LaunchDescription(initial_entities=ARGUMENTS)
    launch_description.add_action(l.actions.OpaqueFunction(function=process_ros2_controllers_config))
    launch_description.add_action(declare_jsp_gui_cmd)
    launch_description.add_action(declare_rviz_config_file)
    launch_description.add_action(declare_urdf_model_path)
    launch_description.add_action(declare_use_joint_state_publisher)
    launch_description.add_action(declare_use_rviz)
    launch_description.add_action(declare_use_sim_time)
    launch_description.add_action(start_joint_state_publisher)
    launch_description.add_action(start_joint_state_publisher_gui)
    launch_description.add_action(start_robot_state_publisher)
    launch_description.add_action(start_rviz_cmd)
    return launch_description
