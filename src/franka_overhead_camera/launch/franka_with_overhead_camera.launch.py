# Copyright (c) 2026 Bhavya Shah
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# This mirrors franka_gazebo_bringup's gazebo_joint_position_controller_example
# launch file, with one change: it launches Gazebo with a custom world
# (worlds/empty_with_sensors.sdf) instead of the stock ros_gz_sim empty.sdf,
# because that stock world does not load the gz::sim::systems::Sensors
# system plugin -- without it, camera/depth sensors advertise topics but
# never actually publish frames. It also spawns the fixed overhead RGB-D
# camera and bridges its topics, spawns the calibration floor markers and
# maze, and attaches a laser rangefinder to the robot's flange.

import os
import xacro
import xml.dom.minidom as minidom
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def get_overhead_camera_pose():
    """Read the camera's spawn pose from config/overhead_camera.yaml.

    See that file for what each field means and why editing it (rather than
    this launch file, or the <pose> in model.sdf) is the right place to
    adjust the camera's position/orientation.
    """
    config_path = os.path.join(
        get_package_share_directory('franka_overhead_camera'),
        'config', 'overhead_camera.yaml')
    with open(config_path, 'r') as f:
        cfg = yaml.safe_load(f)['overhead_camera']
    return {
        'x': cfg['x'],
        'y': cfg['y'],
        'z': cfg['robot_height_m'] + cfg['clearance_m'],
        'roll': cfg['roll'],
        'pitch': cfg['pitch'],
        'yaw': cfg['yaw'],
    }


def get_calibration_markers():
    """Read the calibration marker list from config/calibration_markers.yaml.

    See that file for what each field means and how to add/move/recolor
    markers.
    """
    config_path = os.path.join(
        get_package_share_directory('franka_overhead_camera'),
        'config', 'calibration_markers.yaml')
    with open(config_path, 'r') as f:
        return yaml.safe_load(f)['calibration_markers']


def make_marker_sdf(marker):
    """Build a minimal static colored-box model.sdf, inline, for one marker."""
    size_x, size_y, size_z = marker['size']
    r, g, b = marker['color']
    return f'''<?xml version="1.0" ?>
<sdf version="1.9">
  <model name="{marker['name']}">
    <static>true</static>
    <link name="link">
      <visual name="visual">
        <geometry>
          <box>
            <size>{size_x} {size_y} {size_z}</size>
          </box>
        </geometry>
        <material>
          <ambient>{r} {g} {b} 1</ambient>
          <diffuse>{r} {g} {b} 1</diffuse>
        </material>
      </visual>
      <collision name="collision">
        <geometry>
          <box>
            <size>{size_x} {size_y} {size_z}</size>
          </box>
        </geometry>
      </collision>
    </link>
  </model>
</sdf>'''


def make_maze_sdf():
    """Build the maze as one static model, from config/maze_layout.yaml's
    wall list mapped onto the square formed by config/calibration_markers.yaml's
    3 markers (see both files for details). Returns None if fewer than 3
    markers are configured (nothing to map the grid onto).
    """
    share_dir = get_package_share_directory('franka_overhead_camera')

    with open(os.path.join(share_dir, 'config', 'maze_layout.yaml'), 'r') as f:
        maze_cfg = yaml.safe_load(f)['maze']
    with open(os.path.join(share_dir, 'config', 'calibration_markers.yaml'), 'r') as f:
        markers = yaml.safe_load(f)['calibration_markers']

    if len(markers) < 3:
        return None

    xs = [m['x'] for m in markers]
    ys = [m['y'] for m in markers]
    x_min, x_max = min(xs), max(xs)
    y_min, y_max = min(ys), max(ys)

    n = maze_cfg['cells_per_side']
    thickness = maze_cfg['wall_thickness_m']
    height = maze_cfg['wall_height_m']
    cell_size_x = (x_max - x_min) / n
    cell_size_y = (y_max - y_min) / n

    def to_world(gx, gy):
        return (x_min + gx * cell_size_x, y_min + gy * cell_size_y)

    wall_elements = []
    for (gx1, gy1), (gx2, gy2) in maze_cfg['segments_grid_units']:
        wx1, wy1 = to_world(gx1, gy1)
        wx2, wy2 = to_world(gx2, gy2)
        cx, cy = (wx1 + wx2) / 2.0, (wy1 + wy2) / 2.0
        length = ((wx2 - wx1) ** 2 + (wy2 - wy1) ** 2) ** 0.5
        # Segments are axis-aligned in grid space and stay axis-aligned in
        # world space, so orientation is a size choice, not a rotation.
        if abs(wy2 - wy1) < 1e-9:
            size_x, size_y = length, thickness
        else:
            size_x, size_y = thickness, length
        wall_elements.append(f'''
      <visual name="wall_visual_{len(wall_elements)}">
        <pose>{cx} {cy} {height / 2.0} 0 0 0</pose>
        <geometry>
          <box>
            <size>{size_x} {size_y} {height}</size>
          </box>
        </geometry>
        <material>
          <ambient>0.6 0.6 0.6 1</ambient>
          <diffuse>0.6 0.6 0.6 1</diffuse>
        </material>
      </visual>
      <collision name="wall_collision_{len(wall_elements)}">
        <pose>{cx} {cy} {height / 2.0} 0 0 0</pose>
        <geometry>
          <box>
            <size>{size_x} {size_y} {height}</size>
          </box>
        </geometry>
      </collision>''')

    return f'''<?xml version="1.0" ?>
<sdf version="1.9">
  <model name="calibration_maze">
    <static>true</static>
    <link name="link">{''.join(wall_elements)}
    </link>
  </model>
</sdf>'''


def add_laser_to_urdf(doc, parent_link):
    """Attach a laser rangefinder to `parent_link` (the robot's flange, e.g.
    fr3_link8) directly in the xacro-generated URDF DOM: a small red
    cylinder (the visible laser module) on a fixed joint, plus a
    single-ray gpu_lidar sensor via a <gazebo> extension tag, pointing
    along the flange's local +Z axis -- verified against
    robots/fr3/kinematics.yaml's joint8 entry (xyz 0 0 0.107, zero
    rotation), which means link8 shares link7's orientation and its
    local +Z is the flange's outward/tool-pointing axis. A gz-sim sensor's
    forward axis is local +X (as with the overhead camera), so the sensor
    pose below pitches -90 degrees to align +X with the link's +Z.
    """
    extra_xml = f'''<root>
      <link name="laser_link">
        <visual>
          <geometry>
            <cylinder length="0.03" radius="0.005"/>
          </geometry>
          <material name="laser_red">
            <color rgba="1 0 0 1"/>
          </material>
        </visual>
      </link>
      <joint name="laser_joint" type="fixed">
        <parent link="{parent_link}"/>
        <child link="laser_link"/>
        <origin xyz="0 0 0.02" rpy="0 0 0"/>
      </joint>
      <gazebo reference="laser_link">
        <sensor name="end_effector_laser" type="gpu_lidar">
          <pose>0 0 0 0 -1.5707963 0</pose>
          <topic>end_effector_laser</topic>
          <update_rate>30</update_rate>
          <lidar>
            <scan>
              <horizontal>
                <samples>1</samples>
                <resolution>1</resolution>
                <min_angle>0</min_angle>
                <max_angle>0</max_angle>
              </horizontal>
              <vertical>
                <samples>1</samples>
                <resolution>1</resolution>
                <min_angle>0</min_angle>
                <max_angle>0</max_angle>
              </vertical>
            </scan>
            <range>
              <min>0.02</min>
              <max>5.0</max>
              <resolution>0.001</resolution>
            </range>
          </lidar>
          <alwaysOn>1</alwaysOn>
          <visualize>true</visualize>
        </sensor>
      </gazebo>
    </root>'''
    extra_doc = minidom.parseString(extra_xml)
    robot_elem = doc.getElementsByTagName('robot')[0]
    for child in list(extra_doc.documentElement.childNodes):
        if child.nodeType == child.ELEMENT_NODE:
            robot_elem.appendChild(doc.importNode(child, deep=True))
    return doc


def get_robot_description(context: LaunchContext, robot_type, load_gripper, franka_hand):
    robot_type_str = context.perform_substitution(robot_type)
    load_gripper_str = context.perform_substitution(load_gripper)
    franka_hand_str = context.perform_substitution(franka_hand)

    franka_xacro_file = os.path.join(
        get_package_share_directory('franka_description'),
        'robots',
        robot_type_str,
        robot_type_str + '.urdf.xacro'
    )

    robot_description_config = xacro.process_file(
        franka_xacro_file,
        mappings={
            'robot_type': robot_type_str,
            'hand': load_gripper_str,
            'ros2_control': 'true',
            'gazebo': 'true',
            'ee_id': franka_hand_str
        }
    )
    add_laser_to_urdf(robot_description_config, parent_link=f'{robot_type_str}_link8')
    robot_description = {'robot_description': robot_description_config.toxml()}

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            robot_description,
        ]
    )

    return [robot_state_publisher]


def generate_launch_description():
    load_gripper_name = 'load_gripper'
    franka_hand_name = 'franka_hand'
    robot_type_name = 'robot_type'
    namespace_name = 'namespace'

    load_gripper = LaunchConfiguration(load_gripper_name)
    franka_hand = LaunchConfiguration(franka_hand_name)
    robot_type = LaunchConfiguration(robot_type_name)
    namespace = LaunchConfiguration(namespace_name)

    load_gripper_launch_argument = DeclareLaunchArgument(
        load_gripper_name,
        default_value='false',
        description='true/false for activating the gripper')
    franka_hand_launch_argument = DeclareLaunchArgument(
        franka_hand_name,
        default_value='franka_hand',
        description='Default value: franka_hand')
    robot_type_launch_argument = DeclareLaunchArgument(
        robot_type_name,
        default_value='fr3',
        description='Available values: fr3, fp3 and fer')
    namespace_launch_argument = DeclareLaunchArgument(
        namespace_name,
        default_value='',
        description='Namespace for the robot. If not set, the robot will be launched in the '
                     'root namespace.')

    robot_state_publisher = OpaqueFunction(
        function=get_robot_description,
        args=[robot_type, load_gripper, franka_hand])

    os.environ['GZ_SIM_RESOURCE_PATH'] = os.path.dirname(
        get_package_share_directory('franka_description'))
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    world_file = os.path.join(
        get_package_share_directory('franka_overhead_camera'),
        'worlds', 'empty_with_sensors.sdf')
    gazebo_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'{world_file} -r', }.items(),
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        namespace=namespace,
        arguments=['-topic', '/robot_description'],
        output='screen',
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    rviz_file = os.path.join(get_package_share_directory('franka_description'), 'rviz',
                              'visualize_franka.rviz')
    rviz = Node(package='rviz2',
                executable='rviz2',
                name='rviz2',
                namespace=namespace,
                arguments=['--display-config', rviz_file, '-f', 'world'],
                )

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_joint_position_example_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_position_example_controller'],
        output='screen'
    )

    overhead_camera_model = os.path.join(
        get_package_share_directory('franka_overhead_camera'),
        'models', 'overhead_camera', 'model.sdf')

    overhead_camera_pose = get_overhead_camera_pose()

    spawn_overhead_camera = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', overhead_camera_model,
            '-name', 'overhead_camera',
            '-x', str(overhead_camera_pose['x']),
            '-y', str(overhead_camera_pose['y']),
            '-z', str(overhead_camera_pose['z']),
            '-R', str(overhead_camera_pose['roll']),
            '-P', str(overhead_camera_pose['pitch']),
            '-Y', str(overhead_camera_pose['yaw']),
        ],
        output='screen',
    )

    overhead_camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/overhead_camera/image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/overhead_camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            '/overhead_camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/overhead_camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
        ],
        output='screen',
    )

    spawn_calibration_markers = [
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-string', make_marker_sdf(marker),
                '-name', marker['name'],
                '-x', str(marker['x']),
                '-y', str(marker['y']),
                '-z', str(marker['z']),
            ],
            output='screen',
        )
        for marker in get_calibration_markers()
    ]

    laser_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/end_effector_laser@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
        ],
        output='screen',
    )

    maze_sdf = make_maze_sdf()
    spawn_maze = [
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-string', maze_sdf, '-name', 'calibration_maze'],
            output='screen',
        )
    ] if maze_sdf is not None else []

    return LaunchDescription([
        load_gripper_launch_argument,
        franka_hand_launch_argument,
        robot_type_launch_argument,
        namespace_launch_argument,
        gazebo_world,
        robot_state_publisher,
        rviz,
        spawn,
        bridge,
        spawn_overhead_camera,
        overhead_camera_bridge,
        *spawn_calibration_markers,
        *spawn_maze,
        laser_bridge,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn,
                on_exit=[load_joint_state_broadcaster],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_joint_position_example_controller],
            )
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            namespace=namespace,
            parameters=[
                {'source_list': ['joint_states'],
                 'rate': 30}],
        ),
    ])
