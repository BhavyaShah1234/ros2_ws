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

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Highest point of the FR3 arm (the wrist cluster) in its default Gazebo spawn
# pose, measured via forward kinematics of franka_arm.ros2_control.xacro's
# baked-in initial joint values. Kept in sync with the comment and <pose> in
# models/overhead_camera/model.sdf.
ROBOT_HEIGHT_M = 0.697
CAMERA_CLEARANCE_M = 0.10
CAMERA_HEIGHT_M = ROBOT_HEIGHT_M + CAMERA_CLEARANCE_M


def generate_launch_description():
    robot_type_name = 'robot_type'
    load_gripper_name = 'load_gripper'
    franka_hand_name = 'franka_hand'
    namespace_name = 'namespace'

    robot_type_launch_argument = DeclareLaunchArgument(
        robot_type_name,
        default_value='fr3',
        description='Available values: fr3, fp3 and fer')
    load_gripper_launch_argument = DeclareLaunchArgument(
        load_gripper_name,
        default_value='false',
        description='true/false for activating the gripper')
    franka_hand_launch_argument = DeclareLaunchArgument(
        franka_hand_name,
        default_value='franka_hand',
        description='Default value: franka_hand')
    namespace_launch_argument = DeclareLaunchArgument(
        namespace_name,
        default_value='',
        description='Namespace for the robot. If not set, the robot will be launched in the '
                     'root namespace.')

    franka_gazebo_bringup_dir = get_package_share_directory('franka_gazebo_bringup')
    franka_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(franka_gazebo_bringup_dir, 'launch',
                         'gazebo_joint_position_controller_example.launch.py')),
        launch_arguments={
            'robot_type': LaunchConfiguration(robot_type_name),
            'load_gripper': LaunchConfiguration(load_gripper_name),
            'franka_hand': LaunchConfiguration(franka_hand_name),
            'namespace': LaunchConfiguration(namespace_name),
        }.items(),
    )

    overhead_camera_model = os.path.join(
        get_package_share_directory('franka_overhead_camera'),
        'models', 'overhead_camera', 'model.sdf')

    spawn_overhead_camera = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', overhead_camera_model,
            '-name', 'overhead_camera',
            '-x', '0', '-y', '0', '-z', str(CAMERA_HEIGHT_M),
            '-R', '0', '-P', '1.5707963', '-Y', '0',
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

    return LaunchDescription([
        robot_type_launch_argument,
        load_gripper_launch_argument,
        franka_hand_launch_argument,
        namespace_launch_argument,
        franka_gazebo,
        spawn_overhead_camera,
        overhead_camera_bridge,
    ])
