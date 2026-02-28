# Table Tennis Gazebo

## ✅ FULLY WORKING - Dual Robot Control Verified

Gazebo simulation for table tennis playing robots using ROS 2 Jazzy with active ros2_control for both robots.

## Status
All controllers successfully active for both red and green robots:
- `/red/arm_controller` (joint_trajectory_controller)
- `/red/joint_state_broadcaster`
- `/green/arm_controller` (joint_trajectory_controller)
- `/green/joint_state_broadcaster`

Joint states publishing at 100-300 Hz for both robots.

## Critical Setup Requirements

**This simulation requires `gz_ros2_control` built from source to avoid ABI mismatch:**

```bash
cd ~/Projects/ros2_ws/src/
git clone https://github.com/ros-controls/gz_ros2_control.git -b jazzy
cd ..
colcon build --packages-up-to gz_ros2_control table_tennis_gazebo
source install/setup.bash
```

**Reason:** System package `/opt/ros/jazzy/lib/libgz_hardware_plugins.so` has ABI incompatibility with workspace-built `hardware_interface`. Building from source resolves this.

## Features

- Dual Franka FR3 robots with paddle end effectors (namespaced: `/red/`, `/green/`)
- Physics-accurate table tennis ball (40mm, 2.7g, restitution 0.89)
- Standard table tennis table (2.74m × 1.525m × 0.76m)
- 4 RealSense D435 RGBD cameras:
  - 2 arm-mounted (one per robot)
  - 1 overhead camera
  - 1 side camera viewing along net
- Service-based ball spawning at any position
- ros2_control integration with trajectory controllers

## Building

```bash
# Navigate to workspace root
cd ~/Projects/ros2_ws

# Build this package and its dependencies
colcon build --packages-select table_tennis_description table_tennis_gazebo

# Source the workspace
source install/setup.bash
```

## Quick Start

### 1. Launch the Simulation

```bash
# Kill any existing processes
pkill -9 -f "gz|rviz"

# Launch simulation with RViz visualization
ros2 launch table_tennis_gazebo simulation.launch.py
```

## Verifying Controllers

**Check red robot controllers:**
```bash
ros2 control list_controllers --controller-manager /red/controller_manager
```

**Check green robot controllers:**
```bash
ros2 control list_controllers --controller-manager /green/controller_manager
```

Expected output for each:
```
arm_controller          joint_trajectory_controller/JointTrajectoryController  active
joint_state_broadcaster joint_state_broadcaster/JointStateBroadcaster          active
```

**Monitor joint states:**
```bash
# Red robot
ros2 topic hz /red/joint_states --window 5

# Green robot
ros2 topic hz /green/joint_states --window 5
```

## Spawning a Ball

```bash
# Move red robot
ros2 action send_goal /red/arm_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory "{
    trajectory: {
      joint_names: [fr3_joint1, fr3_joint2, fr3_joint3, fr3_joint4, fr3_joint5, fr3_joint6, fr3_joint7],
      points: [
        { positions: [0.0, -0.3, 0.0, -1.5, 0.0, 1.2, 0.785], time_from_start: { sec: 2 } }
      ]
    }
  }"

# Move green robot
ros2 action send_goal /green/arm_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory "{
    trajectory: {
      joint_names: [fr3_joint1, fr3_joint2, fr3_joint3, fr3_joint4, fr3_joint5, fr3_joint6, fr3_joint7],
      points: [
        { positions: [0.0, 0.3, 0.0, -1.2, 0.0, 1.5, -0.785], time_from_start: { sec: 2 } }
      ]
    }
  }"
```

## Package Structure

```
table_tennis_gazebo/
├── action/
│   └── SpawnBall.action          # Ball spawner action definition
├── config/
│   ├── red_controllers.yaml      # Red robot ros2_control config
│   └── green_controllers.yaml    # Green robot ros2_control config
├── launch/
│   ├── simulation.launch.py      # Main simulation launcher
│   ├── bridge.launch.py          # ROS-Gazebo bridges
│   ├── ball_spawn.launch.py      # Ball spawner node
│   └── test.launch.py            # Testing launcher
├── models/
│   ├── table/                    # Table tennis table model
│   ├── ball/                     # Ball model with physics
│   └── cameras/                  # Camera models
├── rviz/
│   ├── red_robot.rviz            # Red robot visualization config
│   └── green_robot.rviz          # Green robot visualization config
├── scripts/
│   ├── ball_spawner_node.py      # Ball spawner action server (Python)
│   └── ball_spawn_client.py      # Example ball spawn client
├── src/
│   └── ball_spawner_node.cpp     # Ball spawner action server (C++)
├── srv/
│   └── SpawnBall.srv             # Legacy service definition
├── worlds/
│   └── arena.world               # Gazebo world file
├── CMakeLists.txt
├── package.xml
└── README.md
```

## Topics and Services

### Ball Spawner
- **Action**: `/spawn_ball` (table_tennis_gazebo/action/SpawnBall)
  - Goal: Spawn position (x, y, z)
  - Feedback: Current position and status
  - Result: Final position, time alive, destruction reason

### Ball Status
- `/ball/status` (std_msgs/String) - "alive" or "dead"
- `/ball/pose` (geometry_msgs/Pose) - Current 3D position

### Robot Controllers (per robot: red/green)
- `/red/arm_controller/follow_joint_trajectory` (control_msgs/action/FollowJointTrajectory)
- `/green/arm_controller/follow_joint_trajectory` (control_msgs/action/FollowJointTrajectory)
- `/red/joint_states` (sensor_msgs/JointState)
- `/green/joint_states` (sensor_msgs/JointState)

### Camera Topics (per camera)

**Red Robot Arm Camera:**
- `/red/arm_cam/color/image_raw`
- `/red/arm_cam/depth/image_rect_raw`
- `/red/arm_cam/color/camera_info`
- `/red/arm_cam/depth/color/points`

**Green Robot Arm Camera:**
- `/green/arm_cam/color/image_raw`
- `/green/arm_cam/depth/image_rect_raw`
- `/green/arm_cam/color/camera_info`
- `/green/arm_cam/depth/color/points`

**Overhead Camera:**
- `/overhead_cam/color/image_raw`
- `/overhead_cam/depth/image_rect_raw`
- `/overhead_cam/color/camera_info`
- `/overhead_cam/depth/color/points`

**Side Camera:**
- `/side_cam/color/image_raw`
- `/side_cam/depth/image_rect_raw`
- `/side_cam/color/camera_info`

## Robot Control

Both robots are controlled via ros2_control trajectory controllers:
- **Red robot:** `/red/arm_controller` accepts `trajectory_msgs/msg/JointTrajectory` on topic `/red/arm_controller/joint_trajectory`
- **Green robot:** `/green/arm_controller` accepts `trajectory_msgs/msg/JointTrajectory` on topic `/green/arm_controller/joint_trajectory`

**Example command (red robot):**
```bash
ros2 topic pub --once /red/arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
  "{joint_names: [fr3_joint1, fr3_joint2, fr3_joint3, fr3_joint4, fr3_joint5, fr3_joint6, fr3_joint7], \
   points: [{positions: [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785], time_from_start: {sec: 2}}]}"
```

## Package Structure

```
table_tennis_gazebo/
├── config/          # Controller configurations
├── launch/          # Launch files
├── models/          # Gazebo models (ball, table, cameras)
├── src/             # Source code (ball spawner node)
├── srv/             # Service definitions
└── worlds/          # Gazebo world files
```
