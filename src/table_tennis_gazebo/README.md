# Table Tennis Gazebo

A comprehensive Gazebo Harmonic simulation for table tennis playing robots using ROS 2 Jazzy with dual Franka FR3 robots.

## Overview

This package provides a realistic table tennis simulation environment with physics-based ball dynamics, multiple camera viewpoints, and full ros2_control integration for robot manipulation.

## Features

### Robots
- **Dual Franka FR3 robots** (red and green teams)
- **Custom paddle end effectors** for table tennis
- **Position-controlled joints** via ros2_control
- **Trajectory following** using `FollowJointTrajectory` action servers
- **Proper namespacing** for multi-robot scenarios

### Environment
- **Standard table tennis table**: 2.74m × 1.525m × 0.76m (ITTF regulation)
- **Physics-accurate ball**: 40mm diameter, 2.7g mass, 0.89 restitution coefficient
- **Arena world** with proper lighting and ground

### Sensors
- **4 RealSense D435 RGBD cameras**:
  - 2 arm-mounted cameras (one per robot)
  - 1 overhead camera (bird's eye view)
  - 1 side camera (viewing along the net)
- **All cameras publish**:
  - RGB images (`image_raw`)
  - Depth images (`depth/image_rect_raw`)
  - Camera info (`camera_info`)
  - Point clouds (`depth/color/points`)

### Ball Spawner
- **Action server interface** with real-time feedback
- **Automatic ball destruction** when:
  - Ball hits ground (z < 0.05m)
  - Ball falls below table (z < 0.65m)
  - Ball becomes stationary (velocity < 0.01 m/s for >2 seconds)
- **Physics-based tracking** with position and velocity monitoring
- **Single ball constraint** (only one ball active at a time)

## Prerequisites

- ROS 2 Jazzy
- Gazebo Harmonic (gz-sim 8)
- ros2_control packages
- Python 3.10+

See the [main workspace README](../../README.md) for detailed installation instructions.

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

# Wait ~25 seconds for all controllers to activate
```

The simulation includes:
- Gazebo Harmonic with the arena world
- Two Franka FR3 robots (red and green)
- RViz visualization for both robots
- Ball spawner action server
- All camera streams
- Controller manager and trajectory controllers

### 2. Verify Controllers

```bash
# Check active controllers
ros2 control list_controllers

# Expected output:
# red/arm_controller[joint_trajectory_controller/JointTrajectoryController] active
# green/arm_controller[joint_trajectory_controller/JointTrajectoryController] active
# red/joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
# green/joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
```

### 3. Spawn a Ball

```bash
# Spawn ball using action server
ros2 action send_goal /spawn_ball table_tennis_gazebo/action/SpawnBall \
  "{x: 0.3, y: 0.5, z: 3.0}" --feedback

# The ball will:
# 1. Spawn at the specified position
# 2. Fall due to gravity
# 3. Bounce on the table
# 4. Automatically be destroyed when stationary
```

### 4. Control Robots

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
- `/side_cam/depth/color/points`

## Advanced Usage

### Multi-Waypoint Trajectories

```bash
ros2 action send_goal /red/arm_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory "{
    trajectory: {
      joint_names: [fr3_joint1, fr3_joint2, fr3_joint3, fr3_joint4, fr3_joint5, fr3_joint6, fr3_joint7],
      points: [
        { positions: [0.0, -0.3, 0.0, -1.5, 0.0, 1.2, 0.785], time_from_start: { sec: 1 } },
        { positions: [0.3, -0.5, 0.2, -1.2, 0.0, 1.5, 0.0], time_from_start: { sec: 3 } },
        { positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], time_from_start: { sec: 5 } }
      ]
    }
  }"
```

### Monitoring Ball State

```bash
# Watch ball status
ros2 topic echo /ball/status

# Watch ball pose
ros2 topic echo /ball/pose

# Monitor action feedback
ros2 action send_goal /spawn_ball table_tennis_gazebo/action/SpawnBall \
  "{x: 0.0, y: 0.0, z: 3.0}" --feedback
```

### Viewing Camera Streams

```bash
# View red robot arm camera
ros2 run rqt_image_view rqt_image_view /red/arm_cam/color/image_raw

# View overhead camera
ros2 run rqt_image_view rqt_image_view /overhead_cam/color/image_raw

# View depth images
ros2 run rqt_image_view rqt_image_view /red/arm_cam/depth/image_rect_raw
```

### Accessing Point Clouds

```bash
# List point cloud topics
ros2 topic list | grep points

# Echo point cloud data
ros2 topic echo /red/arm_cam/depth/color/points --once

# Visualize in RViz (already configured in simulation)
```

## Configuration Files

### Controller Configuration

Controllers are configured in `config/` directory:

- **red_controllers.yaml**: Red robot controller parameters
- **green_controllers.yaml**: Green robot controller parameters

Key parameters:
- Joint trajectory controller gains
- Tolerances for goal and path
- State publish rate
- Action monitor rate

### RViz Configuration

RViz configs in `rviz/` directory provide:
- Robot model visualization
- TF tree display
- Camera image overlays
- Joint state visualization
- Trajectory markers

## Launch Files

### simulation.launch.py
Main launcher that orchestrates:
1. Gazebo world startup
2. Robot state publishers (red and green)
3. Robot spawning in Gazebo
4. Controller loading and activation
5. ROS-Gazebo bridges
6. RViz visualization
7. Ball spawner node

**Launch Arguments:**
- `use_sim_time`: Use simulation time (default: true)

### bridge.launch.py
Manages ROS-Gazebo topic bridges for:
- Clock synchronization
- Joint states
- Image and depth streams
- Camera info
- Point clouds

### ball_spawn.launch.py
Launches the ball spawner action server node.

### test.launch.py
Minimal launcher for testing components.

## Troubleshooting

### Controllers Not Activating

```bash
# Check if controllers are loaded
ros2 control list_controllers

# Check controller manager logs
ros2 run ros2 topic echo /rosout | grep controller_manager

# Verify ros2_control packages installed
ros2 pkg list | grep ros2_control
```

**Solution**: Ensure ros2_control packages are installed:
```bash
sudo apt install -y ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-gz-ros2-control
```

### Ball Not Spawning

```bash
# Check if action server is running
ros2 action list

# Test action server
ros2 action send_goal /spawn_ball table_tennis_gazebo/action/SpawnBall "{x: 0.0, y: 0.0, z: 2.0}"
```

**Common issues:**
- Action server not started (wait ~25 seconds after launch)
- Another ball already active (only one ball allowed)
- Invalid spawn position (z must be > 0)

### Camera Images Not Visible

```bash
# List camera topics
ros2 topic list | grep cam

# Check topic frequency
ros2 topic hz /red/arm_cam/color/image_raw

# Verify bridge is running
ros2 node list | grep bridge
```

**Solution**: Ensure ROS-Gazebo bridges are active (automatically launched).

### RViz Shows "Error" for RobotModel

This usually means:
1. `robot_description` topic not publishing
2. Controllers not activated
3. Missing TF transforms

**Solution**:
```bash
# Check robot_description
ros2 topic list | grep robot_description

# Check TF tree
ros2 run tf2_tools view_frames

# Verify controller status
ros2 control list_controllers
```

### Simulation Runs Slowly

**Tips:**
- Close unused RViz windows
- Reduce camera resolution in world file
- Disable shadows in Gazebo GUI
- Use `--headless` mode for testing

```bash
# Headless mode (no GUI)
ros2 launch table_tennis_gazebo simulation.launch.py headless:=true
```

## Development

### Adding New Models

1. Create model directory in `models/`
2. Add `model.config` and `model.sdf`
3. Update `GZ_SIM_RESOURCE_PATH` in launch file
4. Reference in world file

### Modifying Ball Physics

Edit `models/ball/model.sdf`:
- Mass: `<mass>0.0027</mass>` (2.7g)
- Radius: `radius>0.02</radius>` (40mm)
- Restitution: `<restitution>0.89</restitution>`
- Friction: `<mu>0.4</mu>`, `<mu2>0.4</mu2>`

### Adding More Cameras

1. Define camera in `urdf/sensors/` (table_tennis_description package)
2. Add to robot xacro file
3. Create bridge in `bridge.launch.py`
4. Update RViz config

### Custom Controller Configuration

Modify `config/*_controllers.yaml`:
```yaml
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz
    
arm_controller:
  ros__parameters:
    joints:
      - fr3_joint1
      # ... more joints
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity
    constraints:
      goal_time: 0.5
      # ... tolerances
```

## Dependencies

- table_tennis_description
- franka_description
- ros2_control
- ros2_controllers
- gz_ros2_control
- ros_gz_sim
- ros_gz_bridge
- xacro

## See Also

- [table_tennis_description](../table_tennis_description/README.md) - Robot and sensor URDF definitions
- [Main Workspace README](../../README.md) - Complete setup and installation guide
- [ROS 2 Jazzy Documentation](https://docs.ros.org/en/jazzy/)
- [Gazebo Harmonic Documentation](https://gazebosim.org/docs/harmonic)

## License

[Add your license here]

## Maintainers

[Add maintainer information]

---

**Package**: table_tennis_gazebo  
**Version**: 0.1.0  
**ROS 2 Distro**: Jazzy  
**Gazebo Version**: Harmonic (gz-sim 8)  
**Last Updated**: February 23, 2026
