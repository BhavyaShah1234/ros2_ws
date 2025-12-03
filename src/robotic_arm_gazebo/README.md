# Robotic Arm Gazebo Package

This package provides Gazebo simulation capabilities for robotic arms.

## Contents

- **launch/**: Gazebo launch files
  - `spawn_arm.launch.py`: Launches Gazebo and spawns the robotic arm
- **config/**: Configuration files for controllers and plugins

## Usage

To simulate the robotic arm in Gazebo:
```bash
ros2 launch robotic_arm_gazebo spawn_arm.launch.py
```

This will:
1. Start Gazebo simulator
2. Load the robotic arm URDF from `robotic_arm_description`
3. Spawn the arm in the simulation
4. Start the robot state publisher

## Dependencies

This package depends on:
- `robotic_arm_description`: For the URDF model
- `gazebo_ros`: ROS 2 Gazebo integration
- `robot_state_publisher`: Publishes robot state

## Adding Controllers

To add motion controllers:
1. Add controller configurations to the `config/` directory
2. Modify the launch file to load controller plugins
3. Use ros2_control framework for advanced control

## Integration with Worlds

You can combine this with the `gazebo_worlds` package to spawn the arm in different environments.
