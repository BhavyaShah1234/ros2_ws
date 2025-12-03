# Autonomous Car Gazebo Package

This package provides Gazebo simulation capabilities for autonomous vehicles.

## Contents

- **launch/**: Gazebo launch files
  - `spawn_car.launch.py`: Launches Gazebo and spawns the autonomous car
- **config/**: Configuration files for controllers and plugins

## Usage

To simulate the autonomous car in Gazebo:
```bash
ros2 launch autonomous_car_gazebo spawn_car.launch.py
```

This will:
1. Start Gazebo simulator
2. Load the car URDF from `autonomous_car_description`
3. Spawn the vehicle in the simulation
4. Start the robot state publisher

## Dependencies

This package depends on:
- `autonomous_car_description`: For the URDF model
- `gazebo_ros`: ROS 2 Gazebo integration
- `robot_state_publisher`: Publishes vehicle state

## Adding Features

### Sensors
Add Gazebo plugins for sensors like:
- Lidar (already included in URDF, needs plugin)
- Cameras
- IMU
- GPS

### Control
To add motion control:
1. Add differential drive or Ackermann steering plugin
2. Configure controller parameters in `config/`
3. Publish velocity commands to move the vehicle

## Integration with Worlds

Combine with `gazebo_worlds` package to test navigation in different environments:
```bash
# Terminal 1: Launch world
ros2 launch gazebo_worlds gazebo_world.launch.py world:=indoor.world

# Terminal 2: Spawn car
ros2 launch autonomous_car_gazebo spawn_car.launch.py
```
