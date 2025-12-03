# Gazebo Worlds Package

This package contains a collection of Gazebo world files for different simulation scenarios.

## Contents

- **worlds/**: SDF world files
  - `empty.world`: Basic empty world with ground plane and lighting
  - `indoor.world`: Indoor environment with walls forming a room
  - `outdoor.world`: Outdoor environment with obstacles (trees and rocks)
- **launch/**: Launch files to load worlds
  - `gazebo_world.launch.py`: Configurable launch file for any world

## Usage

### Empty World
Basic environment for testing:
```bash
ros2 launch gazebo_worlds gazebo_world.launch.py world:=empty.world
```

### Indoor World
Simulates an indoor environment with walls:
```bash
ros2 launch gazebo_worlds gazebo_world.launch.py world:=indoor.world
```

### Outdoor World
Simulates an outdoor environment with natural obstacles:
```bash
ros2 launch gazebo_worlds gazebo_world.launch.py world:=outdoor.world
```

## World Details

### Empty World
- Ground plane
- Sun (global light source)
- Basic physics settings

### Indoor World
- 10m x 10m room with 2m high walls
- Suitable for indoor robot testing
- Good for testing collision detection

### Outdoor World
- Open environment with obstacles
- Trees (cylinders) and rocks (spheres)
- Suitable for outdoor navigation testing

## Creating Custom Worlds

To create your own world:
1. Create a new `.world` file in the `worlds/` directory
2. Use SDF format (version 1.6 or compatible)
3. Launch it using:
```bash
ros2 launch gazebo_worlds gazebo_world.launch.py world:=your_world.world
```

## Integration with Robots

These worlds can be used with any robot simulation. Example workflow:

**Method 1: Launch world first, then spawn robot**
```bash
# Terminal 1: Launch world
ros2 launch gazebo_worlds gazebo_world.launch.py world:=indoor.world

# Terminal 2: Spawn robot
ros2 launch robotic_arm_gazebo spawn_arm.launch.py
# or
ros2 launch autonomous_car_gazebo spawn_car.launch.py
```

**Method 2: Create combined launch file**
You can create a custom launch file that includes both the world and robot spawning.

## Customization

You can modify existing worlds or create new ones to include:
- Different lighting conditions
- More complex obstacles
- Custom models from Gazebo model database
- Dynamic objects
- Different terrain types
