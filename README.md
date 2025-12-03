# ROS 2 Workspace

Collection of ROS 2 packages for simulating robotic arms, autonomous cars, and various Gazebo worlds.

## Overview

This workspace contains multiple ROS 2 packages organized for different simulation purposes:

### Robotic Arm Packages
- **robotic_arm_description**: URDF models and visualization files for robotic arms
- **robotic_arm_gazebo**: Gazebo simulation launch files for robotic arms

### Autonomous Car Packages
- **autonomous_car_description**: URDF models and visualization files for autonomous cars
- **autonomous_car_gazebo**: Gazebo simulation launch files for autonomous cars

### Gazebo Worlds Package
- **gazebo_worlds**: Collection of different world files for various simulation scenarios
  - Empty world
  - Indoor world (with walls)
  - Outdoor world (with obstacles)

## Prerequisites

- ROS 2 (Humble, Iron, or newer)
- Gazebo (Gazebo 11 for ROS 2 Humble, or Gazebo Fortress/Garden for newer distributions)
- Python 3
- colcon build tool

## Installation

1. Clone this repository:
```bash
git clone https://github.com/BhavyaShah1234/ros2_ws.git
cd ros2_ws
```

2. Install dependencies:
```bash
rosdep install --from-paths src --ignore-src -r -y
```

3. Build the workspace:
```bash
colcon build
```

4. Source the workspace:
```bash
source install/setup.bash
```

## Usage

### Visualizing Robotic Arm (RViz)

To visualize the robotic arm in RViz with joint state publisher GUI:
```bash
ros2 launch robotic_arm_description display.launch.py
```

### Simulating Robotic Arm in Gazebo

To spawn the robotic arm in Gazebo:
```bash
ros2 launch robotic_arm_gazebo spawn_arm.launch.py
```

### Visualizing Autonomous Car (RViz)

To visualize the autonomous car in RViz with joint state publisher GUI:
```bash
ros2 launch autonomous_car_description display.launch.py
```

### Simulating Autonomous Car in Gazebo

To spawn the autonomous car in Gazebo:
```bash
ros2 launch autonomous_car_gazebo spawn_car.launch.py
```

### Loading Different Gazebo Worlds

To launch Gazebo with different worlds:

Empty world:
```bash
ros2 launch gazebo_worlds gazebo_world.launch.py world:=empty.world
```

Indoor world (with walls):
```bash
ros2 launch gazebo_worlds gazebo_world.launch.py world:=indoor.world
```

Outdoor world (with obstacles):
```bash
ros2 launch gazebo_worlds gazebo_world.launch.py world:=outdoor.world
```

## Package Structure

```
ros2_ws/
├── src/
│   ├── robotic_arm_description/
│   │   ├── urdf/              # URDF robot models
│   │   ├── launch/            # Launch files
│   │   ├── meshes/            # 3D mesh files
│   │   └── config/            # Configuration files
│   ├── robotic_arm_gazebo/
│   │   ├── launch/            # Gazebo launch files
│   │   └── config/            # Simulation configurations
│   ├── autonomous_car_description/
│   │   ├── urdf/              # URDF vehicle models
│   │   ├── launch/            # Launch files
│   │   ├── meshes/            # 3D mesh files
│   │   └── config/            # Configuration files
│   ├── autonomous_car_gazebo/
│   │   ├── launch/            # Gazebo launch files
│   │   └── config/            # Simulation configurations
│   └── gazebo_worlds/
│       ├── worlds/            # World SDF files
│       └── launch/            # Launch files for worlds
├── build/                     # Build artifacts (not committed)
├── install/                   # Installation files (not committed)
└── log/                       # Log files (not committed)
```

## Contributing

Contributions are welcome! Please feel free to submit pull requests or open issues for bugs and feature requests.

## License

This project is licensed under the MIT License - see the LICENSE file for details.
