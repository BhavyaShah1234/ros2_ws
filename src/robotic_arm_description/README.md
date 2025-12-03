# Robotic Arm Description Package

This package contains URDF models and visualization launch files for robotic arms.

## Contents

- **urdf/**: Robot description files in URDF format
  - `simple_arm.urdf`: A simple 3-DOF robotic arm with joints and links
- **launch/**: Launch files for visualization
  - `display.launch.py`: Launch file to visualize the arm in RViz with joint state publisher GUI
- **meshes/**: 3D mesh files (placeholder for custom meshes)
- **config/**: Configuration files (placeholder for RViz configs, etc.)

## Usage

To visualize the robotic arm in RViz:
```bash
ros2 launch robotic_arm_description display.launch.py
```

This will launch:
- Robot State Publisher: Publishes the robot's state
- Joint State Publisher GUI: Allows manual control of joint positions

## URDF Model Details

The simple arm consists of:
- Base link (fixed to ground)
- Link 1: 0.3m cylinder (rotates around Z-axis)
- Link 2: 0.25m cylinder (rotates around Y-axis)
- End effector: Sphere representing the tool

## Customization

You can modify the URDF file to:
- Add more joints and links
- Change dimensions and masses
- Add custom mesh files
- Include sensors (cameras, lidar, etc.)
