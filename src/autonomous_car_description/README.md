# Autonomous Car Description Package

This package contains URDF models and visualization launch files for autonomous vehicles.

## Contents

- **urdf/**: Vehicle description files in URDF format
  - `simple_car.urdf`: A simple 4-wheeled car with lidar sensor
- **launch/**: Launch files for visualization
  - `display.launch.py`: Launch file to visualize the car in RViz with joint state publisher GUI
- **meshes/**: 3D mesh files (placeholder for custom meshes)
- **config/**: Configuration files (placeholder for RViz configs, etc.)

## Usage

To visualize the autonomous car in RViz:
```bash
ros2 launch autonomous_car_description display.launch.py
```

This will launch:
- Robot State Publisher: Publishes the vehicle's state
- Joint State Publisher GUI: Allows manual control of wheel positions

## URDF Model Details

The simple car consists of:
- Base link (chassis): 0.6m x 0.3m x 0.2m box
- 4 wheels: Continuous joints for rotation
  - Front left/right wheels
  - Rear left/right wheels
- Lidar sensor: Mounted on top of the chassis

## Customization

You can modify the URDF file to:
- Add more sensors (cameras, IMU, GPS, etc.)
- Change vehicle dimensions
- Add steering mechanisms
- Include custom mesh files for realistic appearance
- Add suspension systems
