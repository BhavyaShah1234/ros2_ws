# Yahboom X3 ROS 2

ROS 2 Jazzy packages for the Yahboom Dofbot/ROS Master X3 robotic arm - a compact educational robot platform for learning robotics, computer vision, and AI applications.

## Overview

This package provides ROS 2 integration for the Yahboom X3 series robots, including URDF descriptions, simulation capabilities, and hardware control interfaces. The X3 platform is designed for education and research in manipulation and robot learning.

## Features

- **Complete URDF model** with accurate kinematics
- **Visual and collision meshes** for realistic simulation
- **ros2_control integration** for unified control interface
- **RViz visualization** with interactive controls
- **Configurable robot parameters** (namespace, origin, etc.)
- **Camera/sensor integration** (optional)
- **Educational-friendly** documentation and examples

## Packages

### yahboom_x3_description
Complete robot description files for Yahboom X3 robots.

**Contents**:
- URDF/xacro definitions
- 3D meshes (visual and collision)
- RViz configuration files
- Controller configuration
- Launch files for robot state publisher

**Key Files**:
- `urdf/` - Robot URDF and xacro files
- `meshes/` - 3D model files
- `config/` - Controller and parameter configs
- `launch/robot_state_publisher.launch.py` - Main launcher

### yahboom_x3_ros2
Core ROS 2 nodes and utilities.

**Purpose**:
- Custom robot control nodes
- Utility libraries
- Hardware interface implementations
- Service and action servers
- Example scripts

## Robot Specifications

**Yahboom X3 (Dofbot/ROS Master)**:
- **DOF**: 4-5 (depending on model)
- **Payload**: ~100g
- **Reach**: ~250mm
- **Joints**: Servo-based (bus servos)
- **Communication**: Serial/USB
- **Power**: 7-12V DC
- **Gripper**: 2-finger adaptive gripper

**Typical Joint Configuration**:
- Base rotation (joint1)
- Shoulder (joint2)
- Elbow (joint3)
- Wrist pitch (joint4)
- Gripper (optional joint5)

## Prerequisites

- **ROS 2 Jazzy**
- **Python 3.10+**
- **ros2_control** packages
- **xacro, robot_state_publisher**
- **RViz2** (for visualization)
- **PySerial** (for hardware interface)

See the [main workspace README](../../README.md) for installation instructions.

## Building

```bash
# Navigate to workspace
cd ~/Projects/ros2_ws

# Install dependencies
rosdep install --from-paths src/yahboom_x3_ros2 --ignore-src -r -y

# Build Yahboom X3 packages
colcon build --packages-select \
  yahboom_x3_description \
  yahboom_x3_ros2

# Source workspace
source install/setup.bash
```

## Quick Start

### 1. Visualize Robot in RViz

```bash
# Launch robot state publisher and RViz
ros2 launch yahboom_x3_description robot_state_publisher.launch.py

# With interactive joint control GUI
ros2 launch yahboom_x3_description robot_state_publisher.launch.py use_gui:=true
```

### 2. View Robot Model

The RViz window will show:
- 3D robot model with all links
- TF frames
- Joint state visualization
- Interactive markers (with GUI)

### 3. Customize Robot Configuration

```bash
# Launch with custom parameters
ros2 launch yahboom_x3_description robot_state_publisher.launch.py \
  robot_name:=my_robot \
  x:=1.0 \
  y:=0.0 \
  z:=0.5 \
  use_sim_time:=true
```

## Package Structure

```
yahboom_x3_ros2/
├── yahboom_x3_description/
│   ├── config/
│   │   └── controllers.yaml          # ros2_control configuration
│   ├── launch/
│   │   └── robot_state_publisher.launch.py
│   ├── meshes/
│   │   ├── base_link.stl
│   │   ├── link1.stl
│   │   ├── link2.stl
│   │   └── ...                       # Additional link meshes
│   ├── rviz/
│   │   └── view_robot.rviz           # RViz config
│   ├── urdf/
│   │   ├── yahboom_x3.urdf.xacro     # Main robot description
│   │   └── components/               # Component macros
│   ├── CMakeLists.txt
│   └── package.xml
│
└── yahboom_x3_ros2/
    ├── include/
    │   └── yahboom_x3_ros2/          # Header files
    ├── src/
    │   └── *.cpp                      # Source files
    ├── CMakeLists.txt
    └── package.xml
```

## Usage

### Robot State Publisher

Launch file parameters:

```bash
ros2 launch yahboom_x3_description robot_state_publisher.launch.py \
  robot_name:=x3_robot \     # Robot namespace/name
  x:=0.0 \                    # X position (meters)
  y:=0.0 \                    # Y position (meters)
  z:=0.0 \                    # Z position (meters)
  roll:=0.0 \                 # Roll orientation (radians)
  pitch:=0.0 \                # Pitch orientation (radians)
  yaw:=0.0 \                  # Yaw orientation (radians)
  use_sim_time:=false \       # Use simulation time
  use_gui:=false              # Launch joint_state_publisher_gui
```

### Manual Joint Control

```bash
# Publish joint states manually
ros2 topic pub /joint_states sensor_msgs/msg/JointState "{
  name: ['joint1', 'joint2', 'joint3', 'joint4'],
  position: [0.0, -0.5, 0.5, 0.0]
}" --once

# Or use GUI (launched with use_gui:=true)
```

### Viewing in RViz

RViz displays configured with:
- **RobotModel**: Shows 3D visualization
- **TF**: Displays coordinate frames
- **JointStateDisplay**: Shows current joint values
- **Grid**: Reference ground plane

### Accessing URDF

```bash
# Get robot description from parameter server
ros2 param get /robot_state_publisher robot_description > robot.urdf

# Validate URDF
check_urdf robot.urdf

# View URDF structure
urdf_to_graphiz robot.urdf
```

## Configuration

### Controller Configuration

Edit `yahboom_x3_description/config/controllers.yaml`:

```yaml
controller_manager:
  ros__parameters:
    update_rate: 50  # Hz
    
    arm_controller:
      type: joint_trajectory_controller/JointTrajectoryController
    
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

arm_controller:
  ros__parameters:
    joints:
      - joint1
      - joint2
      - joint3
      - joint4
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity
    constraints:
      stopped_velocity_tolerance: 0.01
      goal_time: 2.0
```

### Customizing URDF

Modify `yahboom_x3_description/urdf/yahboom_x3.urdf.xacro`:

```xml
<!-- Adjust link masses -->
<xacro:property name="base_mass" value="0.5"/>
<xacro:property name="link1_mass" value="0.1"/>

<!-- Adjust joint limits -->
<joint name="joint1" type="revolute">
  <limit lower="-3.14" upper="3.14" effort="10.0" velocity="2.0"/>
</joint>

<!-- Add sensors -->
<xacro:include filename="$(find sensor_description)/urdf/camera.urdf.xacro"/>
<xacro:camera_sensor parent="link4" name="wrist_cam"/>
```

## Topics and Services

### Published Topics
- `/joint_states` (sensor_msgs/JointState) - Current joint positions
- `/robot_description` (std_msgs/String) - URDF parameter
- `/tf` (tf2_msgs/TFMessage) - Transform tree
- `/tf_static` (tf2_msgs/TFMessage) - Static transforms

### Subscribed Topics
- `/joint_states` (sensor_msgs/JointState) - For visualization without controller

### Services
- `/robot_state_publisher/get_parameters` - Get node parameters
- `/robot_state_publisher/set_parameters` - Set node parameters

## Integration Examples

### With Gazebo Simulation

```xml
<!-- Create Gazebo launch file -->
<launch>
  <include file="$(find yahboom_x3_description)/launch/robot_state_publisher.launch.py">
    <arg name="use_sim_time" value="true"/>
  </include>
  
  <!-- Add Gazebo specific configurations -->
  <!-- Spawn robot in Gazebo -->
  <!-- Load controllers -->
</launch>
```

### With MoveIt 2

```bash
# Generate MoveIt config (if not exists)
ros2 run moveit_setup_assistant moveit_setup_assistant

# Load yahboom_x3.urdf
# Configure planning groups, end effectors
# Generate package
```

### With Hardware

```python
# Example hardware interface node
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import serial

class YahboomHardwareInterface(Node):
    def __init__(self):
        super().__init__('hardware_interface')
        self.serial = serial.Serial('/dev/ttyUSB0', 115200)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.create_timer(0.02, self.read_joints)
    
    def read_joints(self):
        # Read joint positions from robot
        # Publish JointState message
        pass
```

### With Computer Vision

```python
# Add camera to URDF
# Process camera feed for object detection
# Use with pick-and-place applications
```

## Troubleshooting

### Robot Not Displaying in RViz

**Check**:
```bash
# Verify robot_state_publisher is running
ros2 node list | grep robot_state_publisher

# Check if robot_description is published
ros2 param get /robot_state_publisher robot_description

# Verify TF tree
ros2 run tf2_tools view_frames
evince frames.pdf
```

**Solution**: Ensure launch file started correctly and URDF is valid.

### URDF Parse Errors

```bash
# Validate URDF
check_urdf install/yahboom_x3_description/share/yahboom_x3_description/urdf/yahboom_x3.urdf

# Generate from xacro
xacro install/yahboom_x3_description/share/yahboom_x3_description/urdf/yahboom_x3.urdf.xacro > debug.urdf
check_urdf debug.urdf
```

### Missing Meshes

**Symptoms**: Gray/missing parts in RViz

**Solution**:
```bash
# Verify meshes installed
ls install/yahboom_x3_description/share/yahboom_x3_description/meshes/

# Check mesh paths in URDF
grep "mesh filename" debug.urdf
```

### TF Frames Not Connected

**Check**:
```bash
# View TF tree
ros2 run tf2_tools view_frames

# Listen to specific transform
ros2 run tf2_ros tf2_echo base_link link4
```

**Solution**: Ensure robot_state_publisher is processing all joints correctly.

## Hardware Integration

### Connecting to Real Robot

```bash
# Find USB port
ls /dev/ttyUSB* /dev/ttyACM*

# Test connection
sudo chmod 666 /dev/ttyUSB0  # Grant permissions
python3 -c "import serial; ser = serial.Serial('/dev/ttyUSB0', 115200); print('Connected')"

# Launch hardware interface (if available)
ros2 launch yahboom_x3_ros2 hardware.launch.py port:=/dev/ttyUSB0
```

### Serial Communication

The Yahboom X3 typically uses a custom protocol for servo control:
- **Baud rate**: 115200
- **Protocol**: Varies by model (HTS/LX servos)
- **Commands**: Position, speed, torque control

Refer to manufacturer documentation for exact protocol.

## Development

### Adding Custom Links

1. Create/obtain 3D mesh (STL/DAE format)
2. Place in `meshes/` directory
3 Add link definition in URDF:
   ```xml
   <link name="custom_link">
     <visual>
       <geometry>
         <mesh filename="package://yahboom_x3_description/meshes/custom.stl"/>
       </geometry>
     </visual>
     <collision>
       <geometry>
         <!-- Simplified collision geometry -->
       </geometry>
     </collision>
     <inertial>
       <mass value="0.1"/>
       <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
     </inertial>
   </link>
   ```

### Calculating Inertial Properties

```python
# Use MeshLab or similar tool to get mesh properties
# Or approximate for simple shapes:

# For cylinder:
# I_xx = I_yy = (1/12) * m * (3*r^2 + h^2)
# I_zz = (1/2) * m * r^2

# For box:
# I_xx = (1/12) * m * (h^2 + d^2)
# I_yy = (1/12) * m * (w^2 + d^2)
# I_zz = (1/12) * m * (w^2 + h^2)
```

### Testing Changes

```bash
# Clean build
rm -rf build install log

# Rebuild
colcon build --packages-select yahboom_x3_description

# Test visualization
ros2 launch yahboom_x3_description robot_state_publisher.launch.py use_gui:=true
```

## Educational Use

### Beginner Projects
1. **Joint visualization**: Understand robot kinematics in RViz
2. **Manual control**: Use joint_state_publisher_gui
3. **TF learning**: Explore coordinate frame transforms
4. **URDF exploration**: Modify and visualize robot structure

### Intermediate Projects
1. **Inverse kinematics**: Implement basic IK solver
2. **Trajectory planning**: Create simple motion sequences
3. **Simulation**: Add Gazebo integration
4. **Sensor fusion**: Integrate camera or other sensors

### Advanced Projects
1. **Machine learning**: Train RL agent for manipulation
2. **Computer vision**: Object detection and grasping
3. **Multi-robot**: Coordinate multiple X3 robots
4. **Custom end effectors**: Design and integrate tools

## Dependencies

**Required**:
- ros2_cli
- robot_state_publisher
- joint_state_publisher
- xacro
- urdf

**Optional**:
- joint_state_publisher_gui (for manual control)
- rviz2 (for visualization)
- ros2_control (for advanced control)
- gazebo packages (for simulation)

## See Also

- [Yahboom Official Documentation](https://www.yahboom.com/)
- [ROS 2 URDF Tutorials](https://docs.ros.org/en/jazzy/Tutorials/URDF/)
- [robot_state_publisher Documentation](https://github.com/ros/robot_state_publisher)
- [Main Workspace README](../../README.md)

## License

[Add your license here]

## Maintainers

[Add maintainer information]

---

**Package**: yahboom_x3_ros2  
**Version**: 0.1.0  
**ROS 2 Distro**: Jazzy  
**Robot**: Yahboom Dofbot/ROS Master X3  
**Last Updated**: February 23, 2026
