# MyCobot ROS 2

ROS 2 Jazzy packages for the Elephant Robotics MyCobot 280 robotic arm - a 6-DOF collaborative robot designed for education, research, and pick-and-place applications.

## Overview

This metapackage provides a complete ROS 2 integration for the MyCobot 280, including simulation, visualization, motion planning with MoveIt 2, and hardware control capabilities.

## Features

- **Full URDF description** with accurate kinematics and collision geometry
- **Gazebo Harmonic simulation** with physics-based dynamics
- **MoveIt 2 integration** for motion planning and trajectory execution
- **ros2_control** hardware interfaces for real robot and simulation
- **Pick-and-place demonstrations** with gripper control
- **RViz visualization** with interactive markers
- **System tests** for validation

## Packages

### mycobot_description
URDF and xacro files defining the MyCobot 280 robot model.

**Contents**:
- Robot URDF with joint definitions
- Gripper/end effector descriptions
- Visual and collision meshes
- Material and inertial properties
- Launch files for robot state publisher

**Key Files**:
- `urdf/mycobot_280.urdf.xacro` - Main robot description
- `meshes/` - 3D mesh files for visualization
- `config/` - Robot configuration parameters

### mycobot_gazebo
Gazebo simulation environment for MyCobot.

**Features**:
- Pre-configured simulation worlds
- Pick-and-place demo environment
- ROS-Gazebo bridges for topics
- Simulated gripper actuation
- Camera sensors (if equipped)

**Launch Files**:
- `mycobot.gazebo.launch.py` - Main simulation launcher

### mycobot_moveit_config
MoveIt 2 configuration for motion planning.

**Includes**:
- SRDF (Semantic Robot Description)
- Planning groups (arm, gripper)
- End effector definitions
- Motion planning parameters
- Controllers configuration
- RViz motion planning plugin

### mycobot_bringup
High-level launch files and configurations.

**Purpose**:
- Unified launch files combining multiple nodes
- Hardware bringup scripts
- System integration configurations

### mycobot_ros2
Core ROS 2 nodes and libraries.

**Contents**:
- Custom nodes for robot control
- Utility functions
- Hardware interface implementations
- Service/action servers

### mycobot_system_tests
Integration tests for the MyCobot system.

**Test Coverage**:
- Simulation launch tests
- MoveIt planning tests
- Controller interface tests
- End-to-end workflows

## Prerequisites

- **ROS 2 Jazzy**
- **Gazebo Harmonic** (gz-sim 8)
- **MoveIt 2** for Jazzy
- **ros2_control** packages
- **Python 3.10+**

See the [main workspace README](../../README.md) for installation instructions.

## Building

```bash
# Navigate to workspace
cd ~/Projects/ros2_ws

# Install dependencies
rosdep install --from-paths src/mycobot_ros2 --ignore-src -r -y

# Build MyCobot packages
colcon build --packages-select \
  mycobot_description \
  mycobot_gazebo \
  mycobot_moveit_config \
  mycobot_bringup \
  mycobot_ros2 \
  mycobot_system_tests

# Source the workspace
source install/setup.bash
```

## Quick Start

### 1. Launch Gazebo Simulation

```bash
# Start simulation with pick-and-place world
ros2 launch mycobot_gazebo mycobot.gazebo.launch.py
```

This launches:
- Gazebo with the robot and environment
- Controller manager
- Joint trajectory controller
- Robot state publisher
- ROS-Gazebo bridges

### 2. Visualize in RViz

```bash
# Launch RViz with robot model
ros2 launch mycobot_description robot_state_publisher.launch.py

# Or launch with interactive joint control
ros2 launch mycobot_description robot_state_publisher.launch.py use_gui:=true
```

### 3. Control with MoveIt 2

```bash
# Launch MoveIt motion planning
ros2 launch mycobot_moveit_config demo.launch.py

# In RViz:
# - Use "MotionPlanning" panel
# - Set goal pose with interactive marker
# - Click "Plan" to compute trajectory
# - Click "Execute" to move robot
```

### 4. Send Joint Trajectories

```bash
# Example: Move to a specific joint configuration
ros2 action send_goal /arm_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory "{
    trajectory: {
      joint_names: [joint1, joint2, joint3, joint4, joint5, joint6],
      points: [
        { positions: [0.0, -1.0, 1.5, 0.0, 0.5, 0.0], time_from_start: { sec: 3 } }
      ]
    }
  }"
```

## Robot Specifications

**MyCobot 280**:
- **DOF**: 6 (6 revolute joints)
- **Payload**: 250g
- **Reach**: 280mm
- **Weight**: ~960g
- **Repeatability**: ±0.5mm
- **Communication**: Serial (USB/TTL)
- **Power**: 12V DC

**Joint Ranges**:
- Joint 1-6: Varies by joint (see URDF for exact limits)
- Typical range: ~±150° to ±180° per joint

**End Effector**:
- Electric gripper (optional)
- Custom tool mounting
- Flange diameter: ~35mm

## Usage Examples

### Pick and Place Demo

```bash
# Launch full pick-and-place demonstration
ros2 launch mycobot_gazebo mycobot.gazebo.launch.py world:=pick_and_place_demo

# Run pick-and-place script (if available)
ros2 run mycobot_ros2 pick_and_place_demo
```

### Jogging with Keyboard/Gamepad

```bash
# Install teleop_twist_keyboard
sudo apt install ros-jazzy-teleop-twist-keyboard

# Launch teleop control
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Use with joint position control (requires custom node)
```

### Recording and Playback

```bash
# Record joint states
ros2 bag record /joint_states /arm_controller/state

# Playback recorded motion
ros2 bag play <bag_file>
```

## Configuration

### Controller Parameters

Edit `mycobot_gazebo/config/controllers.yaml`:

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100
    
    arm_controller:
      type: joint_trajectory_controller/JointTrajectoryController
    
    gripper_controller:
      type: position_controllers/GripperActionController

arm_controller:
  ros__parameters:
    joints:
      - joint1
      - joint2
      - joint3
      - joint4
      - joint5
      - joint6
    constraints:
      goal_time: 1.0
      stopped_velocity_tolerance: 0.05
```

### MoveIt Planning Parameters

Modify `mycobot_moveit_config/config/ompl_planning.yaml` for:
- Planning algorithm selection (RRTConnect, RRT*, etc.)
- Planning time limits
- Smoothing parameters
- Collision checking resolution

## Topics and Services

### Joint Control
- `/arm_controller/follow_joint_trajectory` (action) - Send trajectories
- `/joint_states` (topic) - Current joint positions/velocities
- `/arm_controller/state` (topic) - Controller state

### Gripper Control
- `/gripper_controller/gripper_cmd` (action) - Open/close gripper
- `/gripper_position` (topic) - Current gripper position

### Robot State
- `/robot_description` (topic) - URDF parameter
- `/tf` (topic) - Transform tree
- `/tf_static` (topic) - Static transforms

## Troubleshooting

### Controllers Not Starting

```bash
# Check controller manager
ros2 control list_controllers

# Check if hardware interface is loaded
ros2 control list_hardware_interfaces
```

**Solution**: Verify ros2_control packages installed:
```bash
sudo apt install ros-jazzy-ros2-control ros-jazzy-ros2-controllers
```

### MoveIt Planning Fails

**Common issues**:
- Collision with environment/self
- Goal pose unreachable
- Planning timeout too short

**Debug**:
```bash
# Check for collision violations
# In RViz, enable "Planning Scene" display

# Increase planning time in MoveIt config
# Edit: mycobot_moveit_config/config/ompl_planning.yaml
```

### Simulation Runs Slowly

**Solutions**:
- Close unnecessary visualization windows
- Reduce physics update rate in world file
- Disable unused sensors
- Use `--headless` mode for testing

### URDF Parse Errors

```bash
# Validate URDF
check_urdf install/mycobot_description/share/mycobot_description/urdf/mycobot_280.urdf

# Generate URDF from xacro
xacro install/mycobot_description/share/mycobot_description/urdf/mycobot_280.urdf.xacro > debug.urdf
check_urdf debug.urdf
```

## Hardware Integration

### Connecting to Real Robot

```bash
# Install PySerial for USB communication
pip3 install pyserial

# Find robot serial port
ls /dev/ttyUSB* # or /dev/ttyACM*

# Launch hardware interface
ros2 launch mycobot_bringup hardware.launch.py port:=/dev/ttyUSB0

# Verify connection
ros2 topic echo /joint_states
```

### Calibration

Before first use with real hardware:
1. Power on robot
2. Manually move to home position
3. Run calibration routine (if provided)
4. Verify joint limits match physical robot

## Development

### Adding Custom End Effectors

1. Create URDF in `mycobot_description/urdf/end_effectors/`
2. Define xacro macro with parameters
3. Include in main robot file
4. Update MoveIt SRDF with new end effector group
5. Test in Gazebo

### Creating Custom Worlds

```xml
<!-- In mycobot_gazebo/worlds/ -->
<sdf version="1.8">
  <world name="my_custom_world">
    <include><uri>model://sun</uri></include>
    <include><uri>model://ground_plane</uri></include>
    
    <!-- Add your custom models -->
    
  </world>
</sdf>
```

### Writing Pick-and-Place Scripts

Example Python script structure:

```python
import rclpy
from rclpy.node import Node
from moveit_py import MoveGroupInterface

class PickAndPlace(Node):
    def __init__(self):
        super().__init__('pick_and_place')
        self.move_group = MoveGroupInterface('arm')
        
    def pick(self, x, y, z):
        # Set target pose
        # Plan trajectory
        # Execute motion
        # Close gripper
        pass
    
    def place(self, x, y, z):
        # Move to place location
        # Open gripper
        pass
```

## Dependencies

**Required**:
- ros2_control
- ros2_controllers  
- gz_ros2_control
- robot_state_publisher
- xacro
- moveit_ros (for MoveIt features)

**Optional**:
- camera drivers (if using vision)
- gripper controller packages
- rviz2

## See Also

- [Elephant Robotics Official Documentation](https://www.elephantrobotics.com/en/mycobot-280-2023-en/)
- [MoveIt 2 Documentation](https://moveit.picknik.ai/jazzy/)
- [ros2_control Documentation](https://control.ros.org/)
- [Main Workspace README](../../README.md)

## License

[Add your license here]

## Maintainers

[Add maintainer information]

---

**Package**: mycobot_ros2  
**Version**: 0.1.0  
**ROS 2 Distro**: Jazzy  
**Robot**: Elephant Robotics MyCobot 280  
**Last Updated**: February 23, 2026
