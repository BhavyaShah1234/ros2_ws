# Table Tennis Description

URDF and xacro descriptions for table tennis playing robots with paddle end effectors and RealSense D435 RGBD cameras.

## Overview

This package provides modular xacro macros for creating Franka FR3 robots equipped with table tennis-specific hardware. It extends the base `franka_description` package with custom end effectors and sensors.

## Features

### Robot Components
- **Franka FR3 base**: 7-DOF manipulator with proper kinematics
- **Table tennis paddle**: Custom end effector optimized for ball striking
- **RealSense D435 camera**: RGBD sensor mounted on link7 for perception
- **ros2_control integration**: Position-controlled joints with Gazebo plugins
- **Namespacing support**: Multiple robots in same environment (red/green)

### Xacro Modularity
- Reusable macros for easy customization
- Parameterized robot positioning and naming
- Separate definitions for robots, end effectors, sensors, and controllers
- Compatible with Gazebo Harmonic and ROS 2 Jazzy

## Package Structure

```
table_tennis_description/
├── config/
│   ├── red_controllers.yaml      # Red robot controller config
│   └── green_controllers.yaml    # Green robot controller config
├── launch/
│   ├── robot_state_publisher.launch.py   # Publish robot description
│   └── visualize.launch.py               # RViz visualization
├── meshes/
│   ├── paddle/                   # Paddle mesh files (STL/DAE)
│   └── sensors/                  # Camera mesh files
├── rviz/
│   └── view_robot.rviz          # RViz configuration
├── scripts/
│   └── convert_meshes.py        # Mesh conversion utilities
├── urdf/
│   ├── robots/
│   │   └── franka_with_paddle.urdf.xacro    # Main robot definition
│   ├── end_effectors/
│   │   └── paddle/
│   │       ├── paddle.urdf.xacro            # Paddle URDF
│   │       └── paddle_macro.xacro           # Paddle xacro macro
│   ├── sensors/
│   │   └── realsense_d435.urdf.xacro        # Camera definition
│   └── control/
│       └── gazebo_sim_ros2_control.urdf.xacro  # Controller plugin config
├── CMakeLists.txt
├── package.xml
└── README.md
```

## Usage

### Building

```bash
cd ~/Projects/ros2_ws
colcon build --packages-select table_tennis_description
source install/setup.bash
```

### Viewing in RViz

```bash
# Launch robot state publisher and RViz
ros2 launch table_tennis_description visualize.launch.py

# View with joint state publisher GUI for interactive control
ros2 launch table_tennis_description visualize.launch.py use_gui:=true
```

### Using in Your Package

In your xacro file:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="my_robot">
  
  <!-- Include the robot macro -->
  <xacro:include filename="$(find table_tennis_description)/urdf/robots/franka_with_paddle.urdf.xacro"/>
  
  <!-- Instantiate robot with namespace -->
  <xacro:franka_with_paddle 
    arm_id="red"
    robot_origin_x="0.0"
    robot_origin_y="0.5"
    robot_origin_z="0.76"
    use_ros2_control="true"
  />
  
</robot>
```

### Parameters

#### Main Robot Macro

**franka_with_paddle** parameters:
- `arm_id` (string): Namespace/prefix for the robot (e.g., "red", "green")
- `robot_origin_x` (float): X position of robot base (default: 0.0)
- `robot_origin_y` (float): Y position of robot base (default: 0.0)
- `robot_origin_z` (float): Z position of robot base (default: 0.0)
- `robot_origin_roll` (float): Roll orientation (default: 0.0)
- `robot_origin_pitch` (float): Pitch orientation (default: 0.0)
- `robot_origin_yaw` (float): Yaw orientation (default: 0.0)
- `use_ros2_control` (bool): Enable ros2_control plugin (default: true)

#### Paddle Macro

**paddle** parameters:
- `parent_link` (string): Link to attach paddle to (typically "flange")
- `paddle_name` (string): Name for paddle links/joints
- `xyz` (list): Translation offset [x, y, z]
- `rpy` (list): Rotation offset [roll, pitch, yaw]

#### Camera Macro

**realsense_d435** parameters:
- `parent_link` (string): Link to attach camera to
- `camera_name` (string): Name for camera links/topics
- `xyz` (list): Translation offset [x, y, z]
- `rpy` (list): Rotation offset [roll, pitch, yaw]
- `use_nominal_extrinsics` (bool): Use factory calibration (default: true)

## URDF Components

### Robot Definition

The main robot file (`franka_with_paddle.urdf.xacro`) combines:

1. **Franka FR3 arm** from `franka_description`
2. **Paddle end effector** attached to flange
3. **RealSense D435 camera** on link7
4. **ros2_control hardware interface** for Gazebo

### Paddle Specifications

- **Dimensions**: 150mm (length) × 140mm (width) × 5mm (thickness)
- **Mass**: 0.2 kg (realistic for competition paddle)
- **Material**: Wood with rubber surface (simulated)
- **Attachment**: Fixed joint to flange with configurable offset
- **Collision geometry**: Simplified box for performance

Properties in `paddle.urdf.xacro`:
```xml
<link name="${paddle_name}_link">
  <inertial>
    <mass value="0.2"/>
    <!-- Inertia tensor for rectangular paddle -->
  </inertial>
  <visual>
    <!-- Mesh or primitive geometry -->
  </visual>
  <collision>
    <!-- Simplified collision geometry -->
  </collision>
</link>
```

### Camera Specifications

RealSense D435 features:
- **RGB resolution**: 1920×1080 @ 30 Hz
- **Depth resolution**: 1280×720 @ 30 Hz
- **FOV**: 87° × 58° (horizontal × vertical)
- **Range**: 0.3m - 3m (optimal for table tennis)
- **Frame rate**: 30 FPS (configurable)

Publishes:
- RGB images
- Depth images
- Camera info (intrinsics/extrinsics)
- Point clouds (XYZRGB)

### ros2_control Integration

Controller configuration in `control/gazebo_sim_ros2_control.urdf.xacro`:

```xml
<gazebo>
  <plugin filename="gz_ros2_control-system" name="gz_ros2_control::GazeboSimROS2ControlPlugin">
    <parameters>$(find table_tennis_description)/config/${arm_id}_controllers.yaml</parameters>
    <ros>
      <namespace>${arm_id}</namespace>
    </ros>
  </plugin>
</gazebo>
```

**Hardware interfaces**:
- Command interface: `position`
- State interfaces: `position`, `velocity`

**Controllers**:
- `joint_trajectory_controller`: For smooth trajectory execution
- `joint_state_broadcaster`: Publishes joint states to `/joint_states`

## Launch Files

### robot_state_publisher.launch.py

Publishes the robot URDF to `/robot_description` topic and starts `robot_state_publisher` node.

**Arguments**:
- `robot_name`: Robot namespace (default: "red")
- `x`, `y`, `z`: Robot position
- `roll`, `pitch`, `yaw`: Robot orientation
- `use_sim_time`: Use simulation time (default: false)

**Usage**:
```bash
ros2 launch table_tennis_description robot_state_publisher.launch.py \
  robot_name:=green x:=0.0 y:=-0.5 z:=0.76
```

### visualize.launch.py

Launches RViz with the robot model.

**Arguments**:
- `use_gui`: Launch joint_state_publisher_gui for manual control (default: false)
- `use_sim_time`: Use simulation time (default: false)

**Usage**:
```bash
# View only
ros2 launch table_tennis_description visualize.launch.py

# With interactive joint control
ros2 launch table_tennis_description visualize.launch.py use_gui:=true
```

## Configuration Files

### Controller Configuration

Located in `config/`:

**red_controllers.yaml** / **green_controllers.yaml**:
```yaml
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz
    
    arm_controller:
      type: joint_trajectory_controller/JointTrajectoryController
    
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

arm_controller:
  ros__parameters:
    joints:
      - fr3_joint1
      - fr3_joint2
      - fr3_joint3
      - fr3_joint4
      - fr3_joint5
      - fr3_joint6
      - fr3_joint7
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity
    state_publish_rate: 50.0
    action_monitor_rate: 20.0
    constraints:
      stopped_velocity_tolerance: 0.01
      goal_time: 0.6
```

## Customization

### Creating a Custom End Effector

1. **Create mesh files**: STL or DAE format in `meshes/your_effector/`

2. **Define URDF**: Create `urdf/end_effectors/your_effector/your_effector.urdf.xacro`
   ```xml
   <?xml version="1.0"?>
   <robot xmlns:xacro="http://www.ros.org/wiki/xacro">
     <xacro:macro name="your_effector" params="parent_link effector_name xyz rpy">
       <joint name="${effector_name}_joint" type="fixed">
         <parent link="${parent_link}"/>
         <child link="${effector_name}_link"/>
         <origin xyz="${xyz}" rpy="${rpy}"/>
       </joint>
       
       <link name="${effector_name}_link">
         <!-- Define inertial, visual, collision -->
       </link>
     </xacro:macro>
   </robot>
   ```

3. **Include in robot**: Modify `franka_with_paddle.urdf.xacro` or create new robot file

### Adding More Sensors

Example: Adding a force-torque sensor to the wrist:

```xml
<xacro:include filename="$(find sensor_description)/urdf/ft_sensor.urdf.xacro"/>

<xacro:ft_sensor 
  parent_link="${arm_id}_link7"
  sensor_name="${arm_id}_wrist_ft"
  xyz="0 0 0"
  rpy="0 0 0"
/>
```

### Adjusting Camera Position

Edit `urdf/robots/franka_with_paddle.urdf.xacro`:

```xml
<!-- Camera on link7 -->
<xacro:realsense_d435
  parent_link="${arm_id}_link7"
  camera_name="${arm_id}_arm_cam"
  xyz="0.05 0.0 0.03"    <!-- Adjust position -->
  rpy="0 0.2 0"          <!-- Adjust orientation -->
/>
```

### Modifying Joint Limits

Joint limits are inherited from `franka_description`. To override:

```xml
<xacro:property name="joint1_limit_lower" value="-2.8973"/>
<xacro:property name="joint1_limit_upper" value="2.8973"/>
<xacro:property name="joint1_limit_velocity" value="2.1750"/>
<xacro:property name="joint1_limit_effort" value="87"/>
```

## TF Frames

The robot publishes the following TF tree (for `arm_id="red"`):

```
world
└── red_link0 (base)
    └── red_link1
        └── red_link2
            └── red_link3
                └── red_link4
                    └── red_link5
                        └── red_link6
                            └── red_link7
                                ├── red_arm_cam_link
                                │   ├── red_arm_cam_color_frame
                                │   └── red_arm_cam_depth_frame
                                └── red_flange
                                    └── red_paddle_link
```

## Visualization in RViz

The included RViz configuration displays:
- **RobotModel**: 3D visualization of the robot with paddle and camera
- **TF**: Transform tree showing all frames
- **Joint states**: Current joint angles
- **Interactive markers**: Manipulate joints manually (with GUI)

Panels:
- Links with collision geometry
- Paddle mesh
- Camera frame visualization

## Testing

### Check URDF Validity

```bash
# Install check-urdf tool
sudo apt install liburdfdom-tools

# Check URDF syntax
check_urdf install/table_tennis_description/share/table_tennis_description/urdf/robots/franka_with_paddle.urdf
```

### View TF Tree

```bash
# Generate TF tree PDF
ros2 run tf2_tools view_frames

# Open the generated PDF
evince frames.pdf
```

### Test Joint Limits

```bash
# Launch with GUI
ros2 launch table_tennis_description visualize.launch.py use_gui:=true

# Move joints to extremes and verify limits are enforced
```

## Troubleshooting

### "Unable to find franka_description"

**Solution**: Install franka_description package:
```bash
sudo apt install ros-jazzy-franka-description
```

Or build from source if using custom version.

### Camera Not Publishing in Gazebo

Check that the Gazebo plugin for the camera is loaded:

```bash
# List active Gazebo plugins
gz plugin --list

# Should see: ros_gz_bridge, ros_gz_image, etc.
```

**Solution**: Ensure `ros_gz_sim` and `ros_gz_bridge` packages installed:
```bash
sudo apt install ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge
```

### Controllers Not Loading

Verify that controller YAML files exist and are referenced correctly in xacro:

```bash
# Check if files exist
ls ~/Projects/ros2_ws/install/table_tennis_description/share/table_tennis_description/config/

# Should see: red_controllers.yaml, green_controllers.yaml
```

### Mesh Files Not Found

Meshes must be installed during build. Check `CMakeLists.txt`:

```cmake
install(DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}
)
```

### URDF Parse Errors

Common issues:
1. **Missing xmlns declarations** in xacro files
2. **Incorrect xacro syntax** (missing `${}` for variables)
3. **Circular dependencies** in includes
4. **Invalid joint types** or connections

**Debug**:
```bash
# Generate URDF from xacro and check
xacro install/table_tennis_description/share/table_tennis_description/urdf/robots/franka_with_paddle.urdf.xacro > debug.urdf
check_urdf debug.urdf
```

## Integration Examples

### With MoveIt 2

```xml
<!-- In your MoveIt config package -->
<xacro:include filename="$(find table_tennis_description)/urdf/robots/franka_with_paddle.urdf.xacro"/>

<xacro:franka_with_paddle 
  arm_id="panda"
  use_ros2_control="true"
/>

<!-- Define planning groups, end effectors, etc. -->
```

### With Navigation Stack

```xml
<!-- For mobile manipulator -->
<robot name="mobile_table_tennis">
  <xacro:include filename="$(find mobile_base_description)/urdf/base.urdf.xacro"/>
  <xacro:include filename="$(find table_tennis_description)/urdf/robots/franka_with_paddle.urdf.xacro"/>
  
  <xacro:mobile_base base_name="base"/>
  
  <xacro:franka_with_paddle 
    arm_id="arm"
    robot_origin_x="0.3"
    robot_origin_z="0.2"
  />
  
  <joint name="base_to_arm" type="fixed">
    <parent link="base_link"/>
    <child link="arm_link0"/>
    <origin xyz="0.3 0.0 0.2"/>
  </joint>
</robot>
```

### With Custom World

```xml
<!-- In your Gazebo world file -->
<world name="custom_arena">
  <include>
    <uri>model://sun</uri>
  </include>
  
  <!-- Your custom environment -->
  
  <!-- Robots will be spawned via launch file -->
</world>
```

## Performance Considerations

### Mesh Complexity
- **Visual meshes**: Can be high-poly for realism
- **Collision meshes**: Should be simplified (< 1000 triangles)
- Use collision primitives (box, cylinder, sphere) when possible

### Joint Update Rate
- Simulation: 100-1000 Hz (configured in controller manager)
- Real hardware: 1000 Hz
- Camera: 30 Hz typical

### Multi-Robot Scenarios
- Use namespaces to avoid topic/service conflicts
- Separate TF trees per robot (connected at world frame)
- Individual controller managers per robot

## Dependencies

**Required**:
- franka_description
- xacro
- urdf
- robot_state_publisher

**Optional** (for full functionality):
- ros2_control
- ros2_controllers
- gz_ros2_control (for Gazebo simulation)
- rviz2 (for visualization)

## See Also

- [table_tennis_gazebo](../table_tennis_gazebo/README.md) - Gazebo simulation using these descriptions
- [franka_description](https://github.com/frankaemika/franka_ros2/tree/humble/franka_description) - Base Franka robot descriptions
- [Main Workspace README](../../README.md) - Complete setup guide

## Contributing

When adding new end effectors or sensors:
1. Create modular xacro macros
2. Include proper inertial properties
3. Add collision geometry
4. Document parameters
5. Test in Gazebo and RViz
6. Update this README

## License

[Add your license here]

## Maintainers

[Add maintainer information]

---

**Package**: table_tennis_description  
**Version**: 0.1.0  
**ROS 2 Distro**: Jazzy  
**Last Updated**: February 23, 2026
