# Franka FR3 Pick-and-Place Simulation - Complete Documentation  

## Status: ✅ WORKING

### What's Working
✅ **ALL CONTROLLERS ACTIVE** - Successfully loading and activating without timeouts  
✅ **Joint State Broadcasting** - `/joint_states` publishing all 9 joints (7 arm + 2 gripper)  
✅ **Gazebo Simulation** - Robot spawned and visible  
✅ **Hardware Plugin** - `gz_ros2_control` plugin loading correctly  
✅ **RViz** - Visualization running  
✅ **Camera Bridges** - ROS-Gazebo bridges for overhead camera active  

### Critical Fix Applied

**Problem:** Controllers were timing out after 5 seconds during activation  
**Root Cause:** ABI mismatch between system-installed `gz_ros2_control` and `hardware_interface`  
**Solution:** Rebuilt `gz_ros2_control` from source in workspace overlay

### Launch Command
```bash
cd /home/bhavya-shah/Projects/ros2_ws
source install/setup.bash
ros2 launch simple_pick_and_place_gazebo simulation.launch.py
```

### Controller Status
```
$ ros2 control list_controllers
gripper_controller      position_controllers/GripperActionController           active
arm_controller          joint_trajectory_controller/JointTrajectoryController  active
joint_state_broadcaster joint_state_broadcaster/JointStateBroadcaster          active
```

### Joint States
All 9 joints publishing on `/joint_states`:
- fr3_joint1 through fr3_joint7 (arm)
- fr3_finger_joint1, fr3_finger_joint2 (gripper)

### Architecture

**Launch Sequence:**
1. t=0s: Gazebo launches with world file
2. t=2s: `robot_state_publisher` starts
3. t=3s: Camera bridges start  
4. t=4s: Robot spawns in Gazebo
5. t=6s: `joint_state_broadcaster` loads/activates
6. t=9s: `arm_controller` loads/activates
7. t=12s: `gripper_controller` loads/activates
8. t=15s: RViz launches

**Key Configuration Changes:**
- `gazebo='true'` - Enables gz_ros2_control plugin
- `use_fake_hardware='false'` - Uses Gazebo hardware, not mock
- **NO ros2_control_node** - Gazebo plugin provides controller_manager internally

### Files Structure
```
ros2_ws/
├── src/
│   ├── gz_ros2_control/           # Rebuilt from source (CRITICAL!)
│   ├── simple_pick_and_place_description/
│   │   ├── urdf/
│   │   │   ├── robot/franka_pick_and_place.urdf.xacro
│   │   │   └── control/
│   │   │       ├── ros2_control.urdf.xacro
│   │   │       └── gazebo_ros2_control.urdf.xacro
│   │   └── rviz/pick_and_place.rviz
│   └── simple_pick_and_place_gazebo/
│       ├── launch/simulation.launch.py
│       ├── config/controllers.yaml
│       └── worlds/pick_and_place.world
```

### Known Minor Issues
⚠️ **TF Tree Incomplete** - No static transform from `world` to `fr3_link0` (doesn't affect controller operation)  
⚠️ **Clock Warnings** - "No clock received" warnings from controller_manager (harmless, uses sim_time)  

### Testing Robot Control

**Arm Controller:**
```bash
ros2 action send_goal /arm_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{
    trajectory: {
      joint_names: [fr3_joint1, fr3_joint2, fr3_joint3, fr3_joint4, fr3_joint5, fr3_joint6, fr3_joint7],
      points: [{
        positions: [0.0, -0.5, 0.0, -2.0, 0.0, 1.5, 0.785],
        time_from_start: {sec: 2}
      }]
    }
  }"
```

**Gripper Controller:**
```bash
ros2 action send_goal /gripper_controller/gripper_cmd \
  control_msgs/action/GripperCommand \
  "{command: {position: 0.08, max_effort: 100.0}}"
```

### Topics
- `/joint_states` - Joint positions/velocities
- `/overhead_camera/image_raw` - Camera image (sensor_msgs/Image)
- `/overhead_camera/camera_info` - Camera parameters
- `/arm_controller/joint_trajectory` - Arm commands
- `/gripper_controller/gripper_cmd` - Gripper commands

### Cleanup Commands

**Kill all simulation processes:**
```bash
pkill -9 -f "gz sim|ros2 launch|ros_gz_bridge|robot_state_publisher|spawner|rviz2"
```

**Clean rebuild:**
```bash
cd /home/bhavya-shah/Projects/ros2_ws
rm -rf build/ install/ log/
colcon build --symlink-install
source install/setup.bash
```

### References
- ROS 2 Control Demos Example 9: https://control.ros.org/jazzy/doc/ros2_control_demos/example_9/doc/userdoc.html
- AutomaticAddison Tutorial: https://automaticaddison.com/how-to-simulate-a-robotic-arm-in-gazebo-ros-2-jazzy/
- gz_ros2_control: https://github.com/ros-controls/gz_ros2_control

### Build Date
- Project Created: December 2025
- Controller Fix Applied: January 29, 2026
- `gz_ros2_control` rebuilt from: jazzy branch @ latest
