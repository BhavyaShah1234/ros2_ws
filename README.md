# Table Tennis Dual Robot Simulation - ROS 2 Workspace

A comprehensive dual-robot table tennis simulation using ROS 2 Jazzy and Gazebo Harmonic with Franka FR3 robots.

## Overview

This workspace provides a realistic table tennis simulation environment featuring:
- **Dual Franka FR3 robots** with position control (red and green teams)
- **Ball spawner action server** with physics-based tracking and auto-destruction
- **Multiple RGBD cameras**: arm-mounted, overhead, and side views
- **Unified Rviz visualization** showing both robots, cameras, and ball
- **Real-time TF transforms** with proper namespacing

## First-Time Setup on New Machine

Follow these steps when cloning this workspace to a new machine to ensure ros2_control and all dependencies are properly configured.

### Step 1: Install ROS 2 Jazzy

```bash
# Set up ROS 2 apt repository
sudo apt update && sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install -y curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Jazzy Desktop
sudo apt update
sudo apt install -y ros-jazzy-desktop
```

### Step 2: Install Gazebo Harmonic

```bash
# Add Gazebo repository
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# Install Gazebo Harmonic
sudo apt update
sudo apt install -y gz-harmonic ros-jazzy-ros-gz
```

### Step 3: Install ros2_control and Dependencies

**This is the critical step for controller activation!**

```bash
# Install ros2_control packages
sudo apt install -y \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-gz-ros2-control \
  ros-jazzy-controller-manager \
  ros-jazzy-joint-state-broadcaster \
  ros-jazzy-joint-trajectory-controller \
  ros-jazzy-position-controllers \
  ros-jazzy-effort-controllers \
  ros-jazzy-velocity-controllers

# Install additional ROS 2 tools
sudo apt install -y \
  ros-jazzy-xacro \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-joint-state-publisher \
  ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-rviz2 \
  ros-jazzy-moveit \
  python3-colcon-common-extensions \
  python3-vcstool \
  python3-rosdep
```

### Step 4: Clone and Build Workspace

```bash
# Clone the workspace
cd ~
git clone https://github.com/BhavyaShah1234/ros2_ws
cd ros2_ws

# Initialize rosdep (if first time on this machine)
sudo rosdep init || true
rosdep update

# Install workspace dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### Step 5: Configure bashrc

Add these lines to your `~/.bashrc`:

```bash
# ROS 2 Jazzy setup
source /opt/ros/jazzy/setup.bash

# Workspace setup
source ~/ros2_ws/install/setup.bash

# Gazebo resource paths
export GZ_SIM_RESOURCE_PATH=$HOME/ros2_ws/install/table_tennis_gazebo/share/table_tennis_gazebo/models:$HOME/ros2_ws/install/franka_description/share/franka_description:$HOME/ros2_ws/install/table_tennis_description/share/table_tennis_description:$GZ_SIM_RESOURCE_PATH

# Colcon shortcuts (optional)
source /usr/share/colcon_cd/function/colcon_cd.sh
export _colcon_cd_root=/opt/ros/jazzy/
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
```

**Important**: If you have CUDA or other library paths, append them to `LD_LIBRARY_PATH` instead of overwriting:

```bash
# CORRECT: Append to existing path
export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH

# WRONG: This breaks ROS 2!
# export LD_LIBRARY_PATH=/usr/local/cuda/lib64
```

Then source the file:

```bash
source ~/.bashrc
```

### Step 6: Verify Installation

Test that controllers are available:

```bash
# Start a terminal and verify ros2_control plugins
ros2 pkg list | grep ros2_control
ros2 pkg list | grep controller

# Check for critical packages (should see all of these)
ros2 pkg list | grep -E "ros2_control|joint_trajectory_controller|gz_ros2_control"
```

Expected output should include:
- `controller_manager`
- `joint_trajectory_controller`
- `gz_ros2_control`
- `ros2_control`
- `ros2_controllers`

### Step 7: Test Launch

```bash
# Clean any existing processes
pkill -9 -f "gz|rviz|simulation"

# Launch simulation
cd ~/ros2_ws
colcon build --symlink-install && source ~/.bashrc
ros2 launch table_tennis_gazebo simulation.launch.py > test.log 2>&1 &

# Wait for initialization
sleep 25

# Verify controllers are loaded
ros2 control list_controllers

# Should see (after ~25 seconds):
# red/arm_controller[joint_trajectory_controller/JointTrajectoryController] active
# green/arm_controller[joint_trajectory_controller/JointTrajectoryController] active
# red/joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
# green/joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
```

### Troubleshooting Controllers

**Issue**: Controllers not activating or "Error" status in RViz RobotModel

**Common Causes & Solutions**:

1. **Missing ros2_control packages**
   ```bash
   # Install missing packages
   sudo apt install -y ros-jazzy-gz-ros2-control ros-jazzy-controller-manager
   colcon build --symlink-install
   source ~/.bashrc
   ```

2. **LD_LIBRARY_PATH overwritten**
   ```bash
   # Check if ROS libraries are in path
   echo $LD_LIBRARY_PATH | grep ros
   
   # If not found, fix ~/.bashrc (see Step 5 above)
   # Then: source ~/.bashrc
   ```

3. **Controllers timing out during spawn**
   ```bash
   # Check controller manager logs
   grep -i "controller_manager" test.log
   
   # Common fix: Wait longer for Gazebo initialization
   sleep 30  # Instead of 25
   ```

4. **URDF not loading in RViz**
   ```bash
   # Verify robot_description topic
   ros2 topic list | grep robot_description
   
   # Check if robot_state_publisher is running
   ros2 node list | grep robot_state_publisher
   
   # Rebuild with clean install
   rm -rf build install log
   colcon build --symlink-install
   source ~/.bashrc
   ```

5. **Gazebo plugin not found**
   ```bash
   # Check for gz_ros2_control plugin
   gz sim --help | grep ros2_control
   
   # Reinstall if missing
   sudo apt install --reinstall ros-jazzy-gz-ros2-control
   ```

6. **Controller spawner fails**
   ```bash
   # Check spawner logs
   grep "spawner" test.log
   
   # Often needs GZ_SIM_RESOURCE_PATH set correctly
   export GZ_SIM_RESOURCE_PATH=$HOME/ros2_ws/install/table_tennis_gazebo/share/table_tennis_gazebo/models:$HOME/ros2_ws/install/franka_description/share/franka_description:$GZ_SIM_RESOURCE_PATH
   ```

**Verification after fixes**:
```bash
# Clean everything
pkill -9 -f "gz|rviz|simulation"
rm -rf build install log

# Full rebuild
colcon build --symlink-install
source ~/.bashrc

# Launch and verify
ros2 launch table_tennis_gazebo simulation.launch.py > test.log 2>&1 &
sleep 30
ros2 control list_controllers

# Test robot movement
ros2 action send_goal /red/arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{trajectory: {joint_names: [fr3_joint1, fr3_joint2, fr3_joint3, fr3_joint4, fr3_joint5, fr3_joint6, fr3_joint7], points: [{positions: [0.0, -0.3, 0.0, -1.5, 0.0, 1.2, 0.785], time_from_start: {sec: 2}}]}}"
```

## Quick Start

### Prerequisites
- ROS 2 Jazzy
- Gazebo Harmonic (gz-sim 8)
- Python 3.10+

### Build and Launch

```bash
# Clean any existing processes
pkill -9 -f "gz|rviz|simulation|test.launch"

# Build and launch simulation
cd ~/Projects/ros2_ws
colcon build --symlink-install && source ~/.bashrc && ros2 launch table_tennis_gazebo simulation.launch.py > simulation.log 2>&1 &

# Wait for initialization (~25 seconds)
sleep 25

# Check logs for errors
cat simulation.log | grep -i error
```

## Features

### 1. Ball Spawner Action Server

The ball spawner uses an **action server** (not a service) providing real-time feedback and automatic cleanup.

**Spawn a ball**:
```bash
ros2 action send_goal /spawn_ball table_tennis_gazebo/action/SpawnBall "{x: 0.0, y: 0.6, z: 3.0}" --feedback
```

**Action capabilities**:
- Real-time position and velocity feedback
- Automatic destruction when ball hits ground (z < 0.05m) or below table (z < 0.65m)
- Automatic destruction when stationary (velocity < 0.005 m/s for >1 second)
- Only one ball allowed in play at a time
- Final statistics: time alive, final position, destruction reason

**Monitor ball state**:
```bash
ros2 topic echo /ball/status        # "alive" or "dead"
ros2 topic echo /ball/pose          # Current 3D position
```

### 2. Robot Control

Both robots use `follow_joint_trajectory` action servers with namespaces:

**Red robot movement**:
```bash
ros2 action send_goal /red/arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{
  trajectory: {
    joint_names: [fr3_joint1, fr3_joint2, fr3_joint3, fr3_joint4, fr3_joint5, fr3_joint6, fr3_joint7],
    points: [
      { positions: [0.0, -0.3, 0.0, -1.5, 0.0, 1.2, 0.785], time_from_start: { sec: 2 } }
    ]
  }
}"
```

**Green robot movement**:
```bash
ros2 action send_goal /green/arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{
  trajectory: {
    joint_names: [fr3_joint1, fr3_joint2, fr3_joint3, fr3_joint4, fr3_joint5, fr3_joint6, fr3_joint7],
    points: [
      { positions: [0.0, 0.3, 0.0, -1.2, 0.0, 1.5, -0.785], time_from_start: { sec: 2 } }
    ]
  }
}"
```

**Multi-waypoint trajectory**:
```bash
ros2 action send_goal /red/arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{
  trajectory: {
    joint_names: [fr3_joint1, fr3_joint2, fr3_joint3, fr3_joint4, fr3_joint5, fr3_joint6, fr3_joint7],
    points: [
      { positions: [0.0, -0.3, 0.0, -1.5, 0.0, 1.2, 0.785], time_from_start: { sec: 1 } },
      { positions: [0.3, -0.5, 0.2, -1.2, 0.0, 1.5, 0.0], time_from_start: { sec: 3 } },
      { positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], time_from_start: { sec: 5 } }
    ]
  }
}"
```

### 3. Camera System

**Available Topics**:
```bash
# Arm-mounted RGBD cameras
/red/fr3_arm_cam/image              # Red robot RGB
/red/fr3_arm_cam/depth_image        # Red robot depth
/green/fr3_arm_cam/image            # Green robot RGB  
/green/fr3_arm_cam/depth_image      # Green robot depth

# Fixed scene cameras
/overhead_cam/image                 # Overhead RGB
/overhead_cam/depth_image           # Overhead depth
/side_cam/image                     # Side view RGB
/side_cam/depth_image               # Side depth
```

**Test camera feeds**:
```bash
ros2 topic hz /overhead_cam/image
ros2 topic hz /side_cam/image
```

### 4. Rviz Visualization

The simulation launches with a unified Rviz window showing both robots.

#### Configuring Rviz Displays

**For Robot Models**:
- Display Type: `RobotModel`
- Description Topic: `/red/robot_description` or `/green/robot_description`
- TF Prefix: Leave empty (frames already have red/ or green/ prefix)

**For Camera Images (RGB)**:
- Display Type: `Image`
- Image Topic: Select any RGB topic (e.g., `/overhead_cam/image`)
- Transport Hint: `raw` or `compressed`

**For Depth Images**:
- Display Type: `Image`  
- Image Topic: Select depth topic (e.g., `/overhead_cam/depth_image`)
- Normalize Range: ✓ Checked (for better visualization)
- Min Value: 0.0
- Max Value: 10.0

**For Point Clouds** (alternative for depth):
- Display Type: `DepthCloud`
- Depth Map Topic: Select depth topic
- Color Image Topic: Select corresponding RGB topic

**TF Configuration**:
- Fixed Frame: `world` (to see both robots) or `red/world` (for red robot view)
- If you see yellow exclamation marks on `fr3_*` frames, that's normal - use the namespaced frames `red/fr3_*` and `green/fr3_*` instead

#### Common Rviz Issues

**Both robots appear at same position**:
1. Check Fixed Frame is set to `world`
2. Verify TF tree: `ros2 run tf2_tools view_frames`
3. Both robots spawn at x=-1.57 (red) and x=1.57 (green), facing each other

**TF frames with yellow exclamation**:
- `fr3_link0`, `fr3_link1`, etc. (yellow ⚠️) - unprefixed frames, ignore these
- `red/fr3_link0`, `green/fr3_link0`, etc. (green ✓) - correct namespaced frames, use these

**Depth camera shows black/white artifacts**:
- This is normal for simulated RGBD cameras in Gazebo
- Depth values are accurate despite noisy visualization
- Use `DepthCloud` or Point Cloud displays for cleaner visualization
- Or apply `depth_image_proc` filters

## Complete Test Sequence

```bash
# 1. Clean environment
pkill -9 -f "gz|rviz|simulation|test.launch"

# 2. Build and launch
cd ~/Projects/ros2_ws
colcon build --symlink-install && source ~/.bashrc && ros2 launch table_tennis_gazebo simulation.launch.py > simulation.log 2>&1 &

# 3. Wait for initialization
sleep 25

# 4. Verify action servers are available
ros2 action list
# Expected output:
#   /spawn_ball
#   /red/arm_controller/follow_joint_trajectory
#   /green/arm_controller/follow_joint_trajectory

# 5. Check controllers
ros2 control list_controllers --controller-manager /red/controller_manager
ros2 control list_controllers --controller-manager /green/controller_manager
# Expected: joint_state_broadcaster and arm_controller both "active"

# 6. Test ball spawning
ros2 action send_goal /spawn_ball table_tennis_gazebo/action/SpawnBall "{x: 0.0, y: 0.6, z: 3.0}" --feedback

# 7. Test red robot movement
ros2 action send_goal /red/arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{trajectory: {joint_names: [fr3_joint1, fr3_joint2, fr3_joint3, fr3_joint4, fr3_joint5, fr3_joint6, fr3_joint7], points: [{positions: [0.0, -0.3, 0.0, -1.5, 0.0, 1.2, 0.785], time_from_start: {sec: 2}}]}}"

# 8. Test green robot movement  
ros2 action send_goal /green/arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{trajectory: {joint_names: [fr3_joint1, fr3_joint2, fr3_joint3, fr3_joint4, fr3_joint5, fr3_joint6, fr3_joint7], points: [{positions: [0.0, 0.3, 0.0, -1.2, 0.0, 1.5, -0.785], time_from_start: {sec: 2}}]}}"

# 9. Monitor ball during play
ros2 topic echo /ball/status
ros2 topic echo /ball/pose

# 10. Verify camera feeds
ros2 topic hz /overhead_cam/image
ros2 topic hz /side_cam/image

# 11. Check TF tree
ros2 run tf2_tools view_frames
# View generated PDF: frames_*.pdf

# 12. Inspect logs if issues
cat simulation.log | grep -i "error\|warning" | tail -20
```

## Troubleshooting

### Ball Not Destroying

**Issue**: Ball doesn't disappear when hitting ground or becoming stationary

**Debug**:
```bash
# Check ball status
ros2 topic echo /ball/status

# Check ball pose (should show z coordinate)
ros2 topic echo /ball/pose

# Check action server feedback
ros2 action send_goal /spawn_ball table_tennis_gazebo/action/SpawnBall "{x: 0.0, y: 0.0, z: 2.0}" --feedback
```

**Ball destruction conditions**:
- Ground hit: z < 0.05m (ball radius is 0.02m)
- Below table: z < 0.65m (table top at 0.76m)
- Stationary: velocity < 0.005 m/s for more than 1 second

**Known Issue**: The Python ball spawner's pose monitoring via `gz topic` subprocess may not update properly. This is a known limitation and is being worked on.

### Controllers Not Active

```bash
# Check if controller manager is running
ros2 control list_controllers --controller-manager /red/controller_manager

# Check hardware interfaces
ros2 control list_hardware_interfaces --controller-manager /red/controller_manager

# Reload controllers if needed
ros2 control set_controller_state arm_controller inactive --controller-manager /red/controller_manager
ros2 control set_controller_state arm_controller active --controller-manager /red/controller_manager
```

### TF Warnings in Rviz

The yellow exclamation marks on unprefixed frames (`fr3_link*`) are expected. Use the namespaced versions:
- ✓ `red/fr3_link0`, `red/fr3_link1`, ..., `red/fr3_paddle`
- ✓ `green/fr3_link0`, `green/fr3_link1`, ..., `green/fr3_paddle`

### Gazebo/Rviz Won't Start

```bash
# Kill all related processes
pkill -9 -f "gz|rviz|simulation|test.launch"

# Clear Gazebo cache if corrupted
rm -rf ~/.gz/sim

# Relaunch
colcon build --symlink-install && source ~/.bashrc && ros2 launch table_tennis_gazebo simulation.launch.py > simulation.log 2>&1 &
```

## Project Structure

```
ros2_ws/
├── src/
│   ├── table_tennis_gazebo/          # Main simulation package
│   │   ├── launch/
│   │   │   ├── simulation.launch.py  # Dual robot simulation
│   │   │   ├── test.launch.py        # Single robot test
│   │   │   └── ball_spawn.launch.py  # Ball spawning client
│   │   ├── scripts/
│   │   │   ├── ball_spawner_node.py  # Action server for ball
│   │   │   └── ball_spawn_client.py  # Action client example
│   │   ├── worlds/
│   │   │   └── arena.world           # Gazebo world with table
│   │   ├── rviz/
│   │   │   ├── dual_robots.rviz      # Unified visualization
│   │   │   ├── red_robot.rviz        # Red robot only
│   │   │   └── green_robot.rviz      # Green robot only
│   │   ├── config/
│   │   │   ├── ros_gz_bridge_dual.yaml
│   │   │   └── controllers_namespaced.yaml
│   │   └── action/
│   │       └── SpawnBall.action      # Ball spawner interface
│   ├── franka_ros2/                  # Franka robot packages
│   ├── franka_description/           # Robot URDF models
│   └── libfranka/                    # Franka control library
├── FIXES_AND_USAGE.md               # Detailed fixes documentation
└── README.md                         # This file
```

## Known Issues and Fixes

### Issue 1: Ball Spawn Service vs Action
**Fixed**: Use action server, not service:
```bash
# OLD (doesn't work):
ros2 service call /ball_spawner/spawn_ball ...

# NEW (correct):
ros2 action send_goal /spawn_ball table_tennis_gazebo/action/SpawnBall "{x: 0.0, y: 0.6, z: 3.0}"
```

### Issue 2: Robot Commands Need Namespaces
**Fixed**: Dual robot simulation requires `/red/` or `/green/` prefix:
```bash
# Dual robot simulation:
/red/arm_controller/follow_joint_trajectory
/green/arm_controller/follow_joint_trajectory

# Single robot (test.launch.py):
/arm_controller/follow_joint_trajectory
```

### Issue 3: Parameter Typo in Ball Spawn
**Fixed**: Use proper minus sign:
```bash
# Wrong: x:=0.-7
# Correct: x:=-0.7
```

### Issue 4: Cameras Not Visible
**Fixed**: Added to ros_gz_bridge_dual.yaml. Cameras publish with lazy mode (only when subscribed).

### Issue 5: Unified Rviz
**Changed**: Merged two separate Rviz windows into one `dual_robots.rviz` config.

## Development

### Adding New Cameras
Edit `config/ros_gz_bridge_dual.yaml` to add camera bridges:
```yaml
- ros_topic_name: "my_camera/image"
  gz_topic_name: "my_camera/image"
  ros_type_name: "sensor_msgs/msg/Image"
  gz_type_name: "gz.msgs.Image"
  direction: GZ_TO_ROS
  lazy: true
```

### Modifying Robot Controllers
Edit `config/controllers_namespaced.yaml` for controller parameters.

### Custom Ball Physics
Modify the SDF content in `scripts/ball_spawner_node.py`:
- Mass: Default 0.0027 kg (2.7g)
- Radius: 0.020 m (40mm diameter)
- Restitution: 0.85 (bounce coefficient)

## References

- [ROS 2 Jazzy Documentation](https://docs.ros.org/en/jazzy/)
- [Gazebo Harmonic](https://gazebosim.org/docs/harmonic)
- [Franka Robotics](https://frankaemika.github.io/docs/)
- [ros2_control](https://control.ros.org/)

## License

See individual package licenses:
- franka_ros2: Apache 2.0
- table_tennis_gazebo: Apache 2.0

## Support

For issues specific to this simulation, check:
1. `simulation.log` for errors
2. FIXES_AND_USAGE.md for detailed troubleshooting
3. Gazebo/Rviz console output

---

**Last Updated**: February 6, 2026
