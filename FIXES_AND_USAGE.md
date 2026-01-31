# Table Tennis Gazebo - Usage Guide and Fixes

## Fixed Issues (Jan 31, 2026)

### 1. Ball Spawn Command Fix
**Issue**: Parameter type error when spawning ball
```bash
# WRONG (typo):
ros2 launch table_tennis_gazebo ball_spawn.launch.py x:=0.-7 y:=0.0 z:=8.0

# CORRECT:
ros2 launch table_tennis_gazebo ball_spawn.launch.py x:=-0.7 y:=0.0 z:=8.0
```
**Fix**: Use proper minus sign `-0.7` instead of period `0.-7`

### 2. Robot Movement Commands (simulation.launch.py)
**Issue**: Commands don't work without namespace prefix

In the dual robot simulation, each robot has its own namespace (`red` and `green`).

```bash
# Move RED robot:
ros2 action send_goal /red/arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{
  trajectory: {
    joint_names: [fr3_joint1, fr3_joint2, fr3_joint3, fr3_joint4, fr3_joint5, fr3_joint6, fr3_joint7],
    points: [
      { positions: [0.0, -0.3, 0.0, -1.5, 0.0, 1.2, 0.785], time_from_start: { sec: 2 } }
    ]
  }
}"

# Move GREEN robot:
ros2 action send_goal /green/arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{
  trajectory: {
    joint_names: [fr3_joint1, fr3_joint2, fr3_joint3, fr3_joint4, fr3_joint5, fr3_joint6, fr3_joint7],
    points: [
      { positions: [0.0, 0.3, 0.0, -1.2, 0.0, 1.5, -0.785], time_from_start: { sec: 2 } }
    ]
  }
}"

# For single robot (test.launch.py):
ros2 action send_goal /arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{
  trajectory: {
    joint_names: [fr3_joint1, fr3_joint2, fr3_joint3, fr3_joint4, fr3_joint5, fr3_joint6, fr3_joint7],
    points: [
      { positions: [0.0, -0.3, 0.0, -1.5, 0.0, 1.2, 0.785], time_from_start: { sec: 2 } }
    ]
  }
}"
```

### 3. Overhead and Side Camera Feeds
**Issue**: Cameras not visible in Rviz
**Fix**: Added overhead_cam and side_cam to ros_gz_bridge_dual.yaml

After rebuilding and restarting simulation:
```bash
# Check camera topics:
ros2 topic list | grep cam

# Test camera feed:
ros2 topic hz /overhead_cam/image
ros2 topic hz /side_cam/image

# View in Rviz:
# Add Image display, set topic to /overhead_cam/image or /side_cam/image
```

### 4. TF Frame Issues
**Issue**: `No transform for [fr3_<some link>] to [<red or green>/world]`

**Root Cause**: Frame prefixes are set in robot_state_publisher but TF tree needs proper configuration.

**TF Frame Structure**:
- Red robot frames: `red/world`, `red/fr3_link0`, `red/fr3_link1`, ..., `red/fr3_paddle`
- Green robot frames: `green/world`, `green/fr3_link0`, `green/fr3_link1`, ..., `green/fr3_paddle`

**Check TF tree**:
```bash
# For red robot:
ros2 run tf2_ros tf2_echo red/world red/fr3_link0

# For green robot:
ros2 run tf2_ros tf2_echo green/world green/fr3_link0

# View full TF tree:
ros2 run tf2_tools view_frames
```

**Rviz Configuration**:
- Set **Fixed Frame** to `red/world` for red robot view
- Set **Fixed Frame** to `green/world` for green robot view
- RobotModel display should use robot_description from `/red/robot_description` or `/green/robot_description`

## Quick Verification Commands

### For simulation.launch.py (Dual Robots):
```bash
# Launch
ros2 launch table_tennis_gazebo simulation.launch.py

# Check controllers (wait ~20 seconds):
ros2 control list_controllers --controller-manager /red/controller_manager
ros2 control list_controllers --controller-manager /green/controller_manager

# Check camera topics:
timeout 5 ros2 topic hz /overhead_cam/image
timeout 5 ros2 topic hz /side_cam/image

# Check TF:
ros2 run tf2_ros tf2_echo red/world red/fr3_paddle
ros2 run tf2_ros tf2_echo green/world green/fr3_paddle

# Test movement:
ros2 action send_goal /red/arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{trajectory: {joint_names: [fr3_joint1, fr3_joint2, fr3_joint3, fr3_joint4, fr3_joint5, fr3_joint6, fr3_joint7], points: [{positions: [0.0, -0.3, 0.0, -1.5, 0.0, 1.2, 0.785], time_from_start: {sec: 2}}]}}"
```

### For test.launch.py (Single Robot):
```bash
# Launch
ros2 launch table_tennis_gazebo test.launch.py

# Check controllers:
ros2 control list_controllers

# Check camera:
timeout 5 ros2 topic hz /fr3_arm_cam/image

# Check TF:
ros2 run tf2_ros tf2_echo world fr3_paddle

# Test movement:
ros2 action send_goal /arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{trajectory: {joint_names: [fr3_joint1, fr3_joint2, fr3_joint3, fr3_joint4, fr3_joint5, fr3_joint6, fr3_joint7], points: [{positions: [0.0, -0.3, 0.0, -1.5, 0.0, 1.2, 0.785], time_from_start: {sec: 2}}]}}"
```

## Common Issues and Solutions

### Issue: "Could not contact service /controller_manager/list_controllers"
**Solution**: Add namespace for dual robot simulation
```bash
# Instead of:
ros2 control list_controllers

# Use:
ros2 control list_controllers --controller-manager /red/controller_manager
# OR
ros2 control list_controllers --controller-manager /green/controller_manager
```

### Issue: Camera topics don't appear in `ros2 topic list`
**Solution**: Topics use lazy publishing. Subscribe to trigger publishing:
```bash
ros2 topic hz /overhead_cam/image
# OR
ros2 topic echo /overhead_cam/image --once
```

### Issue: Time jumps in Rviz
**Solution**: Already fixed with 15-second Rviz delays. If still occurs, increase delay in launch file.

### Issue: Robots not moving
**Solution**: Check controller state and use correct namespace
```bash
# Check controller state:
ros2 control list_controllers --controller-manager /red/controller_manager

# Ensure it shows "active" state
# Use namespaced action topics: /red/arm_controller/ or /green/arm_controller/
```

## Build and Launch
```bash
cd ~/Projects/ros2_ws
colcon build --symlink-install
source ~/.bashrc

# Single robot test:
ros2 launch table_tennis_gazebo test.launch.py

# Dual robot simulation:
ros2 launch table_tennis_gazebo simulation.launch.py
```

## Key Files Modified
- `config/ros_gz_bridge.yaml` - Single robot camera bridge config
- `config/ros_gz_bridge_dual.yaml` - Dual robot + overhead/side cameras config
- `launch/test.launch.py` - 15s Rviz delay, unified bridge
- `launch/simulation.launch.py` - 15s/15.5s Rviz delays
- `launch/bridge.launch.py` - Unified YAML-based bridge

## References
- Gazebo Harmonic + ROS 2 Jazzy integration uses YAML config for ros_gz_bridge
- Lazy publishing means topics only appear when subscribed
- Frame prefixes (red/, green/) prevent TF collisions in multi-robot setups
