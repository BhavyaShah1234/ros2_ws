# Table Tennis Gazebo - Usage Guide and Fixes

## Overview

This workspace provides a dual-robot table tennis simulation in Gazebo with ROS 2 Jazzy.

**Key Features**:
- Dual Franka FR3 robots with position control
- Ball spawner action server for physics-based ball tracking
- RGBD cameras (arm-mounted, overhead, side view)
- Unified Rviz visualization
- Real-time TF transforms and sensor data

## Action Servers

### Ball Spawner Action Server

The ball spawner uses an **action server** (not a service) to spawn and track balls with feedback.

**Action**: `/spawn_ball` (table_tennis_gazebo/action/SpawnBall)

**Benefits of Action Server**:
- Real-time feedback on ball position and velocity
- Automatic ball destruction when hitting floor or becoming stationary
- Only allows one ball in play at a time
- Provides final statistics when ball "dies"

**Usage**:
```bash
# Spawn a ball at position (x, y, z):
ros2 action send_goal /spawn_ball table_tennis_gazebo/action/SpawnBall "{x: 0.0, y: 0.6, z: 3.0}"

# With feedback (verbose):
ros2 action send_goal /spawn_ball table_tennis_gazebo/action/SpawnBall "{x: -0.7, y: 0.0, z: 2.5}" --feedback

# Check action server status:
ros2 action list
ros2 action info /spawn_ball

# Monitor ball state:
ros2 topic echo /ball/status    # "alive" or "dead"
ros2 topic echo /ball/pose      # Current position
```

**Action Goal Fields**:
- `x`, `y`, `z`: Float64 - 3D position to spawn ball

**Action Feedback** (during execution):
- `status`: "alive" or "dead"
- `current_x`, `current_y`, `current_z`: Current ball position

**Action Result** (when complete):
- `final_status`: Reason for ball death ("hit_ground", "stationary", etc.)
- `final_x`, `final_y`, `final_z`: Final position
- `time_alive`: Duration ball was active (seconds)

## Robot Movement Commands

### Using Action Client (Recommended)

Both robots use `follow_joint_trajectory` action for smooth motion.

```bash
# Move RED robot to specific joint positions:
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

# Multi-waypoint trajectory:
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

### Available Actions:
```bash
# List all action servers:
ros2 action list

# Expected actions:
# /spawn_ball                                      # Ball spawner
# /red/arm_controller/follow_joint_trajectory     # Red robot motion
# /green/arm_controller/follow_joint_trajectory   # Green robot motion
```

## Fixed Issues (Jan 31, 2026)

### 1. Ball Spawn - Using Action Server (Not Service)
**Previous Issue**: Used service call that would hang
```bash
# OLD (service - doesn't work):
ros2 service call /ball_spawner/spawn_ball table_tennis_gazebo/srv/SpawnBall "..."

# NEW (action - correct):
ros2 action send_goal /spawn_ball table_tennis_gazebo/action/SpawnBall "{x: -0.7, y: 0.0, z: 2.5}"
```

### 2. Ball Spawn Parameter Typo
**Issue**: Parameter type error when spawning ball
```bash
# WRONG (typo with period before minus):
x:=0.-7

# CORRECT:
x:=-0.7
```

### 3. Robot Controller Namespaces
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

### 4. Unified Rviz Visualization
**Change**: Merged two separate Rviz windows into one unified view

The unified Rviz (`dual_robots.rviz`) shows:
- Both red and green robots with RobotModel displays  
- All camera feeds (arm cameras, overhead, side)
- Ball pose and status visualization
- TF frames for both robots

**Available Camera Topics**:
```bash
# Arm-mounted RGBD cameras:
/red/fr3_arm_cam/image              # Red RGB
/red/fr3_arm_cam/depth_image        # Red depth
/green/fr3_arm_cam/image            # Green RGB  
/green/fr3_arm_cam/depth_image      # Green depth

# Fixed scene cameras:
/overhead_cam/image                 # Top-down RGB
/overhead_cam/depth_image           # Top-down depth
/side_cam/image                     # Side view RGB
/side_cam/depth_image               # Side depth
```

**Note on Depth Artifacts**: 
Simulated depth cameras may show black/white noise - this is normal. Depth values are correct but visualization can be noisy. Use `depth_image_proc` or Point Cloud displays for better visualization.

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

## Quick Testing Commands

### Complete Test Sequence (simulation.launch.py):
```bash
# 1. Launch simulation (with logging):
cd ~/Projects/ros2_ws
pkill -9 -f "gz|rviz"  # Clean any existing processes
colcon build --symlink-install && source ~/.bashrc && ros2 launch table_tennis_gazebo simulation.launch.py > simulation.log 2>&1 &

# 2. Wait for initialization (~20 seconds), then verify:
sleep 20

# 3. Check action servers are available:
ros2 action list
# Expected: /spawn_ball, /red/arm_controller/follow_joint_trajectory, /green/arm_controller/follow_joint_trajectory

# 4. Check controllers:
ros2 control list_controllers --controller-manager /red/controller_manager
ros2 control list_controllers --controller-manager /green/controller_manager
# Expected: joint_state_broadcaster and arm_controller both "active"

# 5. Test ball spawning:
ros2 action send_goal /spawn_ball table_tennis_gazebo/action/SpawnBall "{x: 0.0, y: 0.6, z: 3.0}" --feedback

# 6. Test robot movement (red):
ros2 action send_goal /red/arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{trajectory: {joint_names: [fr3_joint1, fr3_joint2, fr3_joint3, fr3_joint4, fr3_joint5, fr3_joint6, fr3_joint7], points: [{positions: [0.0, -0.3, 0.0, -1.5, 0.0, 1.2, 0.785], time_from_start: {sec: 2}}]}}"

# 7. Test robot movement (green):
ros2 action send_goal /green/arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{trajectory: {joint_names: [fr3_joint1, fr3_joint2, fr3_joint3, fr3_joint4, fr3_joint5, fr3_joint6, fr3_joint7], points: [{positions: [0.0, 0.3, 0.0, -1.2, 0.0, 1.5, -0.785], time_from_start: {sec: 2}}]}}"

# 8. Monitor ball during play:
ros2 topic echo /ball/status        # Check if ball is alive
ros2 topic echo /ball/pose          # Monitor position

# 9. Check camera feeds:
ros2 topic hz /overhead_cam/image
ros2 topic hz /side_cam/image

# 10. View logs if issues:
cat ~/Projects/ros2_ws/simulation.log | grep -i error
```

### For test.launch.py (Single Robot):
```bash
# Launch
cd ~/Projects/ros2_ws
colcon build --symlink-install && source ~/.bashrc && ros2 launch table_tennis_gazebo test.launch.py > test.log 2>&1 &

# Verify (after 15 seconds):
sleep 15
ros2 control list_controllers
ros2 action send_goal /arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{trajectory: {joint_names: [fr3_joint1, fr3_joint2, fr3_joint3, fr3_joint4, fr3_joint5, fr3_joint6, fr3_joint7], points: [{positions: [0.0, -0.3, 0.0, -1.5, 0.0, 1.2, 0.785], time_from_start: {sec: 2}}]}}"
```

### Cleanup:
```bash
# Kill all simulation processes:
pkill -9 -f "gz|rviz|simulation|test.launch"

# View logs:
cat simulation.log | tail -100
cat test.log | tail -100
```
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
