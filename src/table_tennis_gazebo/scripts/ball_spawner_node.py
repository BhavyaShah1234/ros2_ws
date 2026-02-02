#!/usr/bin/env python3
"""
Python implementation of the Ball Spawner Action Server.
This is equivalent to the C++ ball_spawner_node.cpp for Python users.

The node provides an action server that:
1. Spawns table tennis balls at specified 3D positions
2. Monitors ball state (position, velocity)
3. Automatically destroys balls when they hit the ground or become stationary
4. Provides real-time feedback during ball lifetime
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from table_tennis_gazebo.action import SpawnBall
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from ros_gz_interfaces.srv import SpawnEntity, DeleteEntity
from ros_gz_interfaces.msg import Entity

import math
import time
import threading
import subprocess
import tempfile


class BallSpawnerActionServer(Node):
    """
    Action server for spawning and managing table tennis balls in Gazebo.
    
    Python equivalent of the C++ BallSpawnerActionServer.
    """

    def __init__(self):
        super().__init__('ball_spawner')
        
        # Callback group for parallel execution
        self.callback_group = ReentrantCallbackGroup()
        
        # Create action server
        self._action_server = ActionServer(
            self,
            SpawnBall,
            'spawn_ball',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.callback_group
        )
        
        # Create Gazebo service clients
        self.spawn_client = self.create_client(
            SpawnEntity, 
            '/world/arena/create',
            callback_group=self.callback_group
        )
        self.delete_client = self.create_client(
            DeleteEntity,
            '/world/arena/remove',
            callback_group=self.callback_group
        )
        
        # Publishers
        self.ball_status_pub = self.create_publisher(
            String, 
            '/ball/status', 
            10
        )
        self.ball_pose_pub = self.create_publisher(
            Pose,
            '/ball/pose',
            10
        )
        
        # State tracking
        self.ball_alive = False
        self.current_ball_name = 'table_tennis_ball'
        self.current_pose = Pose()
        self.previous_pose = Pose()
        self.current_velocity = 0.0
        self.spawn_time = None
        self.last_pose_time = None
        self.pose_received = False
        self.ball_has_moved = False  # Track if ball has moved from spawn position
        
        # Gazebo transport subscription (using gz topic command)
        self.pose_lock = threading.Lock()
        
        self.get_logger().info('Ball spawner action server (Python) initialized')
        self.get_logger().info('Action server available at: spawn_ball')
    
    def goal_callback(self, goal_request):
        """Accept or reject incoming goal requests."""
        if self.ball_alive:
            self.get_logger().warn('Ball is still alive, rejecting new spawn request')
            return GoalResponse.REJECT
        
        self.get_logger().info(
            f'Received spawn ball goal at [{goal_request.x:.3f}, '
            f'{goal_request.y:.3f}, {goal_request.z:.3f}]'
        )
        return GoalResponse.ACCEPT
    
    def cancel_callback(self, goal_handle):
        """Handle cancellation requests."""
        self.get_logger().info('Received request to cancel goal')
        return CancelResponse.ACCEPT
    
    async def execute_callback(self, goal_handle):
        """Execute the ball spawning action."""
        self.get_logger().info('Executing spawn ball action')
        
        goal = goal_handle.request
        feedback = SpawnBall.Feedback()
        result = SpawnBall.Result()
        
        # Delete existing ball if any
        if self.ball_exists():
            await self.delete_ball()
            time.sleep(0.5)
        
        # Reset state
        self.current_pose.position.x = goal.x
        self.current_pose.position.y = goal.y
        self.current_pose.position.z = goal.z
        self.pose_received = False
        
        # Spawn new ball
        if not await self.spawn_ball(goal.x, goal.y, goal.z):
            result.final_status = 'failed_to_spawn'
            result.final_x = goal.x
            result.final_y = goal.y
            result.final_z = goal.z
            result.time_alive = 0.0
            goal_handle.abort(result)
            return result
        
        self.ball_alive = True
        self.spawn_time = self.get_clock().now()
        
        # Initialize pose tracking
        self.previous_pose = Pose()
        self.previous_pose.position.x = goal.x
        self.previous_pose.position.y = goal.y
        self.previous_pose.position.z = goal.z
        self.current_velocity = 0.0
        self.last_pose_time = self.spawn_time
        self.ball_has_moved = False
        
        # Start pose monitoring thread
        pose_thread = threading.Thread(
            target=self.monitor_pose_thread,
            daemon=True
        )
        pose_thread.start()
        
        # Wait for initial pose
        wait_start = self.get_clock().now()
        rate = self.create_rate(20)
        while not self.pose_received and \
              (self.get_clock().now() - wait_start).nanoseconds / 1e9 < 1.0:
            rate.sleep()
        
        if self.pose_received:
            self.get_logger().info(
                f'Ball initial pose: [{self.current_pose.position.x:.3f}, '
                f'{self.current_pose.position.y:.3f}, '
                f'{self.current_pose.position.z:.3f}]'
            )
        
        # Monitor ball and send feedback
        rate = self.create_rate(50)  # 50 Hz feedback
        while rclpy.ok() and self.ball_alive:
            # Check if goal was canceled
            if goal_handle.is_cancel_requested:
                result.final_status = 'canceled'
                with self.pose_lock:
                    result.final_x = self.current_pose.position.x
                    result.final_y = self.current_pose.position.y
                    result.final_z = self.current_pose.position.z
                result.time_alive = (self.get_clock().now() - self.spawn_time).nanoseconds / 1e9
                await self.delete_ball()
                self.ball_alive = False
                goal_handle.canceled(result)
                return result
            
            # Update feedback
            with self.pose_lock:
                feedback.status = 'alive'
                feedback.current_x = self.current_pose.position.x
                feedback.current_y = self.current_pose.position.y
                feedback.current_z = self.current_pose.position.z
            
            goal_handle.publish_feedback(feedback)
            
            # Publish status
            status_msg = String()
            status_msg.data = 'alive'
            self.ball_status_pub.publish(status_msg)
            self.ball_pose_pub.publish(self.current_pose)
            
            # Check deletion conditions
            time_alive = (self.get_clock().now() - self.spawn_time).nanoseconds / 1e9
            
            with self.pose_lock:
                z = self.current_pose.position.z
                vel = self.current_velocity
                
                # Check if ball has moved significantly from spawn position
                if not self.ball_has_moved and vel > 0.1:
                    self.ball_has_moved = True
                    self.get_logger().info(f'Ball started moving with velocity {vel:.3f} m/s')
                
                # Check if pose updates have stopped (ball stationary in Gazebo)
                time_since_update = (self.get_clock().now() - self.last_pose_time).nanoseconds / 1e9 if self.last_pose_time else 0
            
            # Ball radius is 0.02m, ground is at z=0
            # Table top is at z=0.76m
            hit_ground = z < 0.05
            below_table = z < 0.65
            # Only check stationary after ball has moved (to avoid false positives at spawn)
            is_stationary = self.ball_has_moved and (time_alive > 2.0) and (vel < 0.01)
            # Also delete if no pose updates for 3+ seconds after ball has moved (Gazebo stops publishing stationary objects)
            pose_timeout = self.ball_has_moved and (time_alive > 3.0) and (time_since_update > 3.0)
            
            if hit_ground or below_table or is_stationary or pose_timeout:
                if hit_ground or below_table:
                    self.get_logger().info(f'Ball hit the ground! z={z:.4f}')
                elif pose_timeout:
                    self.get_logger().info(f'Ball detected as stationary (no pose updates for {time_since_update:.1f}s)')
                else:
                    self.get_logger().info(f'Ball became stationary! velocity={vel:.4f} m/s')
                
                self.ball_alive = False
                
                # Publish dead status
                status_msg.data = 'dead'
                self.ball_status_pub.publish(status_msg)
                
                # Set result
                result.final_status = 'hit_ground' if (hit_ground or below_table) else 'stationary'
                with self.pose_lock:
                    result.final_x = self.current_pose.position.x
                    result.final_y = self.current_pose.position.y
                    result.final_z = self.current_pose.position.z
                result.time_alive = time_alive
                
                # Delete ball
                await self.delete_ball()
                
                goal_handle.succeed(result)
                self.get_logger().info(f'Ball action completed, time alive: {result.time_alive:.2f} seconds')
                return result
            
            rate.sleep()
        
        return result
    
    def monitor_pose_thread(self):
        """
        Monitor ball pose using gz topic echo command.
        Runs in a separate thread to avoid blocking the action execution.
        
        Note: This implementation uses subprocess to monitor Gazebo topics.
        The pose updates may be delayed or not work reliably in all cases.
        A production implementation should use gz-transport Python bindings.
        """
        try:
            # Use gz topic to subscribe to dynamic pose updates
            cmd = ['gz', 'topic', '-e', '-t', '/world/arena/dynamic_pose/info', '-n', '100']
            process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                bufsize=1,
                universal_newlines=True
            )
            
            current_ball_data = {}
            in_ball_entity = False
            in_position_block = False
            last_update_time = None
            
            for line in process.stdout:
                if not self.ball_alive:
                    process.kill()
                    break
                
                line = line.strip()
                
                # Look for our ball entity
                if 'name: "table_tennis_ball"' in line or \
                   f'name: "{self.current_ball_name}"' in line:
                    in_ball_entity = True
                    current_ball_data = {'x': 0.0, 'y': 0.0, 'z': 0.0}
                    in_position_block = False
                    continue
                
                if in_ball_entity:
                    # Parse position block
                    if 'position {' in line:
                        in_position_block = True
                        continue
                    elif line == '}':
                        # End of a block
                        if in_position_block:
                            # We have complete position data, update pose and calculate velocity
                            in_ball_entity = False
                            in_position_block = False
                            
                            current_time = time.time()
                            
                            with self.pose_lock:
                                # Store previous pose for velocity calculation
                                prev_x = self.current_pose.position.x
                                prev_y = self.current_pose.position.y
                                prev_z = self.current_pose.position.z
                                
                                # Update current pose
                                self.current_pose.position.x = current_ball_data['x']
                                self.current_pose.position.y = current_ball_data['y']
                                self.current_pose.position.z = current_ball_data['z']
                                
                                # Calculate velocity magnitude from position change
                                if last_update_time is not None and self.pose_received:
                                    dt = current_time - last_update_time
                                    if dt > 0.001:  # Avoid division by zero
                                        dx = current_ball_data['x'] - prev_x
                                        dy = current_ball_data['y'] - prev_y
                                        dz = current_ball_data['z'] - prev_z
                                        distance = math.sqrt(dx*dx + dy*dy + dz*dz)
                                        self.current_velocity = distance / dt
                                
                                self.last_pose_time = self.get_clock().now()
                                self.pose_received = True
                            
                            last_update_time = current_time
                            current_ball_data = {}
                        continue
                    
                    # Parse field values (only in position block)
                    if in_position_block:
                        if line.startswith('x:'):
                            try:
                                current_ball_data['x'] = float(line.split(':')[1].strip())
                            except:
                                pass
                        elif line.startswith('y:'):
                            try:
                                current_ball_data['y'] = float(line.split(':')[1].strip())
                            except:
                                pass
                        elif line.startswith('z:'):
                            try:
                                current_ball_data['z'] = float(line.split(':')[1].strip())
                            except:
                                pass
                        
        except Exception as e:
            self.get_logger().error(f'Pose monitoring error: {e}')
            self.get_logger().error('Ball pose tracking may not work correctly')
            self.get_logger().info('This is a known issue with subprocess-based pose monitoring')
    
    def ball_exists(self):
        """Check if ball currently exists."""
        return self.ball_alive
    
    async def spawn_ball(self, x, y, z):
        """
        Spawn a table tennis ball at the specified position.
        
        Args:
            x, y, z: Position coordinates in meters
            
        Returns:
            True if spawn successful, False otherwise
        """
        # Create SDF content for ball
        sdf_content = f"""<?xml version='1.0'?>
<sdf version='1.9'>
  <model name='{self.current_ball_name}'>
    <pose>{x} {y} {z} 0 0 0</pose>
    <link name='ball_link'>
      <inertial>
        <mass>0.0027</mass>
        <inertia>
          <ixx>4.32e-7</ixx>
          <ixy>0.0</ixy>
          <ixz>0.0</ixz>
          <iyy>4.32e-7</iyy>
          <iyz>0.0</iyz>
          <izz>4.32e-7</izz>
        </inertia>
      </inertial>
      <collision name='collision'>
        <geometry>
          <sphere><radius>0.020</radius></sphere>
        </geometry>
        <surface>
          <contact>
            <ode>
              <kp>100000.0</kp>
              <kd>10.0</kd>
              <max_vel>100.0</max_vel>
              <min_depth>0.0001</min_depth>
            </ode>
          </contact>
          <friction>
            <ode>
              <mu>0.6</mu>
              <mu2>0.6</mu2>
            </ode>
          </friction>
          <bounce>
            <restitution_coefficient>0.85</restitution_coefficient>
            <threshold>0.01</threshold>
          </bounce>
        </surface>
      </collision>
      <visual name='visual'>
        <geometry>
          <sphere><radius>0.020</radius></sphere>
        </geometry>
        <material>
          <ambient>1.0 0.6 0.0 1.0</ambient>
          <diffuse>1.0 0.7 0.1 1.0</diffuse>
          <specular>0.8 0.8 0.8 1.0</specular>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""
        
        # Write to temporary file
        with tempfile.NamedTemporaryFile(mode='w', suffix='.sdf', delete=False) as f:
            f.write(sdf_content)
            sdf_file = f.name
        
        # Use gz service command to spawn
        self.get_logger().info(f'Spawning ball at [{x:.3f}, {y:.3f}, {z:.3f}]')
        
        cmd = [
            'gz', 'service', '-s', '/world/arena/create',
            '--reqtype', 'gz.msgs.EntityFactory',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', '1000',
            '--req', f'sdf_filename: "{sdf_file}"'
        ]
        
        try:
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=2)
            if result.returncode == 0:
                self.get_logger().info('Ball spawned successfully')
                return True
            else:
                self.get_logger().error(f'Failed to spawn ball: {result.stderr}')
                return False
        except Exception as e:
            self.get_logger().error(f'Exception during spawn: {e}')
            return False
    
    async def delete_ball(self):
        """
        Delete the current ball from Gazebo using gz service command.
        
        Note: Using subprocess instead of ROS 2 service client because the 
        ros_gz service bridging has reliability issues in Jazzy/Harmonic.
        """
        try:
            cmd = [
                'gz', 'service',
                '-s', '/world/arena/remove',
                '--reqtype', 'gz.msgs.Entity',
                '--reptype', 'gz.msgs.Boolean',
                '--timeout', '2000',
                '--req', f'name: "{self.current_ball_name}" type: MODEL'
            ]
            
            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=3.0
            )
            
            if result.returncode == 0:
                self.get_logger().info(f'Ball "{self.current_ball_name}" deleted successfully')
                return True
            else:
                self.get_logger().warn(
                    f'Failed to delete ball: {result.stderr if result.stderr else "Unknown error"}'
                )
                return False
                
        except subprocess.TimeoutExpired:
            self.get_logger().error('Delete command timed out')
            return False
        except Exception as e:
            self.get_logger().error(f'Exception during delete: {e}')
            return False


def main(args=None):
    rclpy.init(args=args)
    
    ball_spawner = BallSpawnerActionServer()
    
    # Use MultiThreadedExecutor for parallel callback execution
    executor = MultiThreadedExecutor()
    executor.add_node(ball_spawner)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        ball_spawner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
