#!/usr/bin/env python3

"""
Example script demonstrating high-level pick and place using MoveIt.
This shows how to use MoveIt's built-in pick() and place() operations.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import sys


class HighLevelMoveItExample(Node):
    def __init__(self):
        super().__init__('highlevel_moveit_example')
        
        # Import moveit_commander here to avoid installation issues during build
        try:
            import moveit_commander
            self.moveit_commander = moveit_commander
        except ImportError:
            self.get_logger().error('moveit_commander not available. Install with: pip install moveit')
            sys.exit(1)
        
        # Initialize moveit_commander
        self.moveit_commander.roscpp_initialize(sys.argv)
        
        # Create robot, scene, and move group interfaces
        self.robot = self.moveit_commander.RobotCommander()
        self.scene = self.moveit_commander.PlanningSceneInterface()
        self.move_group = self.moveit_commander.MoveGroupCommander('fr3_arm')
        
        # Set planning parameters
        self.move_group.set_max_velocity_scaling_factor(0.2)
        self.move_group.set_max_acceleration_scaling_factor(0.2)
        self.move_group.set_planning_time(5.0)
        
        self.get_logger().info('High-level pick-and-place MoveIt example initialized')
        
    def add_collision_objects(self):
        """Add collision objects to the planning scene"""
        
        # Add table
        table_pose = PoseStamped()
        table_pose.header.frame_id = 'world'
        table_pose.pose.position.x = 0.5
        table_pose.pose.position.y = 0.0
        table_pose.pose.position.z = 0.4
        table_pose.pose.orientation.w = 1.0
        
        self.scene.add_box('table', table_pose, size=(1.2, 1.2, 0.8))
        
        # Add example object to pick
        object_pose = PoseStamped()
        object_pose.header.frame_id = 'world'
        object_pose.header.stamp = self.get_clock().now().to_msg()
        object_pose.pose.position.x = 0.6
        object_pose.pose.position.y = 0.2
        object_pose.pose.position.z = 0.85
        object_pose.pose.orientation.w = 1.0
        
        self.scene.add_cylinder('target_object', object_pose, height=0.1, radius=0.025)
        
        self.get_logger().info('Added collision objects to planning scene')
        
    def pick_object(self, object_name):
        """
        Pick an object using MoveIt's pick operation.
        
        This is a simplified version - full implementation would include:
        - Grasp pose generation
        - Pre-grasp and post-grasp poses
        - Approach and retreat vectors
        """
        
        self.get_logger().info(f'Attempting to pick object: {object_name}')
        
        # In a full implementation, you would:
        # 1. Generate grasp poses
        # 2. Call move_group.pick(object_name, grasps)
        
        # For this example, we'll manually move to the object
        # Get object pose from scene
        # Move to pre-grasp
        # Open gripper
        # Move to grasp
        # Close gripper
        # Lift
        
        self.get_logger().warn('Pick operation - simplified implementation')
        self.get_logger().info('In production, use MoveIt grasp planning or manual waypoints')
        
        # Example: Move near the object
        target_pose = PoseStamped()
        target_pose.header.frame_id = 'world'
        target_pose.pose.position.x = 0.6
        target_pose.pose.position.y = 0.2
        target_pose.pose.position.z = 0.95  # Above object
        target_pose.pose.orientation.w = 1.0
        
        self.move_group.set_pose_target(target_pose.pose)
        success = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        
        return success
    
    def place_object(self, location_name, x, y, z):
        """
        Place an object at a specified location.
        
        This is a simplified version - full implementation would include:
        - Place pose generation
        - Pre-place and post-place poses
        - Approach and retreat vectors
        """
        
        self.get_logger().info(f'Attempting to place object at: {location_name}')
        
        # In a full implementation, you would:
        # 1. Generate place poses
        # 2. Call move_group.place(object_name, places)
        
        self.get_logger().warn('Place operation - simplified implementation')
        self.get_logger().info('In production, use MoveIt place planning or manual waypoints')
        
        # Example: Move to target location
        target_pose = PoseStamped()
        target_pose.header.frame_id = 'world'
        target_pose.pose.position.x = x
        target_pose.pose.position.y = y
        target_pose.pose.position.z = z
        target_pose.pose.orientation.w = 1.0
        
        self.move_group.set_pose_target(target_pose.pose)
        success = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        
        return success
    
    def execute_pick_and_place(self):
        """Execute a complete pick-and-place sequence"""
        
        self.get_logger().info('=== Starting Pick-and-Place Sequence ===')
        
        # Step 1: Add collision objects
        self.add_collision_objects()
        
        # Step 2: Move to home position
        self.get_logger().info('Step 1: Moving to home position')
        self.move_group.set_named_target('home')
        self.move_group.go(wait=True)
        
        # Step 3: Pick object
        self.get_logger().info('Step 2: Picking object')
        success = self.pick_object('target_object')
        
        if not success:
            self.get_logger().error('Pick failed')
            return False
        
        # Step 4: Place object
        self.get_logger().info('Step 3: Placing object')
        success = self.place_object('red_bin', x=0.8, y=0.5, z=0.95)
        
        if not success:
            self.get_logger().error('Place failed')
            return False
        
        # Step 5: Return home
        self.get_logger().info('Step 4: Returning home')
        self.move_group.set_named_target('home')
        self.move_group.go(wait=True)
        
        self.get_logger().info('=== Pick-and-Place Sequence Complete ===')
        return True


def main(args=None):
    rclpy.init(args=args)
    
    node = HighLevelMoveItExample()
    
    try:
        # Execute full pick-and-place sequence
        node.execute_pick_and_place()
        
        # Keep node alive for visualization
        self.get_logger().info('Example complete. Press Ctrl+C to exit.')
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
