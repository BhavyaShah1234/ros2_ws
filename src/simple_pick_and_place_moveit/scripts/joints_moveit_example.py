#!/usr/bin/env python3

"""
Example script demonstrating joint space control using MoveIt.
This shows how to move to specific joint configurations.
"""

import rclpy
from rclpy.node import Node
import sys


class JointsMoveItExample(Node):
    def __init__(self):
        super().__init__('joints_moveit_example')
        
        # Import moveit_commander here to avoid installation issues during build
        try:
            import moveit_commander
            self.moveit_commander = moveit_commander
        except ImportError:
            self.get_logger().error('moveit_commander not available. Install with: pip install moveit')
            sys.exit(1)
        
        # Initialize moveit_commander
        self.moveit_commander.roscpp_initialize(sys.argv)
        
        # Create robot and move group interfaces
        self.robot = self.moveit_commander.RobotCommander()
        self.scene = self.moveit_commander.PlanningSceneInterface()
        self.move_group = self.moveit_commander.MoveGroupCommander('fr3_arm')
        
        # Set planning parameters
        self.move_group.set_max_velocity_scaling_factor(0.3)
        self.move_group.set_max_acceleration_scaling_factor(0.3)
        self.move_group.set_planning_time(5.0)
        
        self.get_logger().info('Joint space MoveIt example initialized')
        self.get_logger().info(f'Joint names: {self.move_group.get_active_joints()}')
        
    def move_to_joint_values(self, joint_values):
        """
        Move to specified joint configuration.
        
        Args:
            joint_values: List of 7 joint angles in radians
        """
        
        if len(joint_values) != 7:
            self.get_logger().error(f'Expected 7 joint values, got {len(joint_values)}')
            return False
        
        self.get_logger().info(f'Moving to joint values: {joint_values}')
        
        # Set joint value target
        self.move_group.set_joint_value_target(joint_values)
        
        # Plan and execute
        success, plan, planning_time, error_code = self.move_group.plan()
        
        if success:
            self.get_logger().info(f'Planning successful (time: {planning_time:.2f}s)')
            self.get_logger().info('Executing motion...')
            result = self.move_group.execute(plan, wait=True)
            
            if result:
                self.get_logger().info('Motion completed successfully')
            else:
                self.get_logger().error('Motion execution failed')
        else:
            self.get_logger().error(f'Planning failed with error code: {error_code}')
        
        # Stop to ensure no residual movement
        self.move_group.stop()
        
        return success
    
    def get_current_joint_values(self):
        """Get current joint values"""
        current = self.move_group.get_current_joint_values()
        self.get_logger().info(f'Current joint values: {current}')
        return current


def main(args=None):
    rclpy.init(args=args)
    
    node = JointsMoveItExample()
    
    try:
        # Example 1: Move to home position (named target)
        node.get_logger().info('Example 1: Moving to home position')
        home_joints = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
        node.move_to_joint_values(home_joints)
        
        # Example 2: Move to a reachable configuration
        node.get_logger().info('Example 2: Moving to configuration 1')
        config1 = [0.5, -0.5, 0.0, -2.0, 0.0, 1.5, 1.0]
        node.move_to_joint_values(config1)
        
        # Example 3: Move to another configuration
        node.get_logger().info('Example 3: Moving to configuration 2')
        config2 = [-0.5, -0.8, 0.3, -2.5, 0.2, 1.8, 0.5]
        node.move_to_joint_values(config2)
        
        # Example 4: Check current position
        node.get_logger().info('Example 4: Checking current position')
        node.get_current_joint_values()
        
        # Return to home
        node.get_logger().info('Returning to home')
        node.move_to_joint_values(home_joints)
        
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
