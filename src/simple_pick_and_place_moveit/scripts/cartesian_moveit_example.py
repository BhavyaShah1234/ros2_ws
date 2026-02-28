#!/usr/bin/env python3

"""
Example script demonstrating Cartesian pose control using MoveIt.
This shows how to move the end-effector to specific Cartesian positions.
"""

import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionIK
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
import sys


class CartesianMoveItExample(Node):
    def __init__(self):
        super().__init__('cartesian_moveit_example')
        
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
        self.move_group.set_max_velocity_scaling_factor(0.3)
        self.move_group.set_max_acceleration_scaling_factor(0.3)
        self.move_group.set_planning_time(5.0)
        
        self.get_logger().info('Cartesian MoveIt example initialized')
        self.get_logger().info(f'Planning frame: {self.move_group.get_planning_frame()}')
        self.get_logger().info(f'End effector link: {self.move_group.get_end_effector_link()}')
        
    def move_to_cartesian_pose(self, x, y, z, roll=0.0, pitch=0.0, yaw=0.0):
        """
        Move end-effector to specified Cartesian pose.
        
        Args:
            x, y, z: Position in meters
            roll, pitch, yaw: Orientation in radians
        """
        
        # Create pose target
        pose_target = PoseStamped()
        pose_target.header.frame_id = self.move_group.get_planning_frame()
        pose_target.pose.position.x = x
        pose_target.pose.position.y = y
        pose_target.pose.position.z = z
        
        # Convert RPY to quaternion
        from scipy.spatial.transform import Rotation
        quat = Rotation.from_euler('xyz', [roll, pitch, yaw]).as_quat()
        pose_target.pose.orientation.x = quat[0]
        pose_target.pose.orientation.y = quat[1]
        pose_target.pose.orientation.z = quat[2]
        pose_target.pose.orientation.w = quat[3]
        
        self.get_logger().info(f'Moving to: x={x:.3f}, y={y:.3f}, z={z:.3f}')
        
        # Set pose target
        self.move_group.set_pose_target(pose_target.pose)
        
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
        
        # Clear targets
        self.move_group.clear_pose_targets()
        
        return success
    
    def move_to_named_target(self, target_name):
        """Move to a named target (e.g., 'home', 'ready')"""
        self.get_logger().info(f'Moving to named target: {target_name}')
        self.move_group.set_named_target(target_name)
        self.move_group.go(wait=True)
        self.move_group.stop()


def main(args=None):
    rclpy.init(args=args)
    
    node = CartesianMoveItExample()
    
    try:
        # Example 1: Move to home position
        node.get_logger().info('Example 1: Moving to home position')
        node.move_to_named_target('home')
        
        # Example 2: Move to specific Cartesian pose
        node.get_logger().info('Example 2: Moving to Cartesian pose')
        node.move_to_cartesian_pose(x=0.4, y=0.2, z=0.5, roll=0.0, pitch=0.0, yaw=0.0)
        
        # Example 3: Move to another pose
        node.get_logger().info('Example 3: Moving to another pose')
        node.move_to_cartesian_pose(x=0.5, y=0.0, z=0.6, roll=0.0, pitch=0.0, yaw=0.785)
        
        # Return to home
        node.get_logger().info('Returning to home')
        node.move_to_named_target('home')
        
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
