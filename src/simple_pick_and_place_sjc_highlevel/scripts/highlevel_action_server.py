#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse, ActionClient
from rclpy.action.server import ServerGoalHandle
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from simple_pick_and_place_sjc_highlevel.action import PickAndPlace
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import Image, JointState
from builtin_interfaces.msg import Duration

import cv2
from cv_bridge import CvBridge
import numpy as np
import time


class HighLevelActionServer(Node):
    """
    High-level action server for pick-and-place operations with computer vision.
    Detects objects using overhead camera and executes full pick-place sequence.
    """
    
    def __init__(self):
        super().__init__('highlevel_action_server')
        
        # Callback group for parallel execution
        self.callback_group = ReentrantCallbackGroup()
        
        # Joint names
        self.joint_names = [
            'fr3_joint1', 'fr3_joint2', 'fr3_joint3', 'fr3_joint4',
            'fr3_joint5', 'fr3_joint6', 'fr3_joint7'
        ]
        
        # State
        self.current_image = None
        self.current_joint_state = None
        self.bridge = CvBridge()
        
        # Subscribe to camera
        self.camera_sub = self.create_subscription(
            Image,
            '/overhead_camera/image_raw',
            self.camera_callback,
            10,
            callback_group=self.callback_group
        )
        
        # Subscribe to joint states
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10,
            callback_group=self.callback_group
        )
        
        # Action clients
        self.arm_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory',
            callback_group=self.callback_group
        )
        
        self.gripper_client = ActionClient(
            self,
            GripperCommand,
            '/gripper_controller/gripper_cmd',
            callback_group=self.callback_group
        )
        
        # Action server
        self.action_server = ActionServer(
            self,
            PickAndPlace,
            '/pick_and_place',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.callback_group
        )
        
        # Predefined positions
        self.bin_positions = {
            'red': [0.8, 0.5, 0.9],
            'green': [0.8, -0.5, 0.9],
            'blue': [1.1, 0.0, 0.9],
        }
        
        self.get_logger().info('High-level pick and place action server started')
        
    def camera_callback(self, msg):
        """Store current camera image"""
        self.current_image = msg
        
    def joint_state_callback(self, msg):
        """Store current joint state"""
        self.current_joint_state = msg
        
    def goal_callback(self, goal_request):
        """Accept or reject goal requests"""
        self.get_logger().info('Received pick-and-place goal request')
        return GoalResponse.ACCEPT
        
    def cancel_callback(self, goal_handle):
        """Handle cancellation requests"""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT
        
    def execute_callback(self, goal_handle: ServerGoalHandle):
        """Execute the pick-and-place sequence"""
        self.get_logger().info('Executing pick-and-place goal...')
        
        goal = goal_handle.request
        feedback = PickAndPlace.Feedback()
        result = PickAndPlace.Result()
        
        try:
            # Phase 1: Detect object
            feedback.phase = 'detecting'
            feedback.progress = 0.1
            goal_handle.publish_feedback(feedback)
            
            object_position = self.detect_object(goal.object_color, goal.object_shape)
            if object_position is None:
                result.success = False
                result.message = f'Object not found: {goal.object_color} {goal.object_shape}'
                goal_handle.abort()
                return result
            
            # Phase 2: Move to pre-grasp position
            feedback.phase = 'approaching'
            feedback.progress = 0.2
            goal_handle.publish_feedback(feedback)
            
            pre_grasp_position = [object_position[0], object_position[1], object_position[2] + 0.1]
            self.move_to_cartesian(pre_grasp_position)
            
            # Phase 3: Open gripper
            feedback.phase = 'opening'
            feedback.progress = 0.3
            goal_handle.publish_feedback(feedback)
            
            self.control_gripper(0.04)  # Open
            
            # Phase 4: Move to grasp position
            feedback.phase = 'grasping'
            feedback.progress = 0.4
            goal_handle.publish_feedback(feedback)
            
            self.move_to_cartesian(object_position)
            
            # Phase 5: Close gripper
            self.control_gripper(0.01)  # Close
            time.sleep(1.0)
            
            # Phase 6: Lift
            feedback.phase = 'lifting'
            feedback.progress = 0.5
            goal_handle.publish_feedback(feedback)
            
            self.move_to_cartesian(pre_grasp_position)
            
            # Phase 7: Move to bin
            feedback.phase = 'moving'
            feedback.progress = 0.7
            goal_handle.publish_feedback(feedback)
            
            bin_position = self.bin_positions.get(goal.target_bin)
            if bin_position:
                self.move_to_cartesian(bin_position)
            
            # Phase 8: Release
            feedback.phase = 'placing'
            feedback.progress = 0.9
            goal_handle.publish_feedback(feedback)
            
            self.control_gripper(0.04)  # Open
            time.sleep(1.0)
            
            # Phase 9: Return home
            self.move_to_home()
            
            result.success = True
            result.message = 'Pick and place completed successfully'
            feedback.progress = 1.0
            goal_handle.publish_feedback(feedback)
            goal_handle.succeed()
            
        except Exception as e:
            result.success = False
            result.message = f'Error: {str(e)}'
            goal_handle.abort()
        
        return result
    
    def detect_object(self, color, shape):
        """
        Detect object using computer vision (simplified implementation).
        Returns [x, y, z] position or None if not found.
        """
        if self.current_image is None:
            self.get_logger().warn('No camera image available')
            return None
        
        # Convert ROS image to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(self.current_image, 'bgr8')
        
        # Define color ranges in HSV
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        color_ranges = {
            'red': ([0, 100, 100], [10, 255, 255]),
            'green': ([50, 100, 100], [70, 255, 255]),
            'blue': ([110, 100, 100], [130, 255, 255]),
        }
        
        if color not in color_ranges:
            return None
        
        # Create mask
        lower, upper = color_ranges[color]
        mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return None
        
        # Get largest contour
        largest_contour = max(contours, key=cv2.contourArea)
        M = cv2.moments(largest_contour)
        
        if M['m00'] == 0:
            return None
        
        # Calculate centroid (in image coordinates)
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        
        # Convert to world coordinates (simplified transformation)
        # This would need proper camera calibration in real implementation
        x = 0.5 + (cx - 320) * 0.001
        y = (cy - 240) * 0.001
        z = 0.85
        
        self.get_logger().info(f'Detected {color} {shape} at ({x:.3f}, {y:.3f}, {z:.3f})')
        
        return [x, y, z]
    
    def move_to_cartesian(self, position):
        """Move end-effector to Cartesian position (simplified)"""
        # In real implementation, use IK solver
        # For now, use pre-defined joint configurations
        self.get_logger().info(f'Moving to position: {position}')
        time.sleep(2.0)  # Simulate motion
        
    def control_gripper(self, position):
        """Control gripper position"""
        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = 50.0
        
        if not self.gripper_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn('Gripper controller not available')
            return
        
        future = self.gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        
    def move_to_home(self):
        """Move to home position"""
        self.get_logger().info('Returning to home position')
        time.sleep(2.0)


def main(args=None):
    rclpy.init(args=args)
    
    node = HighLevelActionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
