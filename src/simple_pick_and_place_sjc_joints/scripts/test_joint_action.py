#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from simple_pick_and_place_sjc_joints.action import MoveToJoints


class TestJointAction(Node):
    def __init__(self):
        super().__init__('test_joint_action')
        
        self.action_client = ActionClient(
            self,
            MoveToJoints,
            '/move_to_joints'
        )
        
    def send_goal(self, joint_positions):
        """Send a joint goal to the action server"""
        
        goal_msg = MoveToJoints.Goal()
        goal_msg.target_joints = joint_positions
        
        self.get_logger().info(f'Sending joint goal: {joint_positions}')
        
        self.action_client.wait_for_server()
        
        send_goal_future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        
        send_goal_future.add_done_callback(self.goal_response_callback)
        
    def goal_response_callback(self, future):
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return
        
        self.get_logger().info('Goal accepted')
        
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)
        
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'Feedback: {feedback.status}, progress: {feedback.progress:.2f}'
        )
        
    def get_result_callback(self, future):
        result = future.result().result
        
        if result.success:
            self.get_logger().info(f'Motion completed: {result.message}')
        else:
            self.get_logger().error(f'Motion failed: {result.message}')


def main(args=None):
    rclpy.init(args=args)
    
    node = TestJointAction()
    
    # Example: Move to a safe home position
    home_position = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
    node.send_goal(home_position)
    
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
