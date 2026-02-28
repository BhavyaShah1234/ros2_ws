#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from simple_pick_and_place_sjc_cartesian.action import MoveToCartesian
from geometry_msgs.msg import Pose


class TestCartesianAction(Node):
    def __init__(self):
        super().__init__('test_cartesian_action')
        
        self.action_client = ActionClient(
            self,
            MoveToCartesian,
            '/move_to_cartesian'
        )
        
    def send_goal(self, x, y, z):
        """Send a Cartesian goal to the action server"""
        
        goal_msg = MoveToCartesian.Goal()
        goal_msg.target_pose = Pose()
        goal_msg.target_pose.position.x = x
        goal_msg.target_pose.position.y = y
        goal_msg.target_pose.position.z = z
        goal_msg.target_pose.orientation.w = 1.0  # No rotation
        
        self.get_logger().info(f'Sending goal: ({x}, {y}, {z})')
        
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
    
    node = TestCartesianAction()
    
    # Example: Move to position (0.4, 0.2, 0.5)
    node.send_goal(0.4, 0.2, 0.5)
    
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
