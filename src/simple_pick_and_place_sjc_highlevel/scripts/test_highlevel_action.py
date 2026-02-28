#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from simple_pick_and_place_sjc_highlevel.action import PickAndPlace


class TestHighLevelAction(Node):
    def __init__(self):
        super().__init__('test_highlevel_action')
        
        self.action_client = ActionClient(
            self,
            PickAndPlace,
            '/pick_and_place'
        )
        
    def send_goal(self, color, shape, target_bin):
        """Send a pick-and-place goal"""
        
        goal_msg = PickAndPlace.Goal()
        goal_msg.object_color = color
        goal_msg.object_shape = shape
        goal_msg.target_bin = target_bin
        
        self.get_logger().info(
            f'Sending goal: Pick {color} {shape}, place in {target_bin} bin'
        )
        
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
            f'Phase: {feedback.phase}, progress: {feedback.progress:.2f}'
        )
        
    def get_result_callback(self, future):
        result = future.result().result
        
        if result.success:
            self.get_logger().info(f'Pick and place completed: {result.message}')
        else:
            self.get_logger().error(f'Pick and place failed: {result.message}')


def main(args=None):
    rclpy.init(args=args)
    
    node = TestHighLevelAction()
    
    # Example: Pick a red cylinder and place in blue bin
    node.send_goal('red', 'cylinder', 'blue')
    
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
