#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from simple_pick_and_place_sjc_joints.action import MoveToJoints
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration

import time


class JointActionServer(Node):
    """
    Action server that accepts joint space goals and sends them to the arm controller.
    """
    
    def __init__(self):
        super().__init__('joint_action_server')
        
        # Callback group for parallel execution
        self.callback_group = ReentrantCallbackGroup()
        
        # Joint names (FR3 arm)
        self.joint_names = [
            'fr3_joint1', 'fr3_joint2', 'fr3_joint3', 'fr3_joint4',
            'fr3_joint5', 'fr3_joint6', 'fr3_joint7'
        ]
        
        # Current joint state
        self.current_joint_state = None
        
        # Subscribe to joint states
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10,
            callback_group=self.callback_group
        )
        
        # Action client to arm controller
        from rclpy.action import ActionClient
        self.arm_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory',
            callback_group=self.callback_group
        )
        
        # Action server for joint commands
        self.action_server = ActionServer(
            self,
            MoveToJoints,
            '/move_to_joints',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.callback_group
        )
        
        self.get_logger().info('Joint action server started')
        
    def joint_state_callback(self, msg):
        """Store current joint state"""
        self.current_joint_state = msg
        
    def goal_callback(self, goal_request):
        """Accept or reject goal requests"""
        self.get_logger().info('Received joint goal request')
        return GoalResponse.ACCEPT
        
    def cancel_callback(self, goal_handle):
        """Handle cancellation requests"""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT
        
    def execute_callback(self, goal_handle: ServerGoalHandle):
        """Execute the joint motion goal"""
        self.get_logger().info('Executing joint goal...')
        
        # Get goal
        goal = goal_handle.request
        feedback = MoveToJoints.Feedback()
        result = MoveToJoints.Result()
        
        # Validate joint positions
        if len(goal.target_joints) != 7:
            result.success = False
            result.message = f'Expected 7 joint values, got {len(goal.target_joints)}'
            goal_handle.abort()
            return result
        
        # Wait for current joint state
        timeout = 5.0
        start_time = time.time()
        while self.current_joint_state is None:
            if time.time() - start_time > timeout:
                result.success = False
                result.message = 'Timeout waiting for joint states'
                goal_handle.abort()
                return result
            time.sleep(0.1)
        
        # Create trajectory
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        
        # Start point (current position)
        point_start = JointTrajectoryPoint()
        point_start.positions = self.get_current_joint_positions()
        point_start.time_from_start = Duration(sec=0, nanosec=0)
        trajectory.points.append(point_start)
        
        # End point (target position)
        point_end = JointTrajectoryPoint()
        point_end.positions = list(goal.target_joints)
        point_end.time_from_start = Duration(sec=3, nanosec=0)
        trajectory.points.append(point_end)
        
        # Send trajectory goal
        arm_goal = FollowJointTrajectory.Goal()
        arm_goal.trajectory = trajectory
        
        # Wait for arm controller client
        if not self.arm_client.wait_for_server(timeout_sec=5.0):
            result.success = False
            result.message = 'Arm controller not available'
            goal_handle.abort()
            return result
        
        # Send goal to arm controller
        feedback.status = 'Sending trajectory to arm controller'
        feedback.progress = 0.5
        goal_handle.publish_feedback(feedback)
        
        arm_future = self.arm_client.send_goal_async(arm_goal)
        rclpy.spin_until_future_complete(self, arm_future)
        
        arm_goal_handle = arm_future.result()
        if not arm_goal_handle.accepted:
            result.success = False
            result.message = 'Trajectory rejected by arm controller'
            goal_handle.abort()
            return result
        
        # Wait for trajectory execution
        arm_result_future = arm_goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, arm_result_future)
        
        # Check result
        arm_result = arm_result_future.result().result
        if arm_result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            result.success = True
            result.message = 'Motion completed successfully'
            result.final_joints = list(goal.target_joints)
            feedback.progress = 1.0
            goal_handle.publish_feedback(feedback)
            goal_handle.succeed()
        else:
            result.success = False
            result.message = f'Arm controller error: {arm_result.error_string}'
            goal_handle.abort()
        
        return result
    
    def get_current_joint_positions(self):
        """Extract current joint positions in correct order"""
        if self.current_joint_state is None:
            return [0.0] * 7
        
        positions = []
        for joint_name in self.joint_names:
            try:
                idx = self.current_joint_state.name.index(joint_name)
                positions.append(self.current_joint_state.position[idx])
            except (ValueError, IndexError):
                positions.append(0.0)
        
        return positions


def main(args=None):
    rclpy.init(args=args)
    
    node = JointActionServer()
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
