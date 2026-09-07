#!/usr/bin/env python3
# Copyright (c) 2026 Bhavya Shah
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Verifies that the arm's ros2_control setup is actually commandable:
#   1. joint_trajectory_controller is loaded and active, claiming the
#      position command interface for all 7 fr3 joints.
#   2. A small FollowJointTrajectory goal is accepted and reported
#      successful.
#   3. The robot's reported joint positions (/joint_states) actually moved
#      to the commanded target, within tolerance.
#
# Run with: ros2 run franka_overhead_camera verify_controller.py
# (the simulation must already be running.)

import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import ListControllers
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

JOINT_NAMES = [f'fr3_joint{i}' for i in range(1, 8)]
CONTROLLER_NAME = 'joint_trajectory_controller'
POSITION_TOLERANCE = 0.01  # rad
TEST_DELTA = {'fr3_joint1': 0.05, 'fr3_joint2': -0.05}
GOAL_DURATION_SEC = 2.0


class ControllerVerifier(Node):

    def __init__(self):
        super().__init__('verify_controller')
        self.list_controllers_client = self.create_client(
            ListControllers, '/controller_manager/list_controllers')
        self.action_client = ActionClient(
            self, FollowJointTrajectory,
            f'/{CONTROLLER_NAME}/follow_joint_trajectory')
        self._latest_joint_state = None
        self.create_subscription(
            JointState, '/joint_states', self._joint_state_cb, 10)

    def _joint_state_cb(self, msg):
        self._latest_joint_state = msg

    def wait_for_joint_state(self, timeout_sec=10.0):
        deadline = time.time() + timeout_sec
        while self._latest_joint_state is None and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self._latest_joint_state

    def get_positions(self, joint_state):
        return {
            name: joint_state.position[joint_state.name.index(name)]
            for name in JOINT_NAMES
        }


def check_controller_active(node):
    print(f'[1/3] Checking "{CONTROLLER_NAME}" is loaded and active...')
    if not node.list_controllers_client.wait_for_service(timeout_sec=10.0):
        print('  FAIL: /controller_manager/list_controllers service not available')
        return False

    future = node.list_controllers_client.call_async(ListControllers.Request())
    rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
    if future.result() is None:
        print('  FAIL: list_controllers call did not return')
        return False

    controllers = {c.name: c for c in future.result().controller}
    if CONTROLLER_NAME not in controllers:
        print(f'  FAIL: "{CONTROLLER_NAME}" is not loaded at all')
        return False

    ctrl = controllers[CONTROLLER_NAME]
    if ctrl.state != 'active':
        print(f'  FAIL: "{CONTROLLER_NAME}" state is "{ctrl.state}", not "active"')
        return False

    claimed = set(ctrl.claimed_interfaces)  # string[], e.g. "fr3_joint1/position"
    missing = [j for j in JOINT_NAMES if f'{j}/position' not in claimed]
    if missing:
        print(f'  FAIL: controller is active but not claiming position '
              f'interfaces for: {missing} (claimed: {sorted(claimed)})')
        return False

    print(f'  PASS: "{CONTROLLER_NAME}" is active and claims all 7 joints\' '
          f'position interfaces')
    return True


def send_test_trajectory(node, start_positions):
    print('[2/3] Sending a small test trajectory...')
    target = dict(start_positions)
    for joint, delta in TEST_DELTA.items():
        target[joint] += delta

    if not node.action_client.wait_for_server(timeout_sec=10.0):
        print(f'  FAIL: action server /{CONTROLLER_NAME}/follow_joint_trajectory '
              f'not available')
        return None

    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = JOINT_NAMES
    point = JointTrajectoryPoint()
    point.positions = [target[j] for j in JOINT_NAMES]
    point.time_from_start.sec = int(GOAL_DURATION_SEC)
    point.time_from_start.nanosec = int((GOAL_DURATION_SEC % 1) * 1e9)
    goal.trajectory.points = [point]

    send_future = node.action_client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, send_future, timeout_sec=10.0)
    goal_handle = send_future.result()
    if goal_handle is None or not goal_handle.accepted:
        print('  FAIL: goal was rejected or send_goal timed out')
        return None

    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(
        node, result_future, timeout_sec=GOAL_DURATION_SEC + 10.0)
    result = result_future.result()
    if result is None:
        print('  FAIL: did not receive a result before timeout')
        return None
    if result.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
        print(f'  FAIL: trajectory finished with error_code={result.result.error_code}')
        return None

    print('  PASS: trajectory goal accepted and reported successful')
    return target


def check_final_positions(node, target):
    print('[3/3] Confirming the arm actually moved to the commanded target...')
    joint_state = node.wait_for_joint_state()
    if joint_state is None:
        print('  FAIL: no /joint_states message received')
        return False

    final = node.get_positions(joint_state)
    all_ok = True
    for joint in JOINT_NAMES:
        err = abs(final[joint] - target[joint])
        status = 'ok' if err <= POSITION_TOLERANCE else 'OUT OF TOLERANCE'
        if joint in TEST_DELTA:
            print(f'  {joint}: target={target[joint]:.4f} actual={final[joint]:.4f} '
                  f'err={err:.4f} [{status}]')
        if err > POSITION_TOLERANCE:
            all_ok = False

    if all_ok:
        print('  PASS: all joints within tolerance of the commanded target')
    else:
        print('  FAIL: one or more joints did not reach the commanded target')
    return all_ok


def main():
    rclpy.init()
    node = ControllerVerifier()

    ok_active = check_controller_active(node)
    if not ok_active:
        print('\nRESULT: FAIL (controller not active/commandable)')
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    start_state = node.wait_for_joint_state()
    if start_state is None:
        print('\nRESULT: FAIL (no /joint_states received)')
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)
    start_positions = node.get_positions(start_state)

    target = send_test_trajectory(node, start_positions)
    if target is None:
        print('\nRESULT: FAIL (trajectory command did not succeed)')
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    ok_final = check_final_positions(node, target)

    print(f'\nRESULT: {"PASS" if ok_final else "FAIL"}')
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok_final else 1)


if __name__ == '__main__':
    main()
