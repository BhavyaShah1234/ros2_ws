import rclpy as r
import tf2_ros
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import CameraInfo, JointState
from geometry_msgs.msg import Pose
from nav_msgs.msg import Path
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.action import ExecuteTrajectory

GROUP_NAME = 'fr3_arm'
LINK_NAME = 'laser_link'
TRACE_HEIGHT_M = 0.015
MAX_STEP_M = 0.005
# fr3_joint_limits.yaml/the URDF's own <limit velocity="..."> give MoveIt2 the
# real joint limits -- this just leaves headroom for a spline's peak velocity
# exceeding its point-to-point average, same reasoning the old ikpy-based
# VELOCITY_SAFETY_FACTOR used.
VELOCITY_SAFETY_FACTOR = 0.5

class MotionExecutor(Node):
    def __init__(self):
        super(MotionExecutor, self).__init__(node_name='motion_executor')
        self.camera_info = None
        self.joint_state = None
        self.executing = False
        self.executed_path_key = None
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        latched_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(CameraInfo, '/overhead_camera/camera_info', self.camera_info_callback, 10)
        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.create_subscription(Path, '/path', self.path_callback, latched_qos)
        self.cartesian_path_client = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        self.execute_trajectory_client = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')
        self.get_logger().info('motion_executor started')

    def camera_info_callback(self, camera_info_message):
        self.camera_info = camera_info_message

    def joint_state_callback(self, joint_state_message):
        self.joint_state = joint_state_message

    def pixel_to_world(self, u, v, z):
        camera_transform = self.tf_buffer.lookup_transform('world', 'overhead_camera/link/overhead_rgbd_camera', Time())
        x_cam = camera_transform.transform.translation.x
        y_cam = camera_transform.transform.translation.y
        height = camera_transform.transform.translation.z
        focal_length = self.camera_info.k[0]
        cx, cy = self.camera_info.k[2], self.camera_info.k[5]
        y = y_cam + (u - cx) * (height - z) / focal_length
        x = x_cam + (v - cy) * (height - z) / focal_length
        return x, y

    def path_key(self, path_message):
        # start/goal are always the same two fixed maze openings, so they
        # can't tell two different mazes apart -- the full waypoint list is
        # what actually changes between one maze and the next
        return tuple((round(pose.pose.position.x, 1), round(pose.pose.position.y, 1)) for pose in path_message.poses)

    def path_callback(self, path_message):
        try:
            if self.camera_info is None or self.joint_state is None or self.executing:
                return
            if not self.cartesian_path_client.service_is_ready():
                return
            path_key = self.path_key(path_message)
            if path_key == self.executed_path_key:
                return
            waypoints = []
            for pose in path_message.poses:
                x, y = self.pixel_to_world(pose.pose.position.x, pose.pose.position.y, TRACE_HEIGHT_M)
                waypoint = Pose()
                waypoint.position.x = x
                waypoint.position.y = y
                waypoint.position.z = TRACE_HEIGHT_M
                # laser_link's local +Z (its pointing axis) faces world -Z --
                # a 180 degree rotation about X does that and there's no
                # reason to prefer any particular roll about the pointing
                # axis itself, so this one fixed orientation covers the
                # whole path.
                waypoint.orientation.x = 1.0
                waypoint.orientation.w = 0.0
                waypoints.append(waypoint)
            request = GetCartesianPath.Request()
            request.header.frame_id = 'world'
            # move_group's own CurrentStateMonitor can be a step behind right
            # after maze_reset_manager's direct (non-MoveIt2) homing publish,
            # which then fails the next Cartesian path's start-state
            # validation ("start point deviates from current robot state") --
            # supplying our own live-tracked state sidesteps that race
            # entirely instead of trusting move_group's cached one.
            request.start_state = RobotState(joint_state=self.joint_state)
            request.group_name = GROUP_NAME
            request.link_name = LINK_NAME
            request.waypoints = waypoints
            request.max_step = MAX_STEP_M
            request.jump_threshold = 0.0
            # the maze walls are a standalone Gazebo model, never published
            # into MoveIt's planning scene as a CollisionObject, so collision
            # avoidance here couldn't see them anyway -- path_planner.py's
            # A* is what actually keeps the path inside the corridor
            request.avoid_collisions = False
            request.max_velocity_scaling_factor = VELOCITY_SAFETY_FACTOR
            request.max_acceleration_scaling_factor = VELOCITY_SAFETY_FACTOR
            self.executing = True
            self.executed_path_key = path_key
            future = self.cartesian_path_client.call_async(request)
            future.add_done_callback(self.cartesian_path_done)
        except Exception:
            self.executing = False

    def cartesian_path_done(self, future):
        try:
            response = future.result()
            points = len(response.solution.joint_trajectory.points)
            if response.fraction < 0.99:
                self.get_logger().warning(f'cartesian path only {response.fraction * 100:.0f}% achievable ({points} points), executing anyway')
            else:
                self.get_logger().info(f'computed cartesian path: {points} points')
            goal = ExecuteTrajectory.Goal()
            goal.trajectory = response.solution
            send_future = self.execute_trajectory_client.send_goal_async(goal)
            send_future.add_done_callback(self.execute_goal_response)
        except Exception:
            self.executing = False

    def execute_goal_response(self, future):
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().warning('move_group rejected the trajectory execution goal')
                self.executing = False
                return
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self.execute_result)
        except Exception:
            self.executing = False

    def execute_result(self, future):
        try:
            result = future.result().result
            self.get_logger().info(f'trajectory execution finished: error_code={result.error_code.val}')
        except Exception:
            pass
        finally:
            self.executing = False

def main(args=None):
    r.init(args=args)
    node = MotionExecutor()
    r.spin(node)
    node.destroy_node()
    r.shutdown()

if __name__ == '__main__':
    main()
