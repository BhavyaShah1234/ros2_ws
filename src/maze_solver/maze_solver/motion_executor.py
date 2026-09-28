import os
import tempfile
import xml.dom.minidom as minidom
import ikpy.chain
import rclpy as r
import tf2_ros
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import CameraInfo, JointState
from std_msgs.msg import String
from nav_msgs.msg import Path
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

BASE_ELEMENTS = [
    'world', 'world_joint', 'fr3_link0',
    'fr3_joint1', 'fr3_link1',
    'fr3_joint2', 'fr3_link2',
    'fr3_joint3', 'fr3_link3',
    'fr3_joint4', 'fr3_link4',
    'fr3_joint5', 'fr3_link5',
    'fr3_joint6', 'fr3_link6',
    'fr3_joint7', 'fr3_link7',
    'fr3_joint8', 'fr3_link8',
    'laser_joint', 'laser_link',
]
ACTIVE_LINKS_MASK = [False, False, True, True, True, True, True, True, True, False, False]
TRACE_HEIGHT_M = 0.015
MIN_STEP_SECONDS = 0.15
FIRST_WAYPOINT_SECONDS = 3.0
# Real FR3 joints can move much faster than this, but the trajectory
# controller here has no joint_limits configured to catch an infeasible
# request itself (see config/franka_gazebo_controllers.yaml), so a segment
# demanding more than the joint's rated velocity is only caught by Gazebo's
# physics failing to track it -- which looks like the arm flying apart.
# Planning against a fraction of the real limit leaves headroom for a
# spline/interpolated move's peak velocity exceeding its point-to-point
# average.
VELOCITY_SAFETY_FACTOR = 0.5

class MotionExecutor(Node):
    def __init__(self):
        super(MotionExecutor, self).__init__(node_name='motion_executor')
        self.camera_info = None
        self.chain = None
        self.joint_names = None
        self.rest_angles = None
        self.rest_angles_active = None
        self.velocity_limits = None
        self.joint_state = {}
        self.busy_until = None
        self.executed_path_key = None
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        latched_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(CameraInfo, '/overhead_camera/camera_info', self.camera_info_callback, 10)
        self.create_subscription(String, '/robot_description', self.robot_description_callback, latched_qos)
        self.create_subscription(Path, '/path', self.path_callback, latched_qos)
        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.trajectory_publisher = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        self.get_logger().info('motion_executor started')

    def camera_info_callback(self, camera_info_message):
        self.camera_info = camera_info_message

    def joint_state_callback(self, joint_state_message):
        self.joint_state = dict(zip(joint_state_message.name, joint_state_message.position))

    def robot_description_callback(self, robot_description_message):
        if self.chain is not None:
            return
        with tempfile.NamedTemporaryFile(mode='w', suffix='.urdf', delete=False) as urdf_file:
            urdf_file.write(robot_description_message.data)
            urdf_path = urdf_file.name
        self.chain = ikpy.chain.Chain.from_urdf_file(urdf_path, base_elements=BASE_ELEMENTS, active_links_mask=ACTIVE_LINKS_MASK, name='fr3_laser')
        self.joint_names = [link.name for link, active in zip(self.chain.links, ACTIVE_LINKS_MASK) if active]
        urdf_doc = minidom.parse(urdf_path)
        os.unlink(urdf_path)
        # the same joint name also appears, with no <limit> child, inside
        # each <transmission> block further down the document -- keep only
        # the first (real, type="revolute") definition of each name
        joints_by_name = {}
        for joint in urdf_doc.getElementsByTagName('joint'):
            if joint.getAttribute('type') == 'revolute':
                joints_by_name.setdefault(joint.getAttribute('name'), joint)
        self.velocity_limits = [float(joints_by_name[name].getElementsByTagName('limit')[0].getAttribute('velocity')) for name in self.joint_names]
        self.rest_angles = [0.0] * len(self.chain.links)
        for i, active in enumerate(ACTIVE_LINKS_MASK):
            if active:
                lo, hi = self.chain.links[i].bounds
                self.rest_angles[i] = (lo + hi) / 2.0
        self.rest_angles_active = [angle for angle, active in zip(self.rest_angles, ACTIVE_LINKS_MASK) if active]
        self.get_logger().info('built ikpy chain from robot_description')

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

    def segment_seconds(self, angles_from, angles_to):
        seconds = MIN_STEP_SECONDS
        for a, b, limit in zip(angles_from, angles_to, self.velocity_limits):
            seconds = max(seconds, abs(b - a) / (VELOCITY_SAFETY_FACTOR * limit))
        return seconds

    def path_key(self, path_message):
        # start/goal are always the same two fixed maze openings, so they
        # can't tell two different mazes apart -- the full waypoint list is
        # what actually changes between one maze and the next
        return tuple((round(pose.pose.position.x, 1), round(pose.pose.position.y, 1)) for pose in path_message.poses)

    def path_callback(self, path_message):
        try:
            if self.chain is None or self.camera_info is None:
                return
            if self.busy_until is not None and self.get_clock().now() < self.busy_until:
                return
            # maze_digitizer/path_planner keep publishing the same solved
            # path every frame even after the arm reaches the goal, right up
            # until the camera actually sees a new maze -- without this,
            # motion_executor would immediately re-solve and re-drive the
            # maze it just finished the moment busy_until clears, racing
            # maze_reset_manager's own "go home" command
            path_key = self.path_key(path_message)
            if path_key == self.executed_path_key:
                return
            angles = list(self.rest_angles)
            current_angles = [self.joint_state.get(name, fallback) for name, fallback in zip(self.joint_names, self.rest_angles_active)]
            points = []
            time_from_start = 0.0
            for pose in path_message.poses:
                x, y = self.pixel_to_world(pose.pose.position.x, pose.pose.position.y, TRACE_HEIGHT_M)
                angles = self.chain.inverse_kinematics([x, y, TRACE_HEIGHT_M], target_orientation=[0, 0, -1], orientation_mode='Z', initial_position=angles)
                new_angles = [angle for angle, active in zip(angles, ACTIVE_LINKS_MASK) if active]
                duration = self.segment_seconds(current_angles, new_angles)
                if not points:
                    duration = max(duration, FIRST_WAYPOINT_SECONDS)
                time_from_start += duration
                point = JointTrajectoryPoint()
                point.positions = new_angles
                point.time_from_start.sec = int(time_from_start)
                point.time_from_start.nanosec = int(round((time_from_start - int(time_from_start)) * 1e9))
                points.append(point)
                current_angles = new_angles
            trajectory_message = JointTrajectory()
            trajectory_message.joint_names = self.joint_names
            trajectory_message.points = points
            self.trajectory_publisher.publish(trajectory_message)
            self.executed_path_key = path_key
            self.busy_until = self.get_clock().now() + Duration(seconds=time_from_start)
            self.get_logger().info(f'published trajectory: {len(points)} points over {time_from_start:.1f}s')
        except Exception:
            pass

def main(args=None):
    r.init(args=args)
    node = MotionExecutor()
    r.spin(node)
    node.destroy_node()
    r.shutdown()

if __name__ == '__main__':
    main()
