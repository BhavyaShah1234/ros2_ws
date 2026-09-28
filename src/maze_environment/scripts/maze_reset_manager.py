#!/usr/bin/python3
import os
import subprocess
import tempfile
import yaml
import tf2_ros
import rclpy as r
from ament_index_python.packages import get_package_share_directory
from mazelib import Maze
from mazelib.generate.BacktrackingGenerator import BacktrackingGenerator
from moveit_msgs.action import ExecuteTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

WORLD_NAME = 'empty'
MAZE_ENTITY_NAME = 'calibration_maze'
GOAL_TOLERANCE_M = 0.03
AWAY_TOLERANCE_M = 0.15
HOME_ANGLES = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
HOME_SECONDS = 4.0
JOINT_NAMES = [f'fr3_joint{i}' for i in range(1, 8)]

class MazeResetManager(Node):
    def __init__(self):
        super(MazeResetManager, self).__init__(node_name='maze_reset_manager')
        share_dir = get_package_share_directory('maze_environment')
        with open(os.path.join(share_dir, 'config', 'camera_and_maze.yaml'), 'r') as f:
            self.corners = yaml.safe_load(f)['maze']
        with open(os.path.join(share_dir, 'config', 'maze_layout.yaml'), 'r') as f:
            self.maze_cfg = yaml.safe_load(f)['maze']
        self.goal_x, self.goal_y = self.compute_goal_position()
        self.triggered = False
        self.joint_state = None
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.execute_trajectory_client = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')
        self.digitizer_reset_publisher = self.create_publisher(Empty, '/maze_digitizer/reset', 10)
        self.create_timer(0.1, self.check_goal)
        self.get_logger().info(f'maze_reset_manager started, goal at world ({self.goal_x:.3f}, {self.goal_y:.3f})')

    def joint_state_callback(self, joint_state_message):
        self.joint_state = joint_state_message

    def bounds(self):
        xs = [c['x'] for c in self.corners.values()]
        ys = [c['y'] for c in self.corners.values()]
        return min(xs), max(xs), min(ys), max(ys)

    def compute_goal_position(self):
        x_min, x_max, y_min, y_max = self.bounds()
        n = self.maze_cfg['cells_per_side']
        cell_size_x = (x_max - x_min) / n
        cell_size_y = (y_max - y_min) / n
        return x_min + (n - 0.5) * cell_size_x, y_min + n * cell_size_y

    def distance_to_goal(self):
        transform = self.tf_buffer.lookup_transform('world', 'laser_link', Time())
        dx = transform.transform.translation.x - self.goal_x
        dy = transform.transform.translation.y - self.goal_y
        return (dx * dx + dy * dy) ** 0.5

    def check_goal(self):
        try:
            distance = self.distance_to_goal()
            if self.triggered:
                if distance > AWAY_TOLERANCE_M:
                    self.triggered = False
            elif distance <= GOAL_TOLERANCE_M:
                self.triggered = True
                self.get_logger().info('laser reached the goal, resetting')
                self.go_home()
        except Exception:
            pass

    def go_home(self):
        # Routed through MoveIt2's own execute_trajectory action (same one
        # motion_executor.py uses) rather than publishing straight to
        # /joint_trajectory_controller/joint_trajectory -- move_group's
        # moveit_simple_controller_manager now owns that controller, and a
        # raw publish here while it still believes a maze trace is executing
        # raced it badly in testing (a stale "trajectory execution finished"
        # arriving seconds late, right as the NEXT maze's path was already
        # being computed).
        #
        # Fully async (send_goal_async + done-callbacks), same as
        # motion_executor.py -- reset_maze() only runs once the result
        # callback confirms the home move actually finished. An earlier
        # version blocked here with rclpy.spin_until_future_complete, called
        # from inside this already-spinning timer callback: that hung
        # outright on the SECOND future (results never delivered) even
        # though it worked, seemingly by luck, on the first -- nested
        # spinning like that isn't reliable and shouldn't be used again.
        if self.joint_state is None:
            self.get_logger().warning('no /joint_states yet, skipping home move')
            return
        # ExecuteTrajectory's own start-state validation checks the
        # trajectory's FIRST point against the robot's actual current state
        # (unlike the old raw topic publish, which didn't care) -- a
        # target-only single-point trajectory always fails that unless the
        # arm happens to already be at HOME_ANGLES, so the current state has
        # to be the explicit first point. allowed_start_tolerance is also
        # widened in franka_with_overhead_camera.launch.py: Gazebo's joints
        # haven't always fully settled to zero velocity the instant a
        # trajectory reports finished, which the default 0.01 tolerance
        # could fail on even with a fresh state read here.
        current_by_name = dict(zip(self.joint_state.name, self.joint_state.position))
        start_point = JointTrajectoryPoint()
        start_point.positions = [current_by_name[name] for name in JOINT_NAMES]
        start_point.time_from_start.sec = 0
        home_point = JointTrajectoryPoint()
        home_point.positions = HOME_ANGLES
        home_point.time_from_start.sec = int(HOME_SECONDS)
        trajectory_message = JointTrajectory()
        trajectory_message.joint_names = JOINT_NAMES
        trajectory_message.points = [start_point, home_point]
        if not self.execute_trajectory_client.server_is_ready():
            self.get_logger().warning('execute_trajectory action server not available, skipping home move')
            return
        goal = ExecuteTrajectory.Goal()
        goal.trajectory.joint_trajectory = trajectory_message
        send_future = self.execute_trajectory_client.send_goal_async(goal)
        send_future.add_done_callback(self.home_goal_response)

    def home_goal_response(self, future):
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().warning('move_group rejected the home trajectory goal')
                self.reset_maze()
                return
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self.home_result)
        except Exception:
            self.reset_maze()

    def home_result(self, future):
        try:
            future.result()
        except Exception:
            pass
        finally:
            self.reset_maze()

    def generate_layout(self):
        n = self.maze_cfg['cells_per_side']
        maze = Maze()
        maze.generator = BacktrackingGenerator(n, n)
        maze.generate()
        grid = maze.grid
        segments = []
        for row in range(n + 1):
            for col in range(n):
                if row == 0 and col == 0:
                    continue
                if row == n and col == n - 1:
                    continue
                if row == 0 or row == n or grid[2 * row, 2 * col + 1]:
                    segments.append([[col, row], [col + 1, row]])
        for row in range(n):
            for col in range(n + 1):
                if col == 0 or col == n or grid[2 * row + 1, 2 * col]:
                    segments.append([[col, row], [col, row + 1]])
        return segments

    def make_maze_sdf(self, segments):
        x_min, x_max, y_min, y_max = self.bounds()
        n = self.maze_cfg['cells_per_side']
        thickness = self.maze_cfg['wall_thickness_m']
        height = self.maze_cfg['wall_height_m']
        cell_size_x = (x_max - x_min) / n
        cell_size_y = (y_max - y_min) / n

        def to_world(gx, gy):
            return (x_min + gx * cell_size_x, y_min + gy * cell_size_y)

        wall_elements = []
        for (gx1, gy1), (gx2, gy2) in segments:
            wx1, wy1 = to_world(gx1, gy1)
            wx2, wy2 = to_world(gx2, gy2)
            cx, cy = (wx1 + wx2) / 2.0, (wy1 + wy2) / 2.0
            length = ((wx2 - wx1) ** 2 + (wy2 - wy1) ** 2) ** 0.5
            if abs(wy2 - wy1) < 1e-9:
                size_x, size_y = length, thickness
            else:
                size_x, size_y = thickness, length
            wall_elements.append(f'''
      <visual name="wall_visual_{len(wall_elements)}">
        <pose>{cx} {cy} {height / 2.0} 0 0 0</pose>
        <geometry>
          <box>
            <size>{size_x} {size_y} {height}</size>
          </box>
        </geometry>
        <material>
          <ambient>0.1 0.7 0.1 1</ambient>
          <diffuse>0.1 0.7 0.1 1</diffuse>
        </material>
      </visual>
      <collision name="wall_collision_{len(wall_elements)}">
        <pose>{cx} {cy} {height / 2.0} 0 0 0</pose>
        <geometry>
          <box>
            <size>{size_x} {size_y} {height}</size>
          </box>
        </geometry>
      </collision>''')

        return f'''<?xml version="1.0" ?>
<sdf version="1.9">
  <model name="{MAZE_ENTITY_NAME}">
    <static>true</static>
    <link name="link">{''.join(wall_elements)}
    </link>
  </model>
</sdf>'''

    def reset_maze(self):
        subprocess.run([
            'gz', 'service', '-s', f'/world/{WORLD_NAME}/remove',
            '--reqtype', 'gz.msgs.Entity', '--reptype', 'gz.msgs.Boolean',
            '--timeout', '2000', '--req', f'name: "{MAZE_ENTITY_NAME}" type: MODEL',
        ], capture_output=True, text=True, timeout=3.0)

        sdf_content = self.make_maze_sdf(self.generate_layout())
        with tempfile.NamedTemporaryFile(mode='w', suffix='.sdf', delete=False) as sdf_file:
            sdf_file.write(sdf_content)
            sdf_path = sdf_file.name
        subprocess.run([
            'gz', 'service', '-s', f'/world/{WORLD_NAME}/create',
            '--reqtype', 'gz.msgs.EntityFactory', '--reptype', 'gz.msgs.Boolean',
            '--timeout', '2000', '--req', f'sdf_filename: "{sdf_path}" name: "{MAZE_ENTITY_NAME}"',
        ], capture_output=True, text=True, timeout=3.0)
        # deliberately not unlinking sdf_path: the create service call above
        # returns as soon as gz accepts the request, not once it has actually
        # read the file, so deleting it here raced Gazebo's own read often
        # enough in testing to leave the maze unspawned ("Unable to read
        # file"). Leaking a few KB per reset in /tmp is a fine trade for that.
        self.digitizer_reset_publisher.publish(Empty())
        self.get_logger().info('maze reset')

def main(args=None):
    r.init(args=args)
    node = MazeResetManager()
    r.spin(node)
    node.destroy_node()
    r.shutdown()

if __name__ == '__main__':
    main()
