#!/usr/bin/python3
import os
import subprocess
import tempfile
import time
import yaml
import tf2_ros
import rclpy as r
from ament_index_python.packages import get_package_share_directory
from mazelib import Maze
from mazelib.generate.BacktrackingGenerator import BacktrackingGenerator
from rclpy.node import Node
from rclpy.time import Time
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
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.trajectory_publisher = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        self.digitizer_reset_publisher = self.create_publisher(Empty, '/maze_digitizer/reset', 10)
        self.create_timer(0.1, self.check_goal)
        self.get_logger().info(f'maze_reset_manager started, goal at world ({self.goal_x:.3f}, {self.goal_y:.3f})')

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
                self.reset_maze()
        except Exception:
            pass

    def go_home(self):
        point = JointTrajectoryPoint()
        point.positions = HOME_ANGLES
        point.time_from_start.sec = int(HOME_SECONDS)
        trajectory_message = JointTrajectory()
        trajectory_message.joint_names = JOINT_NAMES
        trajectory_message.points = [point]
        self.trajectory_publisher.publish(trajectory_message)
        time.sleep(HOME_SECONDS + 0.5)

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
