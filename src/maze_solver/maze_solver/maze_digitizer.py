import cv2
import numpy as np
import rclpy as r
from itertools import groupby
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose, PoseArray
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Empty

STABLE_FRAMES = 5

class MazeDigitizer(Node):
    def __init__(self):
        super(MazeDigitizer, self).__init__(node_name='maze_digitizer')
        self.bridge = CvBridge()
        self.triggered = False
        self.candidate_data = None
        self.stable_count = 0
        self.frozen_grid = None
        self.frozen_goals = None
        self.create_subscription(Image, '/overhead_camera/image', self.callback, 10)
        self.create_subscription(Empty, '/maze_digitizer/reset', self.reset_callback, 10)
        grid_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.grid_publisher = self.create_publisher(OccupancyGrid, 'maze_occupancy_grid', grid_qos)
        self.goals_publisher = self.create_publisher(PoseArray, '/goals', grid_qos)
        self.get_logger().info('maze_digitizer started')

    def decode_image(self, image_message):
        image = self.bridge.imgmsg_to_cv2(image_message)
        if image_message.encoding == 'rgb8':
            return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        if image_message.encoding == 'rgba8':
            return cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)
        if image_message.encoding == 'bgra8':
            return cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
        return image

    def digitize_maze(self, image):
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv_image, (35, 40, 40), (85, 255, 255))
        grid_message = OccupancyGrid()
        grid_message.info.resolution = 1.0
        grid_message.info.width = mask.shape[1]
        grid_message.info.height = mask.shape[0]
        grid_message.info.origin.orientation.w = 1.0
        grid_message.data = (mask // 255 * 100).astype('int8').flatten().tolist()
        return grid_message

    def occupied_grid(self, like):
        grid_message = OccupancyGrid()
        grid_message.info.resolution = 1.0
        grid_message.info.width = like.info.width
        grid_message.info.height = like.info.height
        grid_message.info.origin.orientation.w = 1.0
        grid_message.data = [100] * (like.info.width * like.info.height)
        return grid_message

    def compute_goals(self, grid_message):
        wall = (np.asarray(grid_message.data, dtype=np.int8) > 0).astype(np.uint8).reshape(grid_message.info.height, grid_message.info.width)
        count, labels, stats, _ = cv2.connectedComponentsWithStats(wall)
        keep = [i for i in range(1, count) if stats[i, cv2.CC_STAT_AREA] >= 0.05 * stats[1:, cv2.CC_STAT_AREA].sum()]
        wall = np.isin(labels, keep)
        x, y, w, h = cv2.boundingRect(wall.astype(np.uint8))
        band = max(4, w // 50)
        openings = []
        # start is the opening in the left border (world y_min, the maze_layout entrance), goal the one in the right border
        for u, columns in ((x + band // 2, slice(x, x + band)), (x + w - 1 - band // 2, slice(x + w - band, x + w))):
            rows, row = [], 0
            for solid, group in groupby(wall[y:y + h, columns].any(axis=1)):
                length = len(list(group))
                if not solid and h // 20 <= length <= h // 8:
                    rows.append(y + row + length // 2)
                row += length
            if len(rows) != 1:
                return None
            openings.append((u, rows[0]))
        goals_message = PoseArray()
        for u, v in openings:
            pose = Pose()
            pose.position.x = float(u)
            pose.position.y = float(v)
            pose.orientation.w = 1.0
            goals_message.poses.append(pose)
        return goals_message

    def reset_callback(self, empty_message):
        self.triggered = False
        self.candidate_data = None
        self.stable_count = 0
        self.frozen_grid = None
        self.frozen_goals = None
        self.get_logger().info('maze_digitizer reset, looking for a new stable frame')

    def callback(self, image_message):
        try:
            if self.triggered:
                self.grid_publisher.publish(self.frozen_grid)
                self.goals_publisher.publish(self.frozen_goals)
                return
            image = self.decode_image(image_message)
            grid_message = self.digitize_maze(image)
            grid_message.header.stamp = image_message.header.stamp
            grid_message.header.frame_id = image_message.header.frame_id
            # the arm crossing the camera's view mid-motion changes the mask
            # every frame (it isn't green, so it just looks like missing
            # wall), so only trust a frame once several in a row agree --
            # the arm sitting still for that long only happens before motion
            # starts or once it is safely back at rest between mazes
            if grid_message.data == self.candidate_data:
                self.stable_count += 1
            else:
                self.candidate_data = grid_message.data
                self.stable_count = 1
            if self.stable_count >= STABLE_FRAMES:
                goals_message = self.compute_goals(grid_message)
                if goals_message is not None:
                    goals_message.header = grid_message.header
                    self.frozen_grid = grid_message
                    self.frozen_goals = goals_message
                    self.triggered = True
                    self.grid_publisher.publish(self.frozen_grid)
                    self.goals_publisher.publish(self.frozen_goals)
                    start, goal = goals_message.poses
                    self.get_logger().info(f'triggered on a stable frame: start ({start.position.x:.0f}, {start.position.y:.0f}) goal ({goal.position.x:.0f}, {goal.position.y:.0f})')
                    return
            occupied_message = self.occupied_grid(grid_message)
            occupied_message.header = grid_message.header
            self.grid_publisher.publish(occupied_message)
        except Exception:
            pass

def main(args=None):
    r.init(args=args)
    node = MazeDigitizer()
    r.spin(node)
    node.destroy_node()
    r.shutdown()

if __name__ == '__main__':
    main()
