import cv2
import heapq
import numpy as np
import rclpy as r
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseArray, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path

class PathPlanner(Node):
    def __init__(self):
        super(PathPlanner, self).__init__(node_name='path_planner')
        self.grid = None
        path_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, 'maze_occupancy_grid', self.grid_callback, path_qos)
        self.create_subscription(PoseArray, '/goals', self.goals_callback, path_qos)
        self.path_publisher = self.create_publisher(Path, '/path', path_qos)
        self.get_logger().info('path_planner started')

    def grid_callback(self, grid_message):
        self.grid = grid_message

    def downsample(self, grid_message, scale, inflate):
        wall = (np.asarray(grid_message.data, dtype=np.int8) > 0).astype(np.uint8).reshape(grid_message.info.height, grid_message.info.width)
        width, height = wall.shape[1] // scale, wall.shape[0] // scale
        small = cv2.resize(wall * 255, (width, height), interpolation=cv2.INTER_AREA)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * inflate + 1, 2 * inflate + 1))
        blocked = cv2.dilate((small > 0).astype(np.uint8), kernel) > 0
        # the free floor outside the maze connects the two boundary openings, so a plan
        # over the raw grid can shortcut around the maze instead of through it; fencing
        # everything outside the walls' own bounding box in as blocked forces it to stay inside
        rows, columns = np.nonzero(blocked)
        blocked[:rows.min(), :] = True
        blocked[rows.max() + 1:, :] = True
        blocked[:, :columns.min()] = True
        blocked[:, columns.max() + 1:] = True
        return blocked

    def heuristic(self, a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def neighbors(self, cell, width, height):
        x, y = cell
        candidates = [(x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)]
        return [candidate for candidate in candidates if 0 <= candidate[0] < width and 0 <= candidate[1] < height]

    def reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    def astar(self, blocked, start, goal):
        height, width = blocked.shape
        open_heap = [(self.heuristic(start, goal), 0, start)]
        cost_so_far = {start: 0}
        came_from = {}
        while open_heap:
            _, cost, current = heapq.heappop(open_heap)
            if current == goal:
                return self.reconstruct_path(came_from, current)
            if cost > cost_so_far[current]:
                continue
            for neighbor in self.neighbors(current, width, height):
                if blocked[neighbor[1], neighbor[0]]:
                    continue
                new_cost = cost + 1
                if new_cost < cost_so_far.get(neighbor, float('inf')):
                    cost_so_far[neighbor] = new_cost
                    came_from[neighbor] = current
                    heapq.heappush(open_heap, (new_cost + self.heuristic(neighbor, goal), new_cost, neighbor))
        return None

    def goals_callback(self, goals_message):
        try:
            if self.grid is None:
                return
            scale, inflate = 6, 1
            blocked = self.downsample(self.grid, scale, inflate)
            start_pose, goal_pose = goals_message.poses
            start = (int(start_pose.position.x) // scale, int(start_pose.position.y) // scale)
            goal = (int(goal_pose.position.x) // scale, int(goal_pose.position.y) // scale)
            cells = self.astar(blocked, start, goal)
            if cells is None:
                self.get_logger().warning('no path found through the maze', throttle_duration_sec=5.0)
                return
            path_message = Path()
            path_message.header = goals_message.header
            for x, y in cells:
                pose = PoseStamped()
                pose.header = goals_message.header
                pose.pose.position.x = float((x + 0.5) * scale)
                pose.pose.position.y = float((y + 0.5) * scale)
                pose.pose.orientation.w = 1.0
                path_message.poses.append(pose)
            self.path_publisher.publish(path_message)
            self.get_logger().info(f'published path: {len(cells)} waypoints', throttle_duration_sec=2.0)
        except Exception:
            pass

def main(args=None):
    r.init(args=args)
    node = PathPlanner()
    r.spin(node)
    node.destroy_node()
    r.shutdown()

if __name__ == '__main__':
    main()
