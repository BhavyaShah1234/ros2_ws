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
# Node structure only: subscribes to the overhead camera's image topic,
# hands each frame to a computer-vision placeholder that reconstructs the
# maze's wall layout, and publishes the result as an OccupancyGrid. The CV
# logic itself (color/edge/corner detection -> grid reconstruction) is not
# implemented here -- see digitize_maze() below.

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import Image

IMAGE_TOPIC = '/overhead_camera/image'
OCCUPANCY_GRID_TOPIC = '/maze_occupancy_grid'


def decode_image_msg(msg: Image) -> np.ndarray:
    """Decode a sensor_msgs/Image into a BGR uint8 numpy array.

    cv_bridge is broken in this environment (compiled against NumPy 1.x;
    the environment has NumPy 2.x, an ABI mismatch) -- sidestepped by
    decoding the message manually here instead of via cv_bridge.
    """
    arr = np.frombuffer(msg.data, dtype=np.uint8)
    if msg.encoding == 'rgb8':
        return arr.reshape(msg.height, msg.width, 3)[:, :, ::-1]
    if msg.encoding == 'bgr8':
        return arr.reshape(msg.height, msg.width, 3).copy()
    if msg.encoding == 'rgba8':
        return arr.reshape(msg.height, msg.width, 4)[:, :, [2, 1, 0]]
    if msg.encoding == 'bgra8':
        return arr.reshape(msg.height, msg.width, 4)[:, :, :3].copy()
    raise ValueError(f'Unsupported image encoding: {msg.encoding!r}')


class MazeDigitizer(Node):

    def __init__(self):
        super().__init__('maze_digitizer')

        self._image_sub = self.create_subscription(
            Image, IMAGE_TOPIC, self._image_callback, 10)

        # transient_local so a subscriber (e.g. path_planner) that comes up
        # after the maze has already been digitized still gets the latest
        # grid, instead of only future ones.
        grid_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._grid_pub = self.create_publisher(
            OccupancyGrid, OCCUPANCY_GRID_TOPIC, grid_qos)

        self.get_logger().info(
            f'maze_digitizer up: {IMAGE_TOPIC} -> {OCCUPANCY_GRID_TOPIC}')

    def _image_callback(self, msg: Image) -> None:
        image = decode_image_msg(msg)
        grid = self.digitize_maze(image)
        if grid is not None:
            grid.header.stamp = msg.header.stamp
            grid.header.frame_id = msg.header.frame_id
            self._grid_pub.publish(grid)

    def digitize_maze(self, image: np.ndarray) -> OccupancyGrid:
        """Reconstruct the maze's wall layout from one overhead camera
        frame and return it as an OccupancyGrid.

        `image` is a BGR uint8 numpy array (height, width, 3), decoded from
        the raw camera frame by decode_image_msg() above.

        TODO: color detection, edge detection, corner detection -> the
        occupancy grid's info (resolution, width, height, origin) and data
        (row-major int8, -1/0/100) all need to be filled in here.
        """
        raise NotImplementedError('digitize_maze() is not implemented yet')


def main(args=None):
    rclpy.init(args=args)
    node = MazeDigitizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
