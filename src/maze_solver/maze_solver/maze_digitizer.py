import numpy as np
import rclpy as r
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from nav_msgs.msg import OccupancyGrid

class MazeDigitizer(Node):
    def __init__(self):
        super(MazeDigitizer, self).__init__(node_name='maze_digitizer')
        self.create_subscription(Image, '/overhead_camera/image', self.callback, 10)
        grid_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.grid_publisher = self.create_publisher(OccupancyGrid, '/maze_occupancy_grid', grid_qos)

    def callback(self, image_message):
        image = self.decode_image(image_message)
        grid_message = self.digitize_maze(image)
        if grid_message is not None:
            grid_message.header.stamp = image_message.header.stamp
            grid_message.header.frame_id = image_message.header.frame_id
            self.grid_publisher.publish(grid_message)

    def decode_image(self, image_message):
        array = np.frombuffer(image_message.data, dtype=np.uint8)
        if image_message.encoding == 'rgb8':
            return array.reshape(image_message.height, image_message.width, 3)[:, :, ::-1]
        if image_message.encoding == 'bgr8':
            return array.reshape(image_message.height, image_message.width, 3).copy()
        if image_message.encoding == 'rgba8':
            return array.reshape(image_message.height, image_message.width, 4)[:, :, [2, 1, 0]]
        if image_message.encoding == 'bgra8':
            return array.reshape(image_message.height, image_message.width, 4)[:, :, :3].copy()
        raise ValueError(f'Unsupported image encoding: {image_message.encoding}')

    def digitize_maze(self, image):
        raise NotImplementedError

def main(args=None):
    r.init(args=args)
    node = MazeDigitizer()
    r.spin(node)
    node.destroy_node()
    r.shutdown()

if __name__ == '__main__':
    main()
