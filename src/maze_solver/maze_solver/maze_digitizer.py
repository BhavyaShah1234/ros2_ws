import cv2
import rclpy as r
import yaml
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from nav_msgs.msg import OccupancyGrid

class MazeDigitizer(Node):
    def __init__(self):
        config_path = get_package_share_directory('maze_solver') + '/config/ros_interfaces.yaml'
        with open(config_path, 'r') as file:
            config = yaml.safe_load(file)
        super(MazeDigitizer, self).__init__(node_name=config['nodes']['maze_digitizer'])
        self.bridge = CvBridge()
        self.create_subscription(Image, config['topics']['overhead_camera_image'], self.callback, 10)
        grid_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.grid_publisher = self.create_publisher(OccupancyGrid, config['topics']['maze_occupancy_grid'], grid_qos)

    def callback(self, image_message):
        image = self.decode_image(image_message)
        grid_message = self.digitize_maze(image)
        if grid_message is not None:
            grid_message.header.stamp = image_message.header.stamp
            grid_message.header.frame_id = image_message.header.frame_id
            self.grid_publisher.publish(grid_message)

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
        raise NotImplementedError

def main(args=None):
    r.init(args=args)
    node = MazeDigitizer()
    r.spin(node)
    node.destroy_node()
    r.shutdown()

if __name__ == '__main__':
    main()
