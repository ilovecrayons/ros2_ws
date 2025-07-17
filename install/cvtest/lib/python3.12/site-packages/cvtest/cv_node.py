#usr/bin.env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

class CvNode(Node):
    def __init__(self):
        super().__init__('cv_node')
        self.get_logger().info('CV Node has been started')
        self.camera_subscriber_ = self.create_subscription(
            Image, '/camera/image_raw', self.camera_callback, 10)
        self.camera_subscriber_.use_intra_process_comms = True

    def camera_callback(self, msg):
        self.get_logger().info('Received image with width: %d, height: %d' % (msg.width, msg.height))
        
        
def main(args=None):
    rclpy.init(args=args)
    node = CvNode()
    rclpy.spin(node)  
    rclpy.shutdown()
        