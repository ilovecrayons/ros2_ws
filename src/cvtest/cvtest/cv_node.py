#usr/bin.env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class CvNode(Node):
    def __init__(self):
        super().__init__('cv_node')
        self.get_logger().info('CV Node has been started')
        self.camera_subscriber_ = self.create_subscription(
            Image, '/camera/image_raw', self.camera_callback, 10) 
        self.camera_subscriber_.use_intra_process_comms = True

    def camera_callback(self, msg):
        self.get_logger().info('Received image with width: %d, height: %d' % (msg.width, msg.height))
        image = msg.data

        if not hasattr(self, 'bridge'):
            self.bridge = CvBridge()
        self.get_logger().info(f'Image encoding: {msg.encoding}')

        # convert img format
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            height, width, channels = cv_image.shape
            self.get_logger().info(f'Processed image: {width}x{height}, {channels} channels')
            
        except Exception as e:
            self.get_logger().error(f'Error converting image: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = CvNode()
    rclpy.spin(node)  
    rclpy.shutdown()
        