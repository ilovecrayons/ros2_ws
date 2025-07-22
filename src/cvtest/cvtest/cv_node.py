#usr/bin.env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from .linreg import process_linreg
from .colorSegment import process_color_segmentation
import time

class CvNode(Node):
    def __init__(self):
        super().__init__('cv_node')
        self.get_logger().info('CV Node has been started')
        
        # Separate timing control for each camera
        self.last_downward_callback_time = self.get_clock().now()
        self.last_front_callback_time = self.get_clock().now()
        self.min_interval = 0.07
        
        # Downward camera subscriber (for linear regression)
        self.downward_camera_subscriber_ = self.create_subscription(
            Image, '/camera/downward/image_raw', self.downward_camera_callback, 10)
        self.downward_camera_subscriber_.use_intra_process_comms = True # type: ignore
        
        # Front camera subscriber (for color segmentation)
        self.front_camera_subscriber_ = self.create_subscription(
            Image, '/camera/front/image_raw', self.front_camera_callback, 10)
        self.front_camera_subscriber_.use_intra_process_comms = True # type: ignore

        # Publishers
        self.color_segmentation_publisher_ = self.create_publisher(Image, '/processed/color_segmentation', 10)
        self.linear_regression_publisher_ = self.create_publisher(Image, '/processed/linear_regression', 10)
        
        self.get_logger().info('Subscribed to downward camera: /camera/downward/image_raw')
        self.get_logger().info('Subscribed to front camera: /camera/front/image_raw')
        
        
    def downward_camera_callback(self, msg):
        """Process downward camera feed for linear regression"""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_downward_callback_time).nanoseconds / 1e9
        if dt < self.min_interval:
            return  # Exit early without processing
        
        callback_start = time.perf_counter()
        print(f"[DOWNWARD] Callback started (dt: {dt:.3f}s)")
        
        self.last_downward_callback_time = current_time

        if not hasattr(self, 'bridge'):
            self.bridge = CvBridge()
        
        try:
            conversion_start = time.perf_counter()
            if msg.encoding == 'mono8' or msg.encoding == 'mono16':
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)
            else:
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            conversion_time = time.perf_counter() - conversion_start
            print(f"[DOWNWARD] Image conversion took: {conversion_time*1000:.3f}ms (image size: {cv_image.shape})")
     
            # Only process linear regression for downward camera
            linreg_start = time.perf_counter()
            linreg_result = process_linreg(cv_image)
            linreg_time = time.perf_counter() - linreg_start
            print(f"[DOWNWARD] Linear regression took: {linreg_time*1000:.3f}ms")
            
            if linreg_result is not None:
                publish_start = time.perf_counter()
                linreg_msg = self.bridge.cv2_to_imgmsg(linreg_result, encoding='bgr8')
                linreg_msg.header = msg.header
                self.linear_regression_publisher_.publish(linreg_msg)
                publish_time = time.perf_counter() - publish_start
                print(f"[DOWNWARD] Linear regression publishing took: {publish_time*1000:.3f}ms")
            
            callback_total = time.perf_counter() - callback_start
            print(f"[DOWNWARD] Total callback took: {callback_total*1000:.3f}ms")
            print("=" * 50)
            
        except Exception as e:
            self.get_logger().error(f'Error processing downward camera: {str(e)}')

    def front_camera_callback(self, msg):
        """Process front camera feed for color segmentation"""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_front_callback_time).nanoseconds / 1e9
        if dt < self.min_interval:
            return  # Exit early without processing
        
        callback_start = time.perf_counter()
        print(f"[FRONT] Callback started (dt: {dt:.3f}s)")
        
        self.last_front_callback_time = current_time

        if not hasattr(self, 'bridge'):
            self.bridge = CvBridge()
        
        try:
            conversion_start = time.perf_counter()
            if msg.encoding == 'mono8' or msg.encoding == 'mono16':
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)
            else:
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            conversion_time = time.perf_counter() - conversion_start
            print(f"[FRONT] Image conversion took: {conversion_time*1000:.3f}ms (image size: {cv_image.shape})")

            # Only process color segmentation for front camera
            color_seg_start = time.perf_counter()
            result_image, detected_objects = process_color_segmentation(cv_image)
            color_seg_time = time.perf_counter() - color_seg_start
            print(f"[FRONT] Color segmentation took: {color_seg_time*1000:.3f}ms")
            
            color_publish_start = time.perf_counter()
            color_seg_msg = self.bridge.cv2_to_imgmsg(result_image, encoding='bgr8')
            color_seg_msg.header = msg.header  
            self.color_segmentation_publisher_.publish(color_seg_msg)
            color_publish_time = time.perf_counter() - color_publish_start
            print(f"[FRONT] Color segmentation publishing took: {color_publish_time*1000:.3f}ms")
            
            callback_total = time.perf_counter() - callback_start
            print(f"[FRONT] Total callback took: {callback_total*1000:.3f}ms")
            print("=" * 50)
            
        except Exception as e:
            self.get_logger().error(f'Error processing front camera: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = CvNode()
    rclpy.spin(node)  
    rclpy.shutdown()

if __name__ == '__main__':
    main()