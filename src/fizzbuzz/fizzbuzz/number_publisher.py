from numpy import number
import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32, String

class NumberPublisher(Node):

    def __init__(self):
        super().__init__('number_publisher')
        self.publisher_ = self.create_publisher(Int32, 'numbers', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Int32()
        msg.data = self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%d"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    number_publisher = NumberPublisher()

    rclpy.spin(number_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    number_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()