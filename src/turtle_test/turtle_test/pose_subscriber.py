#!/usr/bin.env python3
import rclpy
from rclpy.node import Node
#if you import more libraries, add them to dependencies in package.xml
from turtlesim.msg import Pose

class PoseSubNode(Node):
    def __init__(self):
        super().__init__('pose_subscriber')
        self.get_logger().info('Node One has been started')
        self.pose_subscriber_ = self.create_subscription(Pose, "/turtle1/pose", self.pose_callback, 10)

        # Add your node logic here
    def pose_callback(self, msg):
        self.get_logger().info('('+str(msg.x)+', '+str(msg.y)+str(msg.theta)+')')
        # For example, you can create a timer, subscribe to topics, etc.
def main(args=None):
    rclpy.init(args=args)
    node = PoseSubNode()
    rclpy.spin(node) #Loops the node
    rclpy.shutdown()


if __name__=='__main__':#Unneeded if you only ever run the node directly with ros2 run 
    main()