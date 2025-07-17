#!usr/bin.env python3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from turtlesim.srv import SetPen
from functools import partial
from geometry_msgs.msg import Twist

class PID():
    def __init__ (self, p=0.0, i=0.0, d=0.0):
        self.p = p
        self.i = i
        self.d = d
        self.previous_error = 0.0
        self.integral = 0.0

    def compute(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt
        output = self.p * error + self.i * self.integral + self.d * derivative
        self.previous_error = error
        return output

class Point():
    def __init__(self, x=0.0, y=0.0):
        self.x = x
        self.y = y

    def distance(self, other):
        return ((self.x - other.x) ** 2 + (self.y - other.y) ** 2) ** 0.5
    
    def angle_to(self, other):
        import math
        return math.atan2(other.y - self.y, other.x - self.x)


class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller')
        self.get_logger().info('Turtle Controller Node has been started')
        self.pose_subscriber_ = self.create_subscription(Pose, "/turtle1/pose", self.pose_callback, 10)
        self.cmd_vel_publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.previous_x = 5.5
        
    def avoid_walls(self, x, y, theta):
        twist = Twist()
        wall_distance = 1.0
        
        if not hasattr(self, 'heading_pid'):
            self.heading_pid = PID(p=0.4, i=0.0, d=0.1)
            self.last_time = self.get_clock().now()
        
        current_time = self.get_clock().now()

        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        if dt > 0:
            center = Point(5.5, 5.5)
            turtle_position = Point(x, y)
            heading_error = turtle_position.angle_to(center) - theta

            import math
            if heading_error > math.pi:
                heading_error -= 2 * math.pi
            elif heading_error < -math.pi:
                heading_error += 2 * math.pi
            twist.angular.z = self.heading_pid.compute(heading_error, dt)
        else:
            twist.angular.z = 0.0
        
        twist.linear.x = 2.0
        self.cmd_vel_publisher_.publish(twist)



    def pen_service_call(self, r, g, b, width, off):        
        client = self.create_client(SetPen, '/turtle1/set_pen')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('waiting for service')

        request = SetPen.Request()
        request.r = r
        request.g = g
        request.b = b
        request.width = width
        request.off = off

        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_set_pen))
    
    def callback_set_pen(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error("serrvice call failed: %r" % (e,))

    def pose_callback(self, msg):
        self.get_logger().info('Turtle Position: (' + str(msg.x) + ', ' + str(msg.y) + ', ' + str(msg.theta) + ')')
        if msg.x < 5.5 and self.previous_x >= 5.5:
            # move to left
            self.pen_service_call(0, 255, 0, 3, 0)
        elif msg.x > 5.5 and self.previous_x < 5.5:
            # move to right
            self.pen_service_call(255, 0, 0, 3, 0)

        self.avoid_walls(msg.x, msg.y, msg.theta)

        self.previous_x = msg.x

def main(args=None):
    rclpy.init(args=args)
    node = TurtleController()
    rclpy.spin(node)  # Keeps the node running
    rclpy.shutdown()

if __name__ == '__main__':  # This is not needed if you only run the node directly with ros2 run
    main()