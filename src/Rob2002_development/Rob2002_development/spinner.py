import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

class MoverSpinner(Node):

    def __init__(self):
        super().__init__('mover_spinner')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):              
        msg = Twist()
        msg.angular.z = 1.0
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg}"')
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    mover_spinner = MoverSpinner()

    rclpy.spin(mover_spinner)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    mover_spinner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()