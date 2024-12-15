import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class MoverLaser(Node):
    """
    A very simple Roamer implementation for LIMO.
    It goes straight until any obstacle is within
    a specified distance and then just turns left.
    A purely reactive approach.
    """
    def __init__(self):
        """
        On construction of the object, create a Subscriber
        to listen to lasr scans and a Publisher to control
        the robot
        """
        super().__init__('mover_laser')
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.subscriber = self.create_subscription(LaserScan, "/scan", self.laserscan_callback, 10)

        self.angular_range = 10
    
    def laserscan_callback(self, data):
        """
        Callback called any time a new laser scan become available
        """
        min_dist = min(data.ranges[int(len(data.ranges)/2) - self.angular_range : int(len(data.ranges)/2) + self.angular_range])
        print(f'Min distance: {min_dist:.2f}')
        msg = Twist()
        if min_dist < 0.5:
            msg.angular.z = 0.5
        else:
            msg.linear.x = 0.2
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    mvoer_laser = MoverLaser()
    rclpy.spin(mvoer_laser)

    mvoer_laser.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()