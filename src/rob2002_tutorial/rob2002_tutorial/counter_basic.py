import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PolygonStamped

class CounterBasic(Node):

    def __init__(self):
        super().__init__('counter_basic')

        self.current_stamp = None
        self.object_counter = 0
        self.frame_counter = -1

        self.subscriber = self.create_subscription(PolygonStamped, "/object_polygon", self.counter_callback, 10)

    def counter_callback(self, data):
        
        if data.header.stamp != self.current_stamp: # new frame detected
            # report the current object count
            if self.frame_counter >= 0:
                print(f'frame {self.frame_counter}: {self.object_counter} objects counted.')            
        
            self.frame_counter += 1 # increment frame counter
            self.object_counter = 0 # reset object counter
            self.current_stamp = data.header.stamp # refresh the current time stamp           

        # keep counting objects
        self.object_counter += 1


def main(args=None):
    rclpy.init(args=args)
    counter_basic = CounterBasic()

    rclpy.spin(counter_basic)

    counter_basic.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()