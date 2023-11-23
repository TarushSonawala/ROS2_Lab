#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import String
from example_interfaces.msg import String, Int32
from my_robot_interface.msg import ManufactureDate

class MyNode(Node):

    def __init__(self):
        super().__init__('number_publisher')
        self.publisher_ = self.create_publisher(String, 'number', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = '%d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('%s' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    node = MyNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()