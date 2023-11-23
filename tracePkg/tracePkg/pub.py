# !/usr/bin/env python3.10

import rclpy
from rclpy.node import Node
from example_interfaces.msg import String
from my_robot_interface.msg import ManufactureDate

class RobotDatePublisher(Node):
    def __init__(self):
        super().__init__("robot_publisher")
        self.robot_name_="ROBOT"
        self.publisher_ = self.create_publisher(ManufactureDate,"robot_manufacturing_date", 10)
        self.timer_ = self.create_timer(0.5, self.publish_news)
        self.get_logger().info("Node Started")
    def publish_news(self):
        msg = ManufactureDate()
        msg.date = 12
        msg.month = "March"
        msg.year = 2022
        self.publisher_.publish(msg)
            
def main(args=None):
    rclpy.init(args=args)
    node = RobotDatePublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
# import rclpy
# from rclpy.node import Node

# from std_msgs.msg import String


# class MinimalPublisher(Node):

#     def __init__(self):
#         super().__init__('minimal_publisher')
#         self.publisher_ = self.create_publisher(String, 'topic', 10)
#         timer_period = 0.5  # seconds
#         self.timer = self.create_timer(timer_period, self.timer_callback)
#         self.i = 0

#     def timer_callback(self):
#         msg = String()
#         msg.data = 'Hello World: %d' % self.i
#         self.publisher_.publish(msg)
#         self.get_logger().info('Publishing: "%s"' % msg.data)
#         self.i += 1


# def main(args=None):
#     rclpy.init(args=args)

#     minimal_publisher = MinimalPublisher()

#     rclpy.spin(minimal_publisher)

#     # Destroy the node explicitly
#     # (optional - otherwise it will be done automatically
#     # when the garbage collector destroys the node object)
#     minimal_publisher.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()