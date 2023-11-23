#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import String
from example_interfaces.msg import String, Int32
from my_robot_interface.msg import ManufactureDate

class RobotDateSubscriber(Node):
    def __init__(self):
            super().__init__("robot_date_subscriber")
            self.subscriber_ = self.create_subscription(ManufactureDate,
            "robot_manufacturing_date", self.callback_robot_news, 10)
            self.get_logger().info("robot_subscriber Node Started")

    def callback_robot_news(self, msg):
        information ="Manufacturing Date of the ROBOT is " + str(msg.date) + " " + str(msg.month) + " " + str(msg.year)
        self.get_logger().info(information)

    def send_number(self):
        number = Int32()
        number.data = self.count_
        self.count_ +=1
        self.publisher_.publish(number)
def main(args=None):
    rclpy.init(args=args)
    node = RobotDateSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()
if __name__ == "__main__":
    main()