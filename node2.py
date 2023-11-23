#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import String
from example_interfaces.msg import String, Int32
from my_robot_interface.msg import ManufactureDate


class MySubNode(Node):

    def __init__(self):
        super().__init__('number_counter')
        self.subscription = self.create_subscription(String,'number',self.listener_callback,10)        
        self.subscription
        self.publisher_=self.create_publisher(String,'number_count',10)
        self.i = 0


    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        self.i+=1
        msg_pub=String()
        msg_pub.data= '%d' % self.i
        self.publisher_.publish(msg_pub)
        self.get_logger().info('Counter =  %s' % msg_pub.data)



def main(args=None):
    rclpy.init(args=args)

    node = MySubNode()

    rclpy.spin(node)


    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()