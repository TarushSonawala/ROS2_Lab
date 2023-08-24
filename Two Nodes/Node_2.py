#!/usr/bin/env python3

# Importing the necessary ROS2 packages
import rclpy
from rclpy.node import Node

# Importing standard messages for strings and integers
from example_interfaces.msg import String, Int32

# Importing a custom message for manufacture date 
# (even though it's not used in this code, it's imported)
from my_robot_interface.msg import ManufactureDate


class MySubNode(Node):
    """
    Custom ROS2 Node class which subscribes to a 'number' topic and maintains
    a count of the number of messages it has received on that topic.
    Each time it receives a message, it also publishes the current count to the 'number_count' topic.
    """

    def __init__(self):
        super().__init__('number_counter')  # Initializing with the node name 'number_counter'

        # Create a subscription to the 'number' topic with messages of type String
        self.subscription = self.create_subscription(String, 'number', self.listener_callback, 10)

        # Create a publisher that publishes to the 'number_count' topic with messages of type String
        self.publisher_ = self.create_publisher(String, 'number_count', 10)

        # Initialize the counter
        self.i = 0

    def listener_callback(self, msg):
        """
        Callback function executed when a message is received on the 'number' topic.
        """
        # Log the received message for debugging purposes
        self.get_logger().info('I heard: "%s"' % msg.data)
        
        # Increment the counter each time a message is received
        self.i += 1
        
        # Prepare a message to publish the current count
        msg_pub = String()
        msg_pub.data = '%d' % self.i
        
        # Publish the counter value to the 'number_count' topic
        self.publisher_.publish(msg_pub)

        # Log the counter for debugging purposes
        self.get_logger().info('Counter =  %s' % msg_pub.data)


def main(args=None):
    """
    Main function to set up ROS2 functionalities and spin the node.
    """
    # Initialize the ROS2 library
    rclpy.init(args=args)

    # Create an instance of the custom Node
    node = MySubNode()

    # Keep the node running and listening to callbacks
    rclpy.spin(node)

    # Cleanup resources before ending the script
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    # If the script is run directly, start the main function
    main()
