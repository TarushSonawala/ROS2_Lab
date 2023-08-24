#!/usr/bin/env python3

# Import necessary ROS2 packages
import rclpy
from rclpy.node import Node

# Import standard messages for strings and integers
from example_interfaces.msg import String, Int32

# Import custom message for manufacture date (even though it's not used in this code, it's imported)
from my_robot_interface.msg import ManufactureDate

class MyNode(Node):
    """
    Custom Node class to periodically publish a number as a string to a topic.
    """

    def __init__(self):
        super().__init__('number_publisher')  # Initialize with the node name 'number_publisher'

        # Create a publisher that publishes to the 'number' topic with messages of type String
        self.publisher_ = self.create_publisher(String, 'number', 10)

        # Set the timer period for the periodic callback
        timer_period = 0.5  # seconds

        # Create a timer that calls `timer_callback` method every `timer_period` seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Initialize the number to be published
        self.i = 0

    def timer_callback(self):
        """
        Callback function that gets triggered periodically.
        Publishes the current value of `self.i` and logs it.
        """
        # Create a new String message
        msg = String()

        # Populate the message with the current value of `self.i`
        msg.data = '%d' % self.i

        # Publish the message to the 'number' topic
        self.publisher_.publish(msg)

        # Log the published number for debugging purposes
        self.get_logger().info('%s' % msg.data)

        # Increment the number for the next iteration
        self.i += 1

def main(args=None):
    """
    Main function to set up ROS2 functionalities and spin the node.
    """
    # Initialize ROS2 library
    rclpy.init(args=args)

    # Create an instance of the custom Node
    node = MyNode()

    # Spin the node to keep it running and listening to callbacks
    rclpy.spin(node)

    # Cleanup resources before ending the script
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    # If the script is run directly, start the main function
    main()
