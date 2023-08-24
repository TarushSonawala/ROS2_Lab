# Import necessary modules and classes
import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn, Kill
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from math import atan2
import random


class TurtleFollower(Node):
    """
    A ROS node class that makes one turtle follow another turtle.
    If the following turtle gets close enough to the target, the target turtle is "killed" and a new one is spawned.
    """

    def __init__(self):
        # Initialize the node with the name 'turtle_follower'
        super().__init__('turtle_follower')

        # Initialize attributes to keep track of the poses of the two turtles
        self.first_turtle_pose = None
        self.target_turtle_pose = None

        # Set up subscribers to listen to the pose of each turtle and a publisher to control the first turtle's movement
        self.pose_subscriber1 = self.create_subscription(Pose, 'turtle1/pose', self.first_turtle_callback, 10)
        self.pose_subscriber2 = self.create_subscription(Pose, 'turtle2/pose', self.target_turtle_callback, 10)
        self.vel_publisher = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)

        # Set up clients for the services to spawn and kill turtles
        self.spawn_client = self.create_client(Spawn, 'spawn')
        self.kill_client = self.create_client(Kill, 'kill')

        # Spawn the second turtle at a random position
        self.spawn_turtle((random.uniform(0, 10)), (random.uniform(0, 10)))

    def first_turtle_callback(self, msg):
        """Callback for updates on the first turtle's pose."""
        self.first_turtle_pose = msg

        # If we know the position of both turtles, command the first turtle to follow the second
        if self.first_turtle_pose and self.target_turtle_pose:
            self.follow_target()

    def target_turtle_callback(self, msg):
        """Callback for updates on the target turtle's pose."""
        self.target_turtle_pose = msg

    def spawn_turtle(self, x, y):
        """Attempt to spawn the target turtle at the given coordinates multiple times until successful."""
        for i in range(10):
            # Wait for the spawn service to become available
            while not self.spawn_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('Waiting for spawn service...')

            # Make a request to spawn the turtle
            request = Spawn.Request()
            request.x = x
            request.y = y
            request.name = 'turtle2'
            future = self.spawn_client.call_async(request)
            future.add_done_callback(self.after_spawn)

    def after_spawn(self, future):
        """Callback after an attempt to spawn a turtle."""
        try:
            response = future.result()
            self.get_logger().info(f"Spawned turtle2 successfully.")
        except Exception as e:
            self.get_logger().warn(f"Service call failed: {e}")

    def follow_target(self):
        """Command the first turtle to follow the target turtle."""
        # Calculate the relative position of the target turtle
        x = self.target_turtle_pose.x - self.first_turtle_pose.x
        y = self.target_turtle_pose.y - self.first_turtle_pose.y

        # Compute the distance to the target turtle
        distance = (x**2 + y**2)**0.5

        # If the first turtle is close enough to the target, "kill" the target and spawn a new one
        if distance < 0.5:
            self.kill_turtle('turtle2')
            self.spawn_turtle((random.uniform(0, 10)), (random.uniform(0, 10)))
            self.target_turtle_pose = None
            return

        # Calculate the angle to the target turtle
        angle_to_target = self.get_angle(x, y)

        # Calculate the turning angle needed
        turn = angle_to_target - self.first_turtle_pose.theta

        # Create a control command to move the first turtle towards the target
        cmd = Twist()
        cmd.linear.x = 1.5
        cmd.angular.z = 4.0 * turn

        # Publish the command to move the first turtle
        self.vel_publisher.publish(cmd)

    def kill_turtle(self, turtle_name):
        """Attempt to "kill" the specified turtle."""
        while not self.kill_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for kill service...')

        # Make a request to kill the turtle
        request = Kill.Request()
        request.name = turtle_name
        future = self.kill_client.call_async(request)
        future.add_done_callback(self.after_kill)

    def after_kill(self, future):
        """Callback after an attempt to kill a turtle."""
        try:
            response = future.result()
            self.get_logger().info(f"Killed {response.name} successfully.")
            self.target_turtle_pose = None  # Reset target turtle's pose
            # Reinitialize the subscription to get updates on the new target turtle's pose
            self.pose_subscriber2 = self.create_subscription(Pose, 'turtle2/pose', self.target_turtle_callback, 10)
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    @staticmethod
    def get_angle(x, y):
        """Calculate and return the angle to a point in the XY plane."""
        return atan2(y, x)

def main():
    """Main function to execute the ROS node."""
    rclpy.init()               # Initialize ROS
    node = TurtleFollower()    # Create the TurtleFollower node
    rclpy.spin(node)           # Keep the node running
    node.destroy_node()        # Clean up the node
    rclpy.shutdown()           # Shutdown ROS

if __name__ == '__main__':
    main()
