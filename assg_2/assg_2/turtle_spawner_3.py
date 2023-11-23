import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn, Kill
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import math
from math import atan2
import random


class TurtleFollower(Node):

    def __init__(self):
        super().__init__('turtle_follower')

        # Attributes for keeping track of turtles
        self.first_turtle_pose = None
        self.target_turtle_pose = None

        # Subscribers and Publishers
        self.pose_subscriber1 = self.create_subscription(Pose, 'turtle1/pose', self.first_turtle_callback, 10)
        self.pose_subscriber2 = self.create_subscription(Pose, 'turtle2/pose', self.target_turtle_callback, 10)
        self.vel_publisher = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)

        # Services
        self.spawn_client = self.create_client(Spawn, 'spawn')
        self.kill_client = self.create_client(Kill, 'kill')

        # Spawn the second turtle
        self.spawn_turtle((random.uniform(0, 10)), (random.uniform(0, 10)))

    def first_turtle_callback(self, msg):
        self.first_turtle_pose = msg

        # If both turtles' poses are available, make the first turtle follow the target
        if self.first_turtle_pose and self.target_turtle_pose:
            self.follow_target()

    def target_turtle_callback(self, msg):
        self.target_turtle_pose = msg

    def spawn_turtle(self, x, y):
        for i in range(10):
            while not self.spawn_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('Waiting for spawn service...')

            request = Spawn.Request()
            request.x = x
            request.y = y
            request.name = 'turtle2'
            future = self.spawn_client.call_async(request)
            future.add_done_callback(self.after_spawn)

    def after_spawn(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Spawned turtle2 successfully.")
        except Exception as e:
            self.get_logger().warn(f"Service call failed: {e}")


    def follow_target(self):
        # Compute the difference in position
        x = self.target_turtle_pose.x - self.first_turtle_pose.x
        y = self.target_turtle_pose.y - self.first_turtle_pose.y

        # Compute distance to target turtle
        distance = (x**2 + y**2)**0.5

        # If close enough, "kill" the target turtle and spawn a new one
        if distance < 0.5:
            self.kill_turtle('turtle2')
            self.spawn_turtle((random.uniform(0, 10)), (random.uniform(0, 10)))
            self.target_turtle_pose = None
            return

        # Compute angle to target turtle
        angle_to_target = self.get_angle(x, y)

        # Compute required turn
        turn = angle_to_target - self.first_turtle_pose.theta

        # Send control commands to the first turtle
        cmd = Twist()
        cmd.linear.x = 1.5
        cmd.angular.z = 4.0 * turn

        self.vel_publisher.publish(cmd)

    def kill_turtle(self, turtle_name):
        while not self.kill_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for kill service...')

        request = Kill.Request()
        request.name = turtle_name
        future = self.kill_client.call_async(request)
        future.add_done_callback(self.after_kill)

    def after_kill(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Killed {response.name} successfully.")
            self.target_turtle_pose = None  # Reset the target's pose
            # Resubscribe to the new turtle's pose topic
            self.pose_subscriber2 = self.create_subscription(Pose, 'turtle2/pose', self.target_turtle_callback, 10)
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")


    def get_angle(self, x, y):
        return atan2(y, x)

def main():
    rclpy.init()
    node = TurtleFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
