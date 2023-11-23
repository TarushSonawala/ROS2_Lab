import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn, Kill
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import random
import math

class TurtleFollower(Node):

    def __init__(self):
        super().__init__('turtle_follower')
        
        # Initialize attributes
        self.pose_ = None
        self.target_x = None
        self.target_y = None
        self.spawned_turtle_name = "turtle2"
        
        # ROS2 service clients, publishers, and subscribers
        self.spawn_client = self.create_client(Spawn, 'spawn')
        self.kill_client = self.create_client(Kill, 'kill')
        self.pose_subscriber_ = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.cmd_vel_publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # Start control loop with timer
        self.timer = self.create_timer(0.1, self.control_loop)

    def spawn_random_turtle(self):
        if not self.spawn_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("Spawn service is not available!")
            return

        x, y = random.uniform(0, 11), random.uniform(0, 11)
        self.target_x = x
        self.target_y = y
        request = Spawn.Request()
        request.x = x
        request.y = y
        request.theta = 0.0
        request.name = self.spawned_turtle_name
        self.spawn_client.call_async(request)

    def kill_spawned_turtle(self):
        if not self.kill_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("Kill service is not available!")
            return

        request = Kill.Request()
        request.name = self.spawned_turtle_name
        self.kill_client.call_async(request)

    def pose_callback(self, msg: Pose):
        self.pose_ = msg

    def control_loop(self):
        if self.pose_ == None:
            return
        dist_x = self.target_x - self.pose_.x
        dist_y = self.target_y - self.pose_.y
        distance = math.sqrt(dist_x * dist_x + dist_y * dist_y)
        msg = Twist()
        if distance > 0.5:
            msg.linear.x = distance
            goal_theta = math.atan2(dist_y, dist_x)
            diff = goal_theta - self.pose_.theta
            if diff > math.pi:
                diff -= 2*math.pi
            elif diff < -math.pi:
                diff += 2*math.pi
            msg.angular.z = diff
        else:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.kill_spawned_turtle()
            self.spawn_random_turtle()
        self.cmd_vel_publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TurtleFollower()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
