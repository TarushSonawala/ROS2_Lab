#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import transforms3d
import math
from sensor_msgs.msg import LaserScan
     
     
class GotoGoalNode(Node):
    def __init__(self):
        super().__init__("move_robot")
        self.target_x = 2
        self.target_y = 2
        self.publisher = self.create_publisher(Twist, "cmd_vel", 10)
        # self.subscriber = self.create_subscription(Odometry, "odom", self.control_loop, 10)
        self.subscriber1=self.create_subscription(LaserScan,"gazebo_lidar/out",self.value_loop,12)
               
    def value_loop(self, msg):
        
        # dist_x = self.target_x - msg.pose.pose.position.x
        # dist_y = self.target_y - msg.pose.pose.position.y
        # print('current position: {} {}'.format(msg.pose.pose.position.x,msg.pose.pose.position.y))
        # distance = math.sqrt(dist_x * dist_x + dist_y * dist_y)
        # print('distance : {}'.format(round(distance, 3)))
        
        
        # goal_theta = math.atan2(dist_y, dist_x)
        # quat = msg.pose.pose.orientation
        # roll, pitch, yaw = transforms3d.euler.quat2euler([quat.w,quat.x,quat.y,quat.z])
        # diff = math.pi - round(yaw, 2) + round(goal_theta, 2)
        # print('yaw: {}'.format(round(yaw, 2)))
        # print('target angle: {}'.format(round(goal_theta, 2)))

        # if diff > math.pi:
        #     diff -= 2*math.pi
        # elif diff < -math.pi:
        #     diff += 2*math.pi
        # print('orientation : {}'.format(round(diff, 2)))   
        
        # vel = LaserScan()
        
        # if(vel.ranges)
        # if()

        # if abs(diff) > 0.2:
        #     vel.linear.x = 0.0
        #     vel.angular.z = 0.4*round(diff, 2)  
          
        # else:
        #     if abs(distance) > 0.2:
        #         vel.linear.x = 0.3*round(distance, 3)
        #         vel.angular.z = 0.0  
        vel=Twist()
        #     else:
        #         vel.linear.x = 0.0
        #         vel.angular.z = 0.0
        # print(vel.ranges[218])
        for i in range(158,221):
            print(msg.ranges[i])
        
        if (str(msg.ranges[158]) == 'inf'):
            vel.linear.x = 0.9
            # vel.angular.z = 0
             
            #  if (str(msg.ranges[i]) != 'inf'):
            #       print(i)
        # print (msg.ranges[158,221])
        self.publisher.publish(vel)
        
     
def main(args=None):
    rclpy.init(args=args)
    node = GotoGoalNode()
    rclpy.spin(node)
    rclpy.shutdown()
    
       
if __name__ == "__main__":
	main()