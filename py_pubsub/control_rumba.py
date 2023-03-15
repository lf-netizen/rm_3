import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from nav_msgs.msg import Odometry
import time
from geometry_msgs.msg import Twist, Vector3
import numpy as np
import tf_transformations
from geometry_msgs.msg import Point, Quaternion, Pose
from nav_msgs.msg import Odometry
from math import *


class Drone_control(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')

        self.goal = [0, -5]

        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.listener_callback,
            10)

        self.subscription  # prevent unused variable warning
        self.line = Twist()
        self.twist = Twist()
        self.position = Pose()
        self.orientation = Pose()

        # publisher
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        self.wgain = 15.0 # Gain for the angular velocity [rad/s / rad]
        self.vconst = 5.0 # Linear velocity when far away [m/s]
        self.distThresh = 0.1 # Distance treshold [m]

    def listener_callback(self, msg):
        
        self.position = msg.pose.pose.position
        self.orientation = msg.pose.pose.orientation


    def timer_callback(self):
        quat = self.orientation
        pos = self.position
        
        angles = tf_transformations.euler_from_quaternion((quat.x,quat.y,
                                                       quat.z,quat.w))

        theta = angles[2]
        pose = [pos.position.x, pos.position.y, theta] 

        v = 0 # default linear velocity
        w = 0 # default angluar velocity
        distance = sqrt((pose[0]-self.goal[0])**2+(pose[1]-self.goal[1])**2)
        if (distance > self.distThresh):
            v = self.vconst
            desireYaw = atan2(self.goal[1]-pose[1],self.goal[0]-pose[0])
            u = desireYaw-theta
            bound = atan2(sin(u),cos(u))
            w = min(0.5 , max(-0.5, self.wgain*bound))
        
    # Publish
        self.line.linear.x = v
        self.twist.angular.z = w


        self.publisher_.publish(self.line)
        self.publisher_.publish(self.twist)
        self.i += 1





def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = Drone_control()


    rclpy.spin(minimal_subscriber)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()