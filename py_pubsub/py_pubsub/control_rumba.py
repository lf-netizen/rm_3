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
            self.position_orientation_sub_callback,
            10)

        self.subscription  # prevent unused variable warning
        self.cmd_pub = Twist()

        self.position = Pose()
        self.orientation = Pose()

        # publisher
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 1/30  # seconds

        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        self.wgain = 15.0 # Gain for the angular velocity [rad/s / rad]
        self.vconst = 5.0 # Linear velocity when far away [m/s]
        self.distThresh = 0.1 # Distance treshold [m]

    def position_orientation_sub_callback(self, msg):
        
        self.position = msg.pose.pose.position
        self.orientation = msg.pose.pose.orientation
        # self.get_logger().info('Position x={}, y={}'.format(
        #     self.position.x, self.position.y))

    def timer_callback(self):
        quat = self.orientation
        pos = self.position
        
        angles = tf_transformations.euler_from_quaternion((quat.x,quat.y,
                                                       quat.z,quat.w))
        # self.get_logger().info('-----Position angles={}'.format(angles))
        theta = angles[2]
        pose = [pos.x, pos.y, theta] 

        v = 0.0 # default linear velocity
        w = 0.0 # default angluar velocity
        distance = sqrt((pose[0]-self.goal[0])**2+(pose[1]-self.goal[1])**2)
        if (distance > self.distThresh):
            # v = self.vconst
            
            desireYaw = atan2(self.goal[1]-pose[1],self.goal[0]-pose[0])
            u = desireYaw-theta
            bound = atan2(sin(u),cos(u))
            w = min(0.5*3 , max(-0.5, self.wgain*bound))
            print(self.wgain*bound)
            v = min(0.5, distance + 0.2) * min(1 - max(w/30, -w/30), 1)
        else:
            v, w = 0.0, 0.0
        
    # Publish
        
        self.cmd_pub.angular.z = float(w)
        self.cmd_pub.linear.x = float(v)
        self.get_logger().info('Position x{pos:.2f}, y: {pos_y:.2f} V_x: {v_x:.2f}, ang_z: {ang:.2f}'.format(pos=self.position.x, pos_y=self.position.y ,
                                                                                                             v_x = self.cmd_pub.linear.x, ang=self.cmd_pub.angular.z))

        self.publisher_.publish(self.cmd_pub)
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