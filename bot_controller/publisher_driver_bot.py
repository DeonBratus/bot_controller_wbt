from cmath import pi
from socket import MsgFlag
import rclpy
import time

from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys


class publisher_driver(Node):
    def __init__(self):
        super().__init__('pub_driver')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.movement)

    def shape_driver(self, linx = None, qnt_rot = None):
        i = 0
        for i in range(qnt_rot):
            vel = Twist()

            vel.linear.x = linx
            vel.linear.y = 0.0
            vel.linear.z = 0.0
            vel.angular.x = 0.0
            vel.angular.y = 0.0
            vel.angular.z = 0.0
            self.publisher_.publish(vel)
            time.sleep(0.5)

            vel.linear.x = 0.0
            vel.linear.y = 0.0
            vel.linear.z = 0.0
            vel.angular.x = 0.0
            vel.angular.y = 0.0
            vel.angular.z = 2*pi/qnt_rot
            self.publisher_.publish(vel)
        
            print(f"Side {i}")
            time.sleep(1.5)


    def movement(self):
        vel = Twist()

        vel.linear.x = 0.1
        vel.linear.y = 0.0
        vel.linear.z = 0.0
        vel.angular.x = 0.0
        vel.angular.y = 0.0
        vel.angular.z = pi/2
        self.publisher_.publish(vel)
        

def main(args=None):
    rclpy.init(args=args)
    driverpub = publisher_driver()
    rclpy.spin(driverpub)
    driverpub.destroy_node()
    rclpy.shutdown()
