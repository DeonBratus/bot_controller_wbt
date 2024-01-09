from rclpy.node import Node
from nav_msgs.msg import Odometry
import rclpy
import time
class GetOdom(Node):
    def __init__(self):
        super().__init__('odom_getter')
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odometry_callback, 1)
    
    def odometry_callback(self, msg: Odometry):
        time.sleep(0.4)
        print(msg.pose.pose.position.x, msg.pose.pose.position.y)



def main(args=None):
    rclpy.init(args=args)
    odom_getter = GetOdom()
    rclpy.spin(odom_getter)
    odom_getter.destroy_node()
    rclpy.shutdown()