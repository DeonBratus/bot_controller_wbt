from rclpy.node import Node
from nav_msgs.msg import Odometry
import rclpy

class GetOdom(Node):
    def __init__(self):
        super().__init__('odom_getter')
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.__getting_odom, 10)

    def __getting_odom(self, msg):
        print(msg)


def main(args=None):
    rclpy.init(args=args)
    odom_getter = GetOdom()
    rclpy.spin(odom_getter)
    odom_getter.destroy_node()
    rclpy.shutdown()