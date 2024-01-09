import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Range


class getLaser(Node):
    
    def __init__(self):
        super().__init__('get_laser')
        self.laser_sub = self.create_subscription(LaserScan, '/scan', self.get_laser, 10)

    def get_laser(self, msg):
        las_msg = msg
        print(las_msg)        


def main(args=None):
    rclpy.init(args=args)
    las_info = getLaser()
    rclpy.spin(las_info)
    las_info.destroy_node()
    rclpy.shutdown()