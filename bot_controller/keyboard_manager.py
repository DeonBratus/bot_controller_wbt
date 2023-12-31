from cmath import pi
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
import curses
from sensor_msgs.msg import LaserScan, Range

class PublisherDriver(Node):
    def __init__(self):
        super().__init__('pub_driver')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        timer_period = 0.1  # seconds (adjust as needed)
    def keybrd_manage(self, stdscr):
        vel = Twist()
        key = stdscr.getch()
        
        if key == ord('w'):
            vel.linear.x = 0.5
            vel.angular.z = 0.0
        elif key == ord('s'):
            vel.linear.x = -0.5
            vel.angular.z = 0.0
        elif key == ord('a'):
            vel.linear.x = 0.0
            vel.angular.z = pi/2
        elif key == ord('d'):
            vel.linear.x = 0.0
            vel.angular.z = -pi/2
        else:
            vel.linear.x = 0.0
            vel.angular.z = 0.0
        self.publisher_.publish(vel)

    def movement(self):
        curses.wrapper(self.keybrd_manage)
    
def main(args=None):
    rclpy.init(args=args)
    driver_pub = PublisherDriver()
    rclpy.spin(driver_pub)
    driver_pub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
