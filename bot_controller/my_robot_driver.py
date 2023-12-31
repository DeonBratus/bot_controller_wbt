import rclpy
from geometry_msgs.msg import Quaternion, Twist, Pose, Point
from sensor_msgs.msg import LaserScan, Range
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from sensor_msgs.msg import CameraInfo
from math import pi, cos, sin

HALF_DISTANCE_BETWEEN_WHEELS = 0.160
WHEEL_RADIUS = 0.066
qnt_tics_koef = 1
class MyRobotDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot

        self.__left_motor = self.__robot.getDevice('left wheel motor')
        self.__right_motor = self.__robot.getDevice('right wheel motor')

        self.__left_motor.setPosition(float('inf'))
        self.__left_motor.setVelocity(0)

        self.__right_motor.setPosition(float('inf'))
        self.__right_motor.setVelocity(0)

        self.__left_encoder = self.__robot.getDevice('left wheel sensor')
        self.__left_encoder.enable(32)

        self.__right_encoder = self.__robot.getDevice('right wheel sensor')
        self.__right_encoder.enable(32)

        self.__cam = self.__robot.getDevice('camera')
        self.__cam.enable(32)

        self.__lidar = self.__robot.getDevice('LDS-01')
        self.__lidar.enable(32)
        self.__lidar.enablePointCloud()

        self.__target_twist = Twist()
        self.__target_pose = Pose()
        rclpy.init(args=None)
        self.__node = rclpy.create_node('my_robot_driver')
        self.__node.create_subscription(Twist, '/cmd_vel', self.__cmd_vel_callback, 1)
        # Odom and joint_state, doesn't work
        self.odom_pub = self.__node.create_publisher(Odometry, '/odom', 10)
        self.joint_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.__publish_odometry,
            10
        )
    def __cmd_vel_callback(self, twist):
        self.__target_twist = twist
    # odom doesn't work now
    def __publish_odometry(self):

        current_time = self.get_clock().now()
        # calculating odometry, formuls can be incorrect
        left_dist = self.left_value_wheel * WHEEL_RADIUS * 2 * pi * qnt_tics_koef
        right_dist = self.right_value_wheel * WHEEL_RADIUS * 2 * pi * qnt_tics_koef
        self.robot_center_position  =  (right_dist - left_dist) / HALF_DISTANCE_BETWEEN_WHEELS / 2
        x =  ( (left_dist + right_dist)/2) * cos(self.robot_center_position)
        y = ( (left_dist + right_dist)/2) * sin(self.robot_center_position)

        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        self.__target_pose.position.x += x
        self.__target_pose.position.y += y
        self.__target_pose.position.z += 0.0
        self.__target_pose.orientation.x = 0.0   
        self.__target_pose.orientation.y = 0.0
        self.__target_pose.orientation.z += 0.0
        self.__target_pose.orientation.w = 0.0

        odom_msg.pose.pose = self.__target_pose
        odom_msg.twist.twist = self.__target_twist
        self.odom_pub.publish(odom_msg)


    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)
        forward_speed = self.__target_twist.linear.x

        self.right_value_wheel = self.__right_encoder.getValue()
        self.left_value_wheel = self.__left_encoder.getValue()

        angular_speed = self.__target_twist.angular.z

        command_motor_left = (forward_speed + 0.5 * angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS
        command_motor_right = (forward_speed + angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS

        if abs(angular_speed) > 0.2: 
            self.__left_motor.setVelocity(-command_motor_left)
            self.__right_motor.setVelocity(command_motor_right)
        else:
            self.__left_motor.setVelocity(command_motor_left)
            self.__right_motor.setVelocity(command_motor_right)
    def main(args=None):
        print('hello')