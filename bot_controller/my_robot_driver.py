import rclpy
from geometry_msgs.msg import Quaternion, Twist, Pose, Point
from nav_msgs.msg import Odometry
from math import pi, cos, sin
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
import math
import numpy as np
from tf2_ros import StaticTransformBroadcaster
from sensor_msgs.msg import LaserScan, Range
from geometry_msgs.msg import TransformStamped

HALF_DISTANCE_BETWEEN_WHEELS = 0.160
WHEEL_RADIUS = 0.066
SENSOR_DIST_FROM_CENTER = 0.035
INFRARED_MAX_RANGE = 0.04
INFRARED_MIN_RANGE = 0.009
TOF_MAX_RANGE = 1.0
NB_INFRARED_SENSORS = 8
SENSOR_DIST_FROM_CENTER = 0.035
OUT_OF_RANGE = 0.0

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

        self.__imu = self.__robot.getDevice('inertial_unit')
        self.__imu.enable(32)

        self.__lidar = self.__robot.getDevice('LDS-01')
        self.__lidar.enable(32)
        self.__lidar.enablePointCloud()
        self.prev_right_value_wheel = 0
        self.prev_left_value_wheel = 0
        self.x = 0
        self.y = 0
        self.theta = 0
        self.__target_twist = Twist()

        rclpy.init(args=None)
        self.__node = rclpy.create_node('my_robot_driver')

        qos_profile = QoSProfile(durability=QoSDurabilityPolicy.VOLATILE, 
                                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                                history=QoSHistoryPolicy.KEEP_LAST,
                                depth=100)
        self.__node.create_subscription(Twist, '/cmd_vel', self.__cmd_vel_callback, 1)
        self.laser_publisher = self.__node.create_publisher(LaserScan, '/scan', 1)
        self.pub_odom = self.__node.create_publisher(Odometry, '/odom',  1)
        self.__subscriber_tof = self.__node.create_subscription(Odometry, '/odom', self.__publish_laserscan_data, 1)


        self.__now = self.__node.get_clock().now().to_msg()

        laser_transform = TransformStamped()
        laser_transform.header.stamp = self.__now
        laser_transform.header.frame_id = 'base_link'
        laser_transform.child_frame_id = 'laser_scanner'
        laser_transform.transform.rotation.x = 0.0
        laser_transform.transform.rotation.y = 0.0
        laser_transform.transform.rotation.z = 0.0
        laser_transform.transform.rotation.w = 1.0
        laser_transform.transform.translation.x = 0.0
        laser_transform.transform.translation.y = 0.0
        laser_transform.transform.translation.z = 0.033

        self.static_broadcaster = StaticTransformBroadcaster(self.__node)
        self.static_broadcaster.sendTransform(laser_transform)



       
    def __publish_laserscan_data(self, msg_odom):
        # Max range of ToF sensor is 2m so we put it as maximum laser range.
        # Therefore, for all invalid ranges we put 0 so it get deleted by rviz
        las_msg = LaserScan()
        las_msg.header.frame_id = 'laser_scanner'
        las_msg.header.stamp = msg_odom.header.stamp
        las_msg.angle_min = - 150 * pi / 180
        las_msg.angle_max = 150 * pi / 180
        las_msg.angle_increment = 15 * pi / 180
        las_msg.range_min = SENSOR_DIST_FROM_CENTER + INFRARED_MIN_RANGE
        las_msg.range_max = SENSOR_DIST_FROM_CENTER + TOF_MAX_RANGE
        las_msg.ranges =  self.distances
        self.laser_publisher.publish(las_msg)

    def __cmd_vel_callback(self, twist):
        self.__target_twist = twist
    
    def quaternion_from_euler(self, ai, aj, ak):
        ai /= 2.0
        aj /= 2.0
        ak /= 2.0
        ci = math.cos(ai)
        si = math.sin(ai)
        cj = math.cos(aj)
        sj = math.sin(aj)
        ck = math.cos(ak)
        sk = math.sin(ak)
        cc = ci*ck
        cs = ci*sk
        sc = si*ck
        ss = si*sk

        q = np.empty((4, ))
        q[0] = cj*sc - sj*cs
        q[1] = cj*ss + sj*cc
        q[2] = cj*cs - sj*sc
        q[3] = cj*cc + sj*ss

        return q

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)
        

        self.distances = self.__lidar.getRangeImage()
        self.roll, self.pitch, self.yaw = self.__imu.getRollPitchYaw()
        self.right_value_wheel = self.__right_encoder.getValue()
        self.left_value_wheel = self.__left_encoder.getValue()
        distance_travelled_right = (self.right_value_wheel - self.prev_right_value_wheel) * WHEEL_RADIUS
        distance_travelled_left = (self.left_value_wheel - self.prev_left_value_wheel) * WHEEL_RADIUS

        delta_distance = 0.5 * (distance_travelled_right + distance_travelled_left)
        delta_theta = (distance_travelled_right - distance_travelled_left) / HALF_DISTANCE_BETWEEN_WHEELS

        self.x += delta_distance * cos(self.theta + 0.5 * delta_theta)
        self.y += delta_distance * sin(self.theta + 0.5 * delta_theta)
        self.theta += delta_theta

        # Update previous wheel encoder values
        self.prev_right_value_wheel = self.right_value_wheel
        self.prev_left_value_wheel = self.left_value_wheel

        msg = Odometry()
        msg.header.stamp = self.__node.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        # Assign values to pose
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = 0.0  # Assuming your robot moves in the xy-plane

        msg.pose.pose.orientation.x = self.quaternion_from_euler(0, 0, self.theta)[0]  # You need to import quaternion_from_euler from a suitable library
        msg.pose.pose.orientation.y = self.quaternion_from_euler(0, 0, self.theta)[1]
        msg.pose.pose.orientation.z = self.quaternion_from_euler(0, 0, self.theta)[2]
        msg.pose.pose.orientation.w = self.quaternion_from_euler(0, 0, self.theta)[3]


        msg.twist.twist.linear.x = self.__target_twist.linear.x
        msg.twist.twist.linear.y = self.__target_twist.linear.y
        msg.twist.twist.linear.z = self.__target_twist.linear.z

        msg.twist.twist.angular.x = self.__target_twist.angular.x
        msg.twist.twist.angular.y = self.__target_twist.angular.y
        msg.twist.twist.angular.z = self.__target_twist.angular.z
        
        #print('msg sent')
        self.pub_odom.publish(msg)
        forward_speed = self.__target_twist.linear.x
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
    rclpy.init(args=args)
    my_driver = MyRobotDriver()
    rclpy.spin(my_driver)
    my_driver.destroy_node()
    rclpy.shutdown()