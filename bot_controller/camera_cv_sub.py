from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
import cv2
import rclpy


class ImageSolver(Node):
    def __init__(self):
        super().__init__('image_solver')
        self.cam_subscriber = self.create_subscription(Image,
                                                        '/camera/image_color',
                                                          self.__cam_cv_aru, 10)
    
    def __cam_cv_aru(self, msg):
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        imS = cv2.resize(cv_image, (540, 540))  
        cv2.imshow("Image window", imS)
        cv2.waitKey(3)


def main(args=None):
    rclpy.init(args=args)
    img_solv = ImageSolver()
    rclpy.spin(img_solv)
    img_solv.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
