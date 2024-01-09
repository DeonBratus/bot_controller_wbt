import cv2
import cv2.aruco as aruco
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ImageSolver(Node):
    def __init__(self):
        super().__init__('image_solver')
        self.cam_subscriber = self.create_subscription(Image,
                                                        '/camera/image_color',
                                                        self.__cam_cv_aru, 10)
    
    def __cam_cv_aru(self, msg):
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)
        parameters = aruco.DetectorParameters()  # Изменено на DetectorParameters
        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        if ids is not None:
            aruco.drawDetectedMarkers(cv_image, corners, ids)  # Рисуем обнаруженные маркеры
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
