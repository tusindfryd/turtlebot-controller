import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import cv2
from cv_bridge import CvBridge
import numpy as np

# ARUCO_DICT = {
#     "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
#     "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
#     "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
#     "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
#     "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
#     "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
#     "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
#     "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
#     "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
#     "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
#     "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
#     "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
#     "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
#     "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
#     "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
#     "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
#     "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
#     "DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
#     "DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
#     "DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
#     "DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11,
# }


class ExamineImage(Node):
    def __init__(self):
        super().__init__("examine_image")
        self.window_name = "camera"
        self.subscription = self.create_subscription(
            Image, "image_raw", self.image_callback, 100
        )
        self.publisher = self.create_publisher(Twist, "cmd_vel", 100)
        self.subscription
        self.point = None
        self.image = None
        self.image_size = None
        self.dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_100)
        self.parameters = cv2.aruco.DetectorParameters_create()

    def image_callback(self, image_data):
        self.image = CvBridge().imgmsg_to_cv2(image_data, "bgr8")
        self.image_size = (image_data.height, image_data.width)

        if self.point is not None:
            cv2.rectangle(
                self.image,
                (self.point[0] - 100, self.point[1] - 100),
                (self.point[0] + 100, self.point[1] + 100),
                (0, 255, 0),
                3,
            )

        cv2.setMouseCallback(self.window_name, self.handle_pointer)

        markerCorners, markerIds, rejectedCandidates = cv2.aruco.detectMarkers(
            self.image, self.dictionary, parameters=self.parameters
        )

        self.image = self.handle_aruco(
            markerCorners, markerIds, rejectedCandidates, self.image
        )

        if self.image is not None:
            cv2.imshow(self.window_name, self.image)
            cv2.waitKey(10)

    def handle_pointer(self, event, x, y, flags, param):
        self.point = (x, y)
        if event == cv2.EVENT_LBUTTONDOWN:
            msg = Twist()
            if y > self.image_size[0] / 2:
                msg.linear.x = -0.5
            else:
                msg.linear.x = 0.5

            self.publisher.publish(msg)

    def handle_aruco(self, corners, ids, rejected, image):
        if corners:
            ids = ids.flatten()

            for (markerCorner, markerID) in zip(corners, ids):
                corners = np.array(markerCorner, np.int32)
                (topLeft, topRight, bottomRight, bottomLeft) = corners.reshape((4, 2))
                cv2.polylines(image, [corners], True, (0, 0, 255), 2)

                cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)

                msg = Twist()
                if cY > self.image_size[0] / 2:
                    msg.linear.x = -0.5
                else:
                    msg.linear.x = 0.5

                self.publisher.publish(msg)

        return image


def main(args=None):
    rclpy.init(args=args)
    examine_image = ExamineImage()
    rclpy.spin(examine_image)
    examine_image.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
