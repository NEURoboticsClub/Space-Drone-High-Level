import rclpy
from rclpy.node import Node
import numpy as np

from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3, Quaternion
from cv_bridge import CvBridge
from scipy.spatial.transform import Rotation

# Some kind of import from CV team here


# Mystery function until CV team finishes proper node construction
def mystery_function(frame):
    """
    Returns:
        processed_frame: np.ndarray
        rotation: np.ndarray shape (3,3)  (rotation matrix)
        translation: np.ndarray shape (3,1) or (3,) (translation vector)
    """
    R = np.eye(3)
    t = np.array([[0.0], [0.0], [1.0]])
    return frame, R, t


class ArucoNode(Node):
    def __init__(self):
        super().__init__("arucoNode")

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image, "/camera/image_raw", self.listener_callback, 10
        )

        self.R_cam_to_NED = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])

        self.rotation = self.create_publisher(Quaternion, "/rotation", 10)
        self.translation = self.create_publisher(Vector3, "/translation", 10)
        self.processed_image = self.create_publisher(Image, "/processed", 10)

    def listener_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        processed, rotation, translation = mystery_function(frame)

        R_NED = self.R_cam_to_NED @ rotation
        R_NED_QUART = Rotation.from_matrix(R_NED).as_quat()
        t_NED = np.squeeze(self.R_cam_to_NED @ translation)

        out_msg = self.bridge.cv2_to_imgmsg(processed, encoding="bgr8")
        self.processed_image.publish(out_msg)

        translation_vec = Vector3(
            x=float(t_NED[0]), y=float(t_NED[1]), z=float(t_NED[2])
        )
        self.translation.publish(translation_vec)

        rotation_quart = Quaternion(
            x=R_NED_QUART[0], y=R_NED_QUART[1], z=R_NED_QUART[2], w=R_NED_QUART[3]
        )
        self.rotation.publish(rotation_quart)
