import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool


class Arm(Node):
    def __init__(self):
        super().__init__("ArmNode")

        self.create_subscription(Bool, "/SystemCheck/health/ok", 10)

    def arm_drone(self):
        pass
