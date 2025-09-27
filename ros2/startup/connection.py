import rclpy
from rclpy.node import Node


class Connection(Node):
    def __init__(self):
        super().__init__("ConnectionCheck")
