import rclpy
from rclpy.node import Node

import asyncio
from mavsdk import System

from std_msgs.msg import Float32
from AprilTagDetection.msg import geometry_msgs/PoseWithCovarianceStamped

class MinimalSubscriber(Node):
def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            geometry_msgs/PoseWithCovarianceStamped,
            'pose',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

async def math(self, msg):
    self.x = msg.geometry_msgs.PoseWithVarianceStamped.PoseWithVariance.Pose.Point.x
    self.y = msg.geometry_msgs.PoseWithVarianceStamped.PoseWithVariance.Pose.Point.y
    self.z = msg.geometry_msgs.PoseWithVarianceStamped.PoseWithVariance.Pose.Point.z