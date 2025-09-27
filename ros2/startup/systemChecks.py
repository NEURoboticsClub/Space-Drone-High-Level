import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from sensor_msgs.msg import BatteryState, NavSatFix, Imu
from mavros_msgs.msg import State


# Need to figure out mavros_msgs.msg import problems
class SystemCheck(Node):
    def __init__(self):
        super().__init__("SystemCheck")

        self.health_pub = self.create_publisher(Bool, "/health/ok", 10)

        self.create_subscription(State, "/mavros/state", self.state_cb, 10)
        self.create_subscription(BatteryState, "/mavros/battery", self.battery_cb, 10)
        self.create_subscription(
            NavSatFix, "/mavros/global_position/global", self.gps_cb, 10
        )
        self.create_subscription(Imu, "/mavros/imu/data", self.imu_cb, 10)

        self.connected = False
        self.battery_ok = False
        self.has_gps = False
        self.imu_ok = False

        self.create_timer(1.0, self.evaluate_health)

    def state_cb(self, msg):
        self.connected = msg.connected

    def battery_cb(self, msg: BatteryState):
        self.battery_ok = msg.percentage > 0.25

    def gps_cb(self, msg: NavSatFix):
        self.has_gps = msg.status.status > 0

    def imu_cb(self, msg: Imu):
        self.imu_ok = (
            abs(msg.linear_acceleration.x)
            + abs(msg.linear_acceleration.y)
            + abs(msg.linear_acceleration.z)
            > 0.1
        )

    def evaluate_health(self):
        all_ok = self.battery_ok and self.has_gps and self.imu_ok and self.connected
        msg = Bool()
        msg.data = all_ok
        self.health_pub.publish(msg)

        if all_ok:
            self.get_logger().info("✅ System checks passed. Ready to arm.")
        else:
            self.get_logger().warn(
                "⚠️ System not ready: "
                f"PX4={self.connected}, "
                f"Battery={self.battery_ok}, "
                f"GPS={self.has_gps}, "
                f"IMU={self.imu_ok}"
            )
