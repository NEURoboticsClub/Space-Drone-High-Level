#https://github.com/mavlink/MAVSDK-Python/blob/main/examples/offboard_position_ned.py

#!/usr/bin/env python3

"""
Caveat when attempting to run the examples in non-gps environments:

`drone.offboard.stop()` will return a `COMMAND_DENIED` result because it
requires a mode switch to HOLD, something that is currently not supported in a
non-gps environment.
"""

import asyncio
from std_msgs.msg import Float32MultiArray
import rclpy
from rclpy.node import Node

from mavsdk import System
from mavsdk.offboard import (OffboardError, PositionNedYaw)

async def run():
    """ Does Offboard control using position NED coordinates. """

    drone = System()

    #UDP or TCP address
    system_address = "udp://:14540"
    await drone.connect(system_address)

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"-- Connected to drone!")
            break

    print("Waiting for drone to have a global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("-- Global position estimate OK")
            break

    print("-- Arming")
    await drone.action.arm()

    print("-- Setting initial setpoint")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))

    print("-- Starting offboard")
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"Starting offboard mode failed \
                with error code: {error._result.result}")
        print("-- Disarming")
        await drone.action.disarm()
        return

    print("-- Go 0m North, 0m East, -5m Down \
            within local coordinate system")
    await drone.offboard.set_position_ned(
            PositionNedYaw(0.0, 0.0, -5.0, 0.0))
    await asyncio.sleep(10)

    print("-- Go 5m North, 0m East, -5m Down \
            within local coordinate system, turn to face East")
    await drone.offboard.set_position_ned(
            PositionNedYaw(5.0, 0.0, -5.0, 90.0))
    await asyncio.sleep(10)

    print("-- Go 5m North, 10m East, -5m Down \
            within local coordinate system")
    await drone.offboard.set_position_ned(
            PositionNedYaw(5.0, 10.0, -5.0, 90.0))
    await asyncio.sleep(15)

    print("-- Go 0m North, 10m East, 0m Down \
            within local coordinate system, turn to face South")
    await drone.offboard.set_position_ned(
            PositionNedYaw(0.0, 10.0, 0.0, 180.0))
    await asyncio.sleep(10)

    print("-- Stopping offboard")
    try:
        await drone.offboard.stop()
    except OffboardError as error:
        print(f"Stopping offboard mode failed \
                with error code: {error._result.result}")

class FloatArraySubscriber(Node):
    def __init__(self):
        super().__init__('float_array_subscriber')
        self.subscription = self.create_subscription(
            Float32MultiArray, 
            'tag_poses',  
            self.listener_callback,
            10)
        self.subscription  

    def listener_callback(self, msg):
        float_array = msg.data  # Extracting the float array
        self.get_logger().info(f'Received data: {float_array}')

        # Access individual elements if needed
        id_value = float_array[0]
        x_value = float_array[1]
        y_value = float_array[2]
        z_value = float_array[3]
        rotation_value = float_array[4]

        self.get_logger().info(f'ID: {id_value}, X: {x_value}, Y: {y_value}, Z: {z_value}, Rotation: {rotation_value}')

def main(args=None):
    rclpy.init(args=args)
    subscriber = FloatArraySubscriber()
    rclpy.spin(subscriber)

    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    # Run the asyncio loop
    asyncio.run(run())