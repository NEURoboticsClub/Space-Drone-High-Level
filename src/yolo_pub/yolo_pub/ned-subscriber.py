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
 
'''
async def run():
 
 
    # KEYBOARD INtERRUPT SHUTDOWN THINGY: 
 
    node = FloatArraySubscriber(drone)
 
    stop_event = asyncio.Event()
 
    def shutdown_handler(sig, frame):
        """ Handles keyboard interrupt to stop offboard mode and disarm safely. """
        print("\n-- KeyboardInterrupt detected! Stopping offboard mode and disarming...")
        stop_event.set()
 
    signal.signal(signal.SIGINT, shutdown_handler)
 
    ros_task = asyncio.create_task(run_ros_node(node, stop_event))
 
    try:
        await stop_event.wait() 
    finally:
        print("-- Stopping offboard mode")
        try:
            await drone.offboard.stop()
        except Exception as error:
            print(f"Stopping offboard mode failed: {error}")
 
        print("-- Disarming drone")
        await drone.action.disarm()
 
        node.destroy_node()
        rclpy.shutdown()
        print("-- Drone and ROS 2 shutdown complete.")
'''
 
class FloatArraySubscriber(Node):
	# DRONE MVOEMENT ASYNC FUNCTION:
	def move_drone(self, x, y, z, yaw):
		try:
			self.drone.offboard.set_position_ned(PositionNedYaw(x, y, z, yaw))
			self.get_logger().info(f"Drone moving to X:{x}, Y:{y}, Z:{z}, Yaw:{yaw}")
		except Exception as e:
			self.get_logger().error(f"Failed to send PositionNedYaw: {str(e)}")
 
 
	def __init__(self):
		super().__init__('ned_subscriber')
		self.subscription = self.create_subscription(
			Float32MultiArray, 
			'tag_poses',  
			self.listener_callback,
			10)
		self.subscription  
		self.drone = System()
 
	async def initialize(self):
		""" Does Offboard control using position NED coordinates. """
		#UDP or TCP address
		system_address = "udp://:14540"
		await self.drone.connect(system_address)
		print("Waiting for drone to connect...")
		async for state in self.drone.core.connection_state():
			if state.is_connected:
				print(f"-- Connected to drone!")
			break
		print("Waiting for drone to have a global position estimate...")
		async for health in self.drone.telemetry.health():
			if health.is_global_position_ok and health.is_home_position_ok:
				print("-- Global position estimate OK")
			break
 
		print("-- Arming")
		await self.drone.action.arm()
 
		print("-- Setting initial setpoint")
		await self.drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))
 
		print("-- Starting offboard")
		try:
			await self.drone.offboard.start()
		except OffboardError as error:
			print(f"Starting offboard mode failed with error code: {error._result.result}")
 
			print("-- Disarming")
			await self.drone.action.disarm()
			return
 
	async def listener_callback(self, msg):
		float_array = msg.data  
		self.get_logger().info(f'Received data: {float_array}')
 
		id_value = float_array[0]
		x_value = float_array[1]
		y_value = float_array[2]
		z_value = float_array[3]
		rotation_value = float_array[4]
 
		self.get_logger().info(f'ID: {id_value}, X: {x_value}, Y: {y_value}, Z: {z_value}, Rotation: {rotation_value}')
 
		self.move_drone(x_value, y_value, z_value, rotation_value)
		print('ID: {id_value}, X: {x_value}, Y: {y_value}, Z: {z_value}, Rotation: {rotation_value}')
 
 
async def async_main(args=None):
	rclpy.init(args=args)
	subscriber = FloatArraySubscriber()
 
	await subscriber.initialize()
 
	rclpy.spin(subscriber)
 
	subscriber.destroy_node()
	rclpy.shutdown()
 
def main(args=None):
	asyncio.run(async_main())
 
 
if __name__ == "__main__":
	# Run the asyncio loop
	main()