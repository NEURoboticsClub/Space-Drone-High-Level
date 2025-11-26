# https://github.com/mavlink/MAVSDK-Python/blob/main/examples/offboard_position_ned.py


async def startup_and_arm(drone):
	await drone.connect(system_address="udpin://0.0.0.0:14540")

	print("Waiting for connection...")
	async for state in drone.core.connection_state():
		if state.is_connected:
		  print("-- Connected to drone")
		  break

	print("Waiting for position estimate (GPS or optical flow)...")
	async for health in drone.telemetry.health():
		  if health.is_global_position_ok or health.is_local_position_ok:
		    print("-- Position estimate OK")
		    break

	print("Checking battery...")
	async for battery in drone.telemetry.battery():
		if battery.remaining_percent < 0.2:
    	raise Exception("Battery too low")
		print(f"-- Battery OK: {battery.remaining_percent*100:.1f}%")
		break

	print("Waiting for external authorization...")
	if not await external_authorization():
	  raise Exception("Arming denied")
	print("-- Authorization granted")

	print("-- Arming")
	await drone.action.arm()

	async for armed in drone.telemetry.armed():
	  if armed:
      print("-- Drone armed and ready")
      break

