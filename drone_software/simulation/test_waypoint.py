#!/usr/bin/env python3
import asyncio
from mavsdk import System
from mavsdk.offboard import PositionNedYaw
import os
import sys

async def test_waypoint(drone):
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -2.0, 0.0))
    try:
        await drone.offboard.start()
        await asyncio.sleep(5)
    except Exception as e:
        print(f"Offboard start failed: {e}")
        return

    await drone.offboard.set_position_ned(PositionNedYaw(1.0, 0.0, -2.0, 0.0))
    await asyncio.sleep(5)

    await drone.offboard.set_position_ned(PositionNedYaw(1.0, 0.0, -2.0, 90.0))
    await asyncio.sleep(5)
    await drone.offboard.set_position_ned(PositionNedYaw(1.0, 1.0, -2.0, 90.0))
    await asyncio.sleep(5)

    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 1.0, -2.0, 90.0))
    await asyncio.sleep(5)

    for yaw in [90.0, 180.0, 270.0, 0.0]:
        await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -2.0, yaw))
        await asyncio.sleep(5)

    print("-- Landing")
    await drone.action.land()


async def main():
    arming_dir = os.path.join(os.path.dirname(__file__), '..', 'flight_control')
    arming_dir = os.path.abspath(arming_dir)
    sys.path.insert(0, arming_dir)
    from precheck import startup_and_arm

    drone = System()
    await startup_and_arm(drone)
    await test_waypoint(drone)

if __name__ == "__main__":
    asyncio.run(main())

