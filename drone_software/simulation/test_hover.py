#!/usr/bin/env python3
import asyncio
import sys
import os
from mavsdk import System


async def test_hover(drone):
    await drone.connect(system_address="udp://:14540")

    print("Waiting for connection...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("-- Connected to drone")
            break

    print("-- Taking off to 5 meters")
    await drone.action.set_takeoff_altitude(5)
    print("set takeoff altitude")
    await drone.action.takeoff()

    # Wait until altitude is reached
    async for position in drone.telemetry.position():
        if position.relative_altitude_m >= 4.8:
            print(f"-- Reached target altitude")
            break
        print(f"-- Hovering at {position.relative_altitude_m:.1f} m")

    # Hover for 10 seconds
    await asyncio.sleep(10)

    print("-- Landing")
    await drone.action.land()


if __name__ == "__main__":
    arming_dir = os.path.join(os.path.dirname(__file__), '..', 'flight_control')
    arming_dir = os.path.abspath(arming_dir)
    sys.path.insert(0, arming_dir)
    from precheck import startup_and_arm

    drone = System()
    asyncio.run(startup_and_arm(drone))
    asyncio.run(test_hover(drone))

