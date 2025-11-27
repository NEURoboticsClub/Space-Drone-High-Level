#!/usr/bin/env python3
import asyncio
from mavsdk import System


async def startup_and_arm(drone):
    # Connect to PX4 SITL
    await drone.connect(system_address="udpin://0.0.0.0:14540")

    print("Waiting for connection...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("-- Connected to drone")
            break

    print("Waiting until drone is armable (all prechecks passed)...")
    async for health in drone.telemetry.health():
        if health.is_armable:
            print("-- Drone is armable")
            break

    print("-- Arming")
    await drone.action.arm()

    async for armed in drone.telemetry.armed():
        if armed:
            print("-- Drone armed and ready")
            break


if __name__ == "__main__":
    drone = System()
    asyncio.run(startup_and_arm(drone))

