#!/usr/bin/env python3
import asyncio
from mavsdk import System


async def test_hover(drone):
    await drone.connect(system_address="udpin://0.0.0.0:14540")

    print("Waiting for connection...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("-- Connected to drone")
            break

    print("-- Taking off to 5 meters")
    await drone.action.takeoff()

    # Wait until altitude is reached
    async for position in drone.telemetry.position():
        if position.relative_altitude_m >= 4.5:
            print(f"-- Hovering at {position.relative_altitude_m:.1f} m")
            break

    # Hover for 10 seconds
    await asyncio.sleep(10)

    print("-- Landing")
    await drone.action.land()


if __name__ == "__main__":
    drone = System()
    asyncio.run(test_hover(drone))

