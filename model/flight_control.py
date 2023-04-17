import asyncio
from mavsdk import System

from model.drone import Drone
from model.calibrator import Calibrator
from model.mission import Mission

class FlightControl():
    
    def __init__(self) -> None:
        self.vehicle = Drone()
        self.calibrator = Calibrator(self.vehicle)
        self.mission = Mission()


    async def connect(self):
        await self.vehicle.connect()
        async for state in self.vehicle.drone.core.connection_state():
            if state.is_connected:
                break

    async def check_position(self):
        async for health in self.vehicle.drone.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok:
                break

    async def return_to_launch(self, state):
        await self.vehicle.drone.mission.set_return_to_launch_after_mission(state)



    