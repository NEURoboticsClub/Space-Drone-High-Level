import asyncio
from mavsdk import System

from model.drone import Drone

class FlightControl():
    
    def __init__(self, mission = False) -> None:
        self.vehicle = Drone()
        #self.status_text_task = asyncio.ensure_future(self.vehicle.print_status_text())
        if mission:
            self.print_mission_progress_task = asyncio.ensure_future(self.vehicle.print_mission_progress())
            self.running_tasks = [self.print_mission_progress_task]
            self.termination_task = asyncio.ensure_future(self.vehicle.observe_is_in_air(self.running_tasks))

        else:
            self.termination_task = None


    async def connect(self):
        await self.vehicle.connect()
        async for state in self.vehicle.drone.core.connection_state():
            if state.is_connected:
                break

    async def check_position(self):
        async for health in self.vehicle.drone.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok:
                break

    async def return_to_launch(self):
        await self.vehicle.drone.mission.set_return_to_launch_after_mission(True)

    async def terminate(self):
        #self.status_text_task.cancel()
        if self.termination_task != None:
            await self.termination_task

    