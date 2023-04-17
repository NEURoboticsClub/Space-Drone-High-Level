import asyncio
from mavsdk import System

class Drone:

    def __init__(self, address: str = "udp://:14540") -> None:
        self.address = address
        self.drone = System()

    async def connect(self):
        await self.drone.connect(system_address=self.address)
    
    # async def print_status_text(self):
    #     try:
    #         async for status_text in self.drone.telemetry.status_text():
    #             return (f"Status: {status_text.type}: {status_text.text}")
    #     except asyncio.CancelledError:
    #         return
        
    async def print_mission_progress(self):
        async for mission_progress in self.drone.mission.mission_progress():
            return(f"Mission progress: "
                f"{mission_progress.current}/"
                f"{mission_progress.total}")
        
    async def observe_is_in_air(self, running_tasks):
        """ Monitors whether the drone is flying or not and
        returns after landing """

        was_in_air = False

        async for is_in_air in self.drone.telemetry.in_air():
            if is_in_air:
                was_in_air = is_in_air

            if was_in_air and not is_in_air:
                for task in running_tasks:
                    task.cancel()
                    try:
                        await task
                    except asyncio.CancelledError:
                        pass
                await asyncio.get_event_loop().shutdown_asyncgens()

                return
        
    async def upload_mission(self, mission_plan):
        await self.drone.mission.upload_mission(mission_plan)
        
    async def arm(self):
        await self.drone.action.arm()

    async def start_mission(self):
        await self.drone.mission.start_mission()

    async def takeoff(self):
        await self.drone.action.takeoff()

    async def land(self):
        await self.drone.action.land()

        
    