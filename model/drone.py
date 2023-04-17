import asyncio
from mavsdk import System

class Drone:

    def __init__(self, address: str = "udp://:14540") -> None:
        self.address = address
        self.drone = System()

    async def connect(self):
        await self.drone.connect(system_address=self.address)
        
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

    async def disarm(self):
        await self.drone.action.disarm()

        
    