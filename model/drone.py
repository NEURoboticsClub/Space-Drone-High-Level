import asyncio
from mavsdk import System


class Drone:
    """Direct control and manipulation of a drone or simulation
    """

    def __init__(self, address: str = "udp://:14540") -> None:
        """Create new drone object

        Args:
            address (_type_, optional): address of the drone.
              Defaults to "udp://:14540".
        """
        self.address = address
        self.drone = System()

    async def return_position(self):
        """Returns the current position (latitude. longitude) of the drone
        """
        result = []
        async for data in self.drone.telemetry.home():
            result.append(data.latitude_deg)
            result.append(data.longitude_deg)
            break

        self.position = result

    async def connect(self):
        """Connect the drone to the system
        """
        await self.drone.connect(system_address=self.address)

    async def upload_mission(self, mission_plan):
        """Upload the mission to the drone

        Args:
            mission_plan (_type_): mission plan object
        """
        await self.drone.mission.upload_mission(mission_plan)

    async def arm(self):
        """Arm the drone to prepare it for flying
        """
        await self.drone.action.arm()

    async def start_mission(self):
        """Begin the mission
        """
        await self.drone.mission.start_mission()

    async def takeoff(self):
        """Takeoff from ground
        """
        await self.drone.action.takeoff()

    async def land(self):
        """Land from air
        """
        await self.drone.action.land()

    async def disarm(self):
        """Disarm the drone, cut power to all motors
        """
        await self.drone.action.disarm()
