import asyncio
from mavsdk import System

from model.drone import Drone
from model.calibrator import Calibrator
from model.mission import Mission


class FlightControl():
    """Flight control for drone or simulation
    """

    def __init__(self) -> None:
        """Create a new flight controller
        """
        self.vehicle = Drone()
        self.calibrator = Calibrator(self.vehicle)
        self.mission = Mission()

    async def connect(self):
        """Run the connection sequence for the drone
        """
        await self.vehicle.connect()
        async for state in self.vehicle.drone.core.connection_state():
            if state.is_connected:
                break

    async def check_position(self):
        """Check if the drone has a valid GPS position
        """
        async for health in self.vehicle.drone.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok:
                break

    async def return_to_launch(self, s: bool):
        """Select whether the drone should return to
        launch site after completing the mission

        Args:
            s (bool): if True, will return to launch
        """
        await self.vehicle.drone.mission.set_return_to_launch_after_mission(s)
