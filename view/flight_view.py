import asyncio

from abc import ABC, abstractmethod
from view.drone_view import DroneView


class FlightView(ABC):
    """Abstract class for a flight view
    Allows user control for a drone flight
    """

    def __init__(self, drone_view: DroneView) -> None:
        """Creates a new flight view

        Args:
            drone_view (DroneView): drone view
        """
        self.drone_view = drone_view

    @abstractmethod
    def get_flight_mode(self):
        """Get the flight mode from the user
        """
        pass

    @abstractmethod
    def get_mission(self):
        """Get the mission from the user
        """
        pass

    @abstractmethod
    def try_connect(self):
        """Report a drone connection attempt
        """
        pass

    @abstractmethod
    def connected(self):
        """Report that the drone is connected
        """
        pass

    @abstractmethod
    def check_position(self):
        """Report drone position check
        """
        pass

    @abstractmethod
    def valid_position(self):
        """Report valid drone position
        """
        pass

    @abstractmethod
    def arm(self):
        """Report drone armed state
        """
        pass

    @abstractmethod
    def takeoff(self):
        """Report drone takeoff
        """
        pass

    @abstractmethod
    def land(self):
        """Report drone landing
        """
        pass

    @abstractmethod
    def calibrate(self):
        """Report drone calibration
        """
        pass

    async def display_drone(self):
        """Display the position of the drone to the user
        """
        await self.drone_view.display()
