import asyncio

from abc import ABC, abstractmethod
from model.drone import Drone


class DroneView(ABC):
    """Abstract class to display the current state of the drone"""

    def __init__(self, drone: Drone) -> None:
        """Create a new drone view

        Args:
            drone (Drone): drone object
        """
        self.drone = drone

    @abstractmethod
    async def display(self):
        """Display the drone
        """
        pass
