import asyncio

from abc import ABC, abstractmethod
from view.drone_view import DroneView

class FlightView(ABC):
    def __init__(self, drone_view: DroneView) -> None:
        self.drone_view = drone_view

    @abstractmethod
    def try_connect(self):
        pass
    
    @abstractmethod
    def connected(self):
        pass
    
    @abstractmethod
    def check_position(self):
        pass

    @abstractmethod
    def valid_position(self):
        pass

    @abstractmethod
    def arm(self):
        pass

    @abstractmethod
    def takeoff(self):
        pass

    @abstractmethod
    def land(self):
        pass

    async def display_drone(self):
        await self.drone_view.display()
