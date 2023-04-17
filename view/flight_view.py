from abc import ABC, abstractmethod
from view.drone_view import DroneView

class FlightView(ABC):
    def __init__(self, drone_view: DroneView) -> None:
        self.drone_view = drone_view

    @abstractmethod
    def arm(self):
        pass

    @abstractmethod
    def takeoff(self):
        pass

    @abstractmethod
    def land(self):
        pass

    def display_drone(self):
        self.drone_view.display()
