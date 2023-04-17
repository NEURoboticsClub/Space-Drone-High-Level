from abc import ABC, abstractmethod
from model.drone import Drone

class DroneView(ABC):
    def __init__(self, drone: Drone) -> None:
        self.drone = drone

    @abstractmethod
    def display(self):
        pass # Display current pose