from view.drone_view import DroneView
from model.drone import Drone

class ConsoleDroneView(DroneView):

    def __init__(self, drone: Drone) -> None:
        super().__init__(drone)

    def display(self):
        
        print(f"Current pose: {self.drone.return_position()}")
