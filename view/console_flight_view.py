from view.flight_view import FlightView
from view.console_drone_view import ConsoleDroneView
from model.drone import Drone

class ConsoleFlightView(FlightView):
    def __init__(self, drone: Drone) -> None:
        drone_view = ConsoleDroneView(drone)
        super().__init__(drone_view)

    def arm(self):
        print("Armed")

    def takeoff(self):
        print("Taking off")

    def land(self):
        print("Landing")