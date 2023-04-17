from view.flight_view import FlightView
from view.console_drone_view import ConsoleDroneView
from model.drone import Drone

class ConsoleFlightView(FlightView):
    def __init__(self, drone: Drone) -> None:
        drone_view = ConsoleDroneView(drone)
        super().__init__(drone_view)

    def get_calibrate(self):
        result = input("Do you want to calibrate the drone? (y/n)")
        return result.lower() == "y" 

    def get_flight_mode(self):
        print("Please select the flight mode from the following options: ")
        print("1 - Takeoff and Land")
        print("2 - Mission")
        result = input("Enter your flight mode choice: ")
        return int(result)
    
    def get_delay(self):
        result = input("Please specity the duration of flight: ")
        return int(result)
    
    def try_connect(self):
        print("***Trying to connect...***")

    def connected(self):
        print("***Connected successfully!***")

    def check_position(self):
        print("***Checking GPS Position***")

    def valid_position(self):
        print("***GPS Position Valid***")
    
    def arm(self):
        print("***Armed***")

    def takeoff(self):
        print("***Taking off***")

    def land(self):
        print("***Landing***")