from view.flight_view import FlightView
from view.console_drone_view import ConsoleDroneView
from model.drone import Drone


class ConsoleFlightView(FlightView):
    """Console flight view for drone flight.
    Allows user input to control the drone for a test flight
    """

    def __init__(self, drone: Drone) -> None:
        """Create a new console flight view

        Args:
            drone (Drone): drone object
        """
        drone_view = ConsoleDroneView(drone)
        super().__init__(drone_view)

    def get_calibrate(self) -> bool:
        """Ask the user if they want to calibrate the drone

        Returns:
            bool: True if the user wants to calibrate the drone
        """
        result = input("Do you want to calibrate the drone? (y/n): ")
        return result.lower() == "y"

    def get_flight_mode(self) -> int:
        """Get the flight mode from the user

        Returns:
            int: 1 for take off and land, 2 for mission
        """
        print("Please select the flight mode from the following options: ")
        print("1 - Takeoff and Land")
        print("2 - Mission")
        result = input("Enter your flight mode choice: ")
        return int(result)

    def get_delay(self) -> int:
        """Get the length of the flight for take off and land mode

        Returns:
            int: length of flight in seconds
        """
        result = input("Please specify the duration of flight: ")
        return int(result)

    def get_mission(self) -> list:
        """Get the mission consisting of GPS points from the user.

        Returns:
            list: list of GPS points as tuples (latitude, longitude)
        """
        print("Please enter your mission points: ")
        mission = []
        while True:
            point = input("Enter the next mission point \
                           (latitude, longitude) or 0 to end :")
            if int(point[0]) == 0:
                break
            coordinates = [float(num) for num in point.split(",")]
            mission.append((coordinates[0], coordinates[1]))

        return mission

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

    def calibrate(self):
        print("***Running Calibration Routine***")

    def gyroscope_calibrated(self):
        """Report gyroscope calibration
        """
        print("***Gyroscope calibrated successfully***")

    def accelerometer_calibrated(self):
        """Report accelerometer calibration
        """
        print("***Accelerometer calibrated successfully***")

    def magnetometer_calibrated(self):
        """Report magnetometer calibration
        """
        print("***Magnetometer calibrated successfully***")

    def board_level_calibrated(self):
        """Report board horizon calibration
        """
        print("***Board horizon level calibrated successfully***")
