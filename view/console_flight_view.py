from view.flight_view import FlightView
from view.console_drone_view import ConsoleDroneView
from model.drone import Drone


class ConsoleFlightView(FlightView):
    """Console flight view for drone flight.
    Allows user input to control the drone for a test flight
    """

    def __init__(self, drone: Drone, file_name: str) -> None:
        """Create a new console flight view

        Args:
            drone (Drone): drone object
        """
        drone_view = ConsoleDroneView(drone)
        super().__init__(drone_view, file_name)

    def write(self, message: str):
        """Write a message to the log file and print it to the console

        Args:
            message (str): message
        """
        super().write(message)
        print(message)
    
    def get_calibrate(self) -> bool:
        """Ask the user if they want to calibrate the drone

        Returns:
            bool: True if the user wants to calibrate the drone
        """
        self.write("Do you want to calibrate the drone? (y/n): ")
        result = input()
        super().write(result)
        return result.lower() == "y"

    def get_flight_mode(self) -> int:
        """Get the flight mode from the user

        Returns:
            int: 1 for take off and land, 2 for mission
        """
        self.write("Please select the flight mode from the following options: ")
        self.write("1 - Takeoff and Land")
        self.write("2 - Mission")
        self.write("Enter your flight mode choice: ")
        result = input()
        super().write(result)
        return int(result)

    def get_delay(self) -> int:
        """Get the length of the flight for take off and land mode

        Returns:
            int: length of flight in seconds
        """
        self.write("Please specify the duration of flight: ")
        result = input()
        super().write(result)
        return int(result)

    def get_mission(self) -> list:
        """Get the mission consisting of GPS points from the user.

        Returns:
            list: list of GPS points as tuples (latitude, longitude)
        """
        self.write("Please enter your mission points: ")
        mission = []
        while True:
            self.write("Enter the next mission point \
                           (latitude, longitude) or 0 to end :")
            point = input()
            super().write(point)
            if int(point[0]) == 0:
                break
            coordinates = [float(num) for num in point.split(", ")]
            mission.append((coordinates[0], coordinates[1]))

        return mission
    
    def return_to_launch(self) -> bool:
        """Ask the user if they want the drone to return
        to home position after mission is complete.

        Returns:
            bool: True if return home
        """
        self.write("Do you want to return to the home position after mission is complete? (y/n): ",
                    end = "")
        result = input().lower()
        super().write(result)
        return  result == "y"
    
    def try_connect(self):
        """Report a drone connection attempt to the console
        """
        self.write("***Trying to connect...***")

    def connected(self):
        """Report that the drone is connected to the console
        """
        self.write("***Connected successfully!***")

    def check_position(self):
        """Report drone position check to the console
        """
        self.write("***Checking GPS Position***")

    def valid_position(self):
        """Report valid drone position to the console
        """
        self.write("***GPS Position Valid***")

    def arm(self):
        """Report drone armed state to the console
        """
        self.write("***Armed***")

    def takeoff(self):
        """Report drone takeoff to the console
        """
        self.write("***Taking off***")

    def land(self):
        """Report drone landing to the console
        """
        self.write("***Landing***")

    def calibrate(self):
        """Report drone calibration to the console
        """
        self.write("***Running Calibration Routine***")

    def gyroscope_calibrated(self):
        """Report gyroscope calibration
        """
        self.write("***Gyroscope calibrated successfully***")

    def accelerometer_calibrated(self):
        """Report accelerometer calibration
        """
        self.write("***Accelerometer calibrated successfully***")

    def magnetometer_calibrated(self):
        """Report magnetometer calibration
        """
        self.write("***Magnetometer calibrated successfully***")

    def board_level_calibrated(self):
        """Report board horizon calibration
        """
        self.write("***Board horizon level calibrated successfully***")
