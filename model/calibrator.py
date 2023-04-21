from model.drone import Drone


class Calibrator:
    """Full calibration of a drone
    """

    def __init__(self, drone: Drone) -> None:
        """Create a new calibrator

        Args:
            drone (Drone): drone to calibrate
        """
        self.drone = drone.drone

    async def gyroscope(self):
        """Calibrate the gyroscope
        """
        self.drone.calibration.calibrate_gyro()

    async def accelerometer(self):
        """Calibrate the accelerometer
        """
        self.drone.calibration.calibrate_accelerometer()

    async def magnetometer(self):
        """Calibrate the magnetometer
        """
        self.drone.calibration.calibrate_magnetometer()

    async def board_level(self):
        """Calibrate the board level sensor
        """
        self.drone.calibration.calibrate_level_horizon()
