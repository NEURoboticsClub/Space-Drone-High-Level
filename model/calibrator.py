from model.drone import Drone

class Calibrator:
    def __init__(self, drone: Drone) -> None:
        self.drone = drone.drone

    async def gyroscope(self):
        self.drone.calibration.calibrate_gyro()

    async def accelerometer(self):
        self.drone.calibration.calibrate_accelerometer()

    async def magnetometer(self):
        self.drone.calibration.calibrate_magnetometer()

    async def board_level(self):
        self.drone.calibration.calibrate_level_horizon()