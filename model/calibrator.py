from model.drone import Drone

class Calibrator:
    def __init__(self, drone: Drone) -> None:
        self.drone = drone

    async def gyroscope(self):
        async for progress_data in self.drone.calibration.calibrate_gyro():
            print(progress_data)

    async def accelerometer(self):
        async for progress_data in self.drone.calibration.calibrate_accelerometer():
            print(progress_data)

    async def magnetometer(self):
        async for progress_data in self.drone.calibration.calibrate_magnetometer():
            print(progress_data)

    async def board_level(self):
        async for progress_data in self.drone.calibration.calibrate_level_horizon():
            print(progress_data)