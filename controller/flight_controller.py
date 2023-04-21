import asyncio

from model.flight_control import FlightControl
from view.flight_view import FlightView
from controller.flight_mode import FlightMode


class Controller:
    """Perform a user controlled flight of a drone or simulation.
    Can run calibration routines, take off and land and perform missions.
    """

    def __init__(self, model: FlightControl, view: FlightView) -> None:
        """Create a new controller for the drone flight

        Args:
            model (FlightControl): flight controller
            view (FlightView): flight view
        """
        self.model = model
        self.view = view

    async def fly(self):
        """Conduct a drone flight with user input.
        Run a calibration routine per user request.
        Run either a "take off and land" or a "mission" flight mode.
        """
        self.view.try_connect()
        await self.model.connect()
        self.view.connected()

        self.view.check_position()
        await self.model.check_position()
        self.view.valid_position()

        await self.view.display_drone()

        calibrate = self.view.get_calibrate()

        
        if calibrate:
            # Run the calibration routine

            self.view.calibrate()

            await(self.model.calibrator.gyroscope())
            self.view.gyroscope_calibrated()

            await(self.model.calibrator.accelerometer())
            self.view.accelerometer_calibrated()

            await(self.model.calibrator.magnetometer())
            self.view.magnetometer_calibrated()

            await(self.model.calibrator.board_level())
            self.view.board_level_calibrated()


        
        flight_mode = self.view.get_flight_mode()

        if flight_mode == FlightMode.takeoff_and_land:
            # Take off the drone and land it after the delay

            delay = self.view.get_delay()

            self.view.arm()
            await self.model.vehicle.arm()

            self.view.takeoff()
            await self.model.vehicle.takeoff()

            await asyncio.sleep(delay)

            self.view.land()
            await self.model.vehicle.land()

        if flight_mode == FlightMode.mission:
            # Conduct a mission of GPS Points

            return_to_launch = self.view.return_to_launch()

            await self.model.return_to_launch(return_to_launch)

            mission_points = self.view.get_mission(self.model)

            for point in mission_points:
                self.model.mission.add_mission_point(point[0], point[1])

            await self.model.vehicle.upload_mission(
                self.model.mission.get_mission())

            self.view.arm()
            await self.model.vehicle.arm()

            await self.model.vehicle.start_mission()
