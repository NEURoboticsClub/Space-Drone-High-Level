import asyncio

from model.flight_control import FlightControl
from view.flight_view import FlightView
from controller.flight_mode import FlightMode

class Controller:
    def __init__(self, model: FlightControl, view: FlightView) -> None:
        self.model = model
        self.view = view

    async def fly(self):
        self.view.try_connect()
        await self.model.connect()
        self.view.connected()

        self.view.check_position()
        await self.model.check_position()
        self.view.valid_position()

        calibrate = self.view.get_calibrate()

        flight_mode = self.view.get_flight_mode()

        if calibrate:
            pass # Calibration code

        if flight_mode == FlightMode.takeoff_and_land:
            self.view.arm()
            await self.model.vehicle.arm()

            self.view.takeoff()
            await self.model.vehicle.takeoff()
            
            delay = self.view.get_delay()

            await asyncio.sleep(delay)

            self.view.land()
            await self.model.vehicle.land()

            await self.model.terminate()

        if flight_mode == FlightMode.mission:
            pass # Code for mission
        

            