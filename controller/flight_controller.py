import asyncio

from model.flight_control import FlightControl
from view.flight_view import FlightView
from controller.flight_mode import FlightMode

class Controller:
    def __init__(self, model: FlightControl, view: FlightView) -> None:
        self.model = model
        self.view = view

    async def fly(self, flight_mode, calibrate = False, mission = None, delay = 0):
        await self.model.connect()
        await self.model.check_position()

        if calibrate:
            pass # Calibration code

        if flight_mode == FlightMode.takeoff_and_land:
            self.view.arm()
            await self.model.vehicle.arm()

            self.view.takeoff()
            await self.model.vehicle.takeoff()
            
            await asyncio.sleep(delay)

            self.view.land()
            await self.model.vehicle.land()

            await self.model.terminate()

        if flight_mode == FlightMode.mission:
            pass # Code for mission
        

            