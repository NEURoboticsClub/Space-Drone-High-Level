import asyncio

from model.flight_control import FlightControl
from view.console_flight_view import ConsoleFlightView
from controller.flight_controller import Controller
from controller.flight_mode import FlightMode


model = FlightControl()
view = ConsoleFlightView(model.vehicle)
controller = Controller(model, view)

async def main():
    await controller.fly(FlightMode.takeoff_and_land, delay = 1)
    return

asyncio.run(main())