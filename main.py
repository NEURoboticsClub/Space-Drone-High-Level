import asyncio

from model.flight_control import FlightControl
from view.console_flight_view import ConsoleFlightView
from controller.flight_controller import Controller


model = FlightControl()
view = ConsoleFlightView(model.vehicle)
controller = Controller(model, view)

async def main():
    await controller.fly()
    return

asyncio.run(main())