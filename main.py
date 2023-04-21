import asyncio

from model.flight_control import FlightControl
from view.console_flight_view import ConsoleFlightView
from controller.flight_controller import Controller

model = FlightControl()
view = ConsoleFlightView(model.vehicle)
controller = Controller(model, view)


# Create an async function for drone flight
async def main():
    await controller.fly()

# Run the async drone flight routine
asyncio.run(main())
