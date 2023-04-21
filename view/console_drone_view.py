from view.drone_view import DroneView
from model.drone import Drone


class ConsoleDroneView(DroneView):
    """Console view for the drone.
    Outputs the current state of the drone to the console"""

    def __init__(self, drone: Drone) -> None:
        """Create a new console drone view

        Args:
            drone (Drone): drone object
        """
        super().__init__(drone)

    async def display(self):
        """Output drone position (latitute, longitude) to the console
        """
        await self.drone.return_position()
        position = self.drone.position
        print(f"Current position: {position[0]}, {position[1]}")
