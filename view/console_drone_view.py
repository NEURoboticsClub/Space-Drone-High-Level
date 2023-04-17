from view.drone_view import DroneView
from model.drone import Drone

class ConsoleDroneView(DroneView):

    def __init__(self, drone: Drone) -> None:
        super().__init__(drone)

    async def display(self):
        await self.drone.return_position()
        position = self.drone.position
        print(f"Current position: {position[0]}, {position[1]}")
