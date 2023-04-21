from enum import IntEnum


class FlightMode(IntEnum):
    """Stores flight modes for the drone:
        take_off_and_land  1,
        mission  2
    """

    takeoff_and_land = 1

    mission = 2
