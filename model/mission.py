import asyncio
from mavsdk.mission import (MissionItem, MissionPlan)


class Mission:
    """Mission consisting of GPS points for the drone to follow.
    Allows creation of new mission points with only latitute and longitude.

    """

    def __init__(self, mission_points: list = []) -> None:
        """Create a new mission for the drone

        Args:
            mission_points (list, optional): mission items. Defaults to [].
        """
        self.mission_points = mission_points

    def get_mission(self):
        """Return the mission

        Returns:
            MissionPlan: mission plan
        """
        return MissionPlan(self.mission_points)

    def add_mission_point(self, lat: float, long: float):
        """Add a new mission point to the mission plan

        Args:
            lat (float): latitude
            long (float): longitude
        """
        self.mission_points.append(MissionItem(lat, long, 25, 10, True,
                                               float('nan'), float('nan'),
                                               MissionItem.CameraAction.NONE,
                                               float('nan'), float('nan'),
                                               float('nan'), float('nan'),
                                               float('nan')))
