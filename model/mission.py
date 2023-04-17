import asyncio
from mavsdk.mission import (MissionItem, MissionPlan)


class Mission:

    def __init__(self, mission_points: list = []) -> None:
        self.mission_points = mission_points

    def get_mission(self):
        return MissionPlan(self.mission_points)
    
    def add_mission_point(self, lat: float, long: float):
        self.mission_points.append(MissionItem(lat, long, 25, 10, True, float('nan'), float('nan'), MissionItem.CameraAction.NONE,
                    float('nan'), float('nan'), float('nan'), float('nan'), float('nan')))
    

    