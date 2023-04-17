import asyncio
from mavsdk.mission import (MissionItem, MissionPlan)


class Mission:

    # Sample Mission Points for reference

    A = MissionItem(47.398039859999997, 8.5455725400000002, 25, 10, True, float('nan'), float('nan'), MissionItem.CameraAction.NONE,
                    float('nan'), float('nan'), float('nan'), float('nan'), float('nan'))
    
    B = MissionItem(47.398036222362471,8.5450146439425509, 25, 10, True, float('nan'), float('nan'), MissionItem.CameraAction.NONE,
                    float('nan'), float('nan'), float('nan'), float('nan'), float('nan'))
    
    C = MissionItem(47.397825620791885, 8.5450092830163271, 25, 10, True, float('nan'), float('nan'), MissionItem.CameraAction.NONE,
                    float('nan'), float('nan'), float('nan'), float('nan'), float('nan'))
    
    def __init__(self, mission_points: list = []) -> None:
        self.mission_points = mission_points

    def get_mission(self):
        return MissionPlan(self.mission_points)
    
    def add_mission_point(self, lat: float, long: float):
        self.mission_points.append(MissionItem(lat, long, 25, 10, True, float('nan'), float('nan'), MissionItem.CameraAction.NONE,
                    float('nan'), float('nan'), float('nan'), float('nan'), float('nan')))
    

    