#!/usr/bin/env python3

import asyncio
from mavsdk import System

import cv2
from ultralytics import YOLO

# Load the YOLOv8 model
model = YOLO('yolov8n.pt')

# Open the camera
cap = cv2.VideoCapture(0)
w = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
h = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
res = w*h

# determine the area of the bounding box returned by the input image
def calcArea(points):
    xDif = abs(points[0] - points[2])
    yDif = abs(points[1] - points[3])
    area = xDif*yDif
    return area

# determine the percentage of how much an inputted image takes up a frame
def calcPercentage(area):
    return float((area / res) * 100)

async def print_position(drone):
    async for position in drone.telemetry.position():
        return position



async def run():

    drone = System()
    await drone.connect(system_address="udp://:14540")
    
    asyncio.ensure_future(print_position(drone))

    status_text_task = asyncio.ensure_future(print_status_text(drone))

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"-- Connected to drone!")
            break

    print("Waiting for drone to have a global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("-- Global position estimate OK")
            break

    print("-- Arming")
    await drone.action.arm()

    print("-- Taking off")
    await drone.action.takeoff()

    # Wait for the drone to takeoff (otherwise you can't give it any commands)
    await asyncio.sleep(5)

    while cap.isOpened():
    # Read a frame from the video
        success, frame = cap.read()

        if success:
            # Run YOLOv8 inference on the frame
            results = model(frame)

            # Visualize the results on the frame
            annotated_frame = results[0].plot()

            # Display the annotated frame
            cv2.imshow("YOLOv8 Inference", annotated_frame)
            
            # View results
            for r in results:
                coordinates = [int(coordinate) for coordinate in r.boxes.xyxy[0]]
                print(coordinates)  # print the Boxes object containing the detection bounding boxes
                print("Area is", calcPercentage(calcArea(coordinates)), "%") # prints the area taken up as a percentage of camera resolution to determine 'distance'
                position = await print_position(drone)
                await drone.action.goto_location(position.latitude_deg, position.longitude_deg, calcPercentage(calcArea(coordinates)) + 20.0, 0)
                print("Goto command sent")
                await asyncio.sleep(10)
                
            # Break the loop if 'q' is pressed
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
        else:
            # Break the loop if the end of the video is reached
            break

    
        

    await asyncio.sleep(10)

    print("-- Landing")
    #await drone.action.land()

    status_text_task.cancel()


async def print_status_text(drone):
    try:
        async for status_text in drone.telemetry.status_text():
            print(f"Status: {status_text.type}: {status_text.text}")
    except asyncio.CancelledError:
        return


if __name__ == "__main__":
    # Run the asyncio loop
    asyncio.run(run())