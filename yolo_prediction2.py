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
    return (area / res) * 100

# Loop through the video frames
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
           

        # Break the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
    else:
        # Break the loop if the end of the video is reached
        break

# Release the video capture object and close the display window
cap.release()
cv2.destroyAllWindows()
