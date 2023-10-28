from ultralytics import YOLO

# Load a pretrained YOLOv8n model
model = YOLO('yolov8n.pt')

# Run inference on an image
results = model('bus.jpg')  # results list

# View results
for r in results:
    #names = [name for name in r.boxes.cls]
    coordinates = [int(coordinate) for coordinate in r.boxes.xyxy[0]]
    print(coordinates)  # print the Boxes object containing the detection bounding boxes
    #print(names)