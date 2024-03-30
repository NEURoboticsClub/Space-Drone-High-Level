import cv2
import rclpy
from rclpy.node import Node
from ultralytics import YOLO
from std_msgs.msg import Float32

# Load the YOLOv8 model
model = YOLO('yolov8n.pt')

#determine the area of the bounding box returned by the input image
def calcArea(points):
    xDif = abs(points[0] - points[2])
    yDif = abs(points[1] - points[3])
    area = xDif*yDif
    return area

# determine the percentage of how much an input image takes up a frame
def calcPercentage(area,res):
    return (area / res) * 100

#calculates the center of the bounding box
def centerOBB(points):
    x = int((points[0] + points[2])/2)
    y = int((points[1] + points[3])/2)
    center = (x,y)

    return center

class Publisher(Node):

    def __init__(self):
        super().__init__('Publisher')
        self.publisher = self.create_publisher(Float32, 'area', 10)
        timer_period = 2  # seconds
        self.timer = self.create_timer(timer_period, self.publish_area)
        

    def publish_area(self):

        ip = "http://10.110.136.58:8080/video"
        cap = cv2.VideoCapture(ip)
        cv2.namedWindow("Video",cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Video",600,600)

        w = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        h = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        res = w*h
        frame_center = (int(w/2),int(h/2))
        color = (0,255,255)
        

        # Loop through the video frames
        while cap.isOpened():
            # Read a frame from the video
            success, frame = cap.read()
            

            if success:
                # Run YOLOv8 inference on the frame
                results = model(frame)

                # Visualize the results on the frame
                annotated_frame = results[0].plot()
                
                # View results
                for r in results:
                    if r.boxes.xyxy != None:

                        coordinates = [int(coordinate) for coordinate in r.boxes.xyxy[0]]
                        box_center = centerOBB(coordinates)
                        cv2.circle(annotated_frame,box_center,10,(0,0,255),cv2.FILLED)
                        
                        cv2.circle(annotated_frame,frame_center,10,color, cv2.FILLED)
                        cv2.imshow("Video", annotated_frame)
                        cv2.waitKey(2000)
                        area = calcPercentage(calcArea(coordinates),res)
                        msg = Float32()
                        msg.data = area
                        self.publisher.publish(msg)
                        self.get_logger().info('Publishing area: "%s"' % msg.data)
                    else:
                        print("Error: The list 'r.boxes.xyxy' is empty.")

                    

                # Break the loop if 'q' is pressed
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
                else:
                # Break the loop if the end of the video is reached
                    break

        # Release the video capture object and close the display window
        cap.release()
        cv2.destroyAllWindows()
        

def main(args=None):
    
    rclpy.init(args=args)

    pub = Publisher()

    rclpy.spin(pub)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()






