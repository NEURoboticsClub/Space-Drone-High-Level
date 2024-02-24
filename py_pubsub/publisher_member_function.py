import cv2
import rclpy
from rclpy.node import Node
from ultralytics import YOLO
from std_msgs.msg import Float32

#Load the YOLOv8 model
model = YOLO('yolov8n.pt')

#determine the area of the bounding box returned by the input image
def calcArea(points):
    xDif = abs(points[0] - points[2])
    yDif = abs(points[1] - points[3])
    area = xDif*yDif
    return area

#determine the percentage of how much an inputted image takes up a frame
def calcPercentage(area,res):
    return (area / res) * 100

class Publisher(Node):

    def init(self):
        super().init('Publisher')
        self.publisher = self.create_publisher(Float32, 'area', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.publish_area)
        
    def publisharea(self):

        # Open the camera
        cap = cv2.VideoCapture(0)
        w = cap.get(cv2.CAPPROPFRAMEWIDTH)
        h = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        res = w*h

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

#View results
                for r in results:
                    coordinates = [int(coordinate) for coordinate in r.boxes.xyxy[0]]
                    area = calcPercentage(calcArea(coordinates),res)
                    msg = Float32()
                    msg.data = area
                    self.publisher.publish(msg)
                    self.get_logger().info('Publishing area: "%s"' % msg.data)

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

    pub = Publisher("publisher")

    rclpy.spin(pub)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()