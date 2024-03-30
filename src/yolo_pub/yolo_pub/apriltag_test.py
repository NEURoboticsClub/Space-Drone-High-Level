import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import time


class VideoPublisher(Node):
    def __init__(self):
        super().__init__('video_publisher')
        self.publisher1 = self.create_publisher(Image, '/camera/image_raw', 10)
        self.publisher2 = self.create_publisher(CameraInfo, '/camera/camera_info', 10)
        self.timer = self.create_timer(0.1, self.publish_frame)
        self.bridge = CvBridge()
        

    def publish_frame(self):
        ip =  "http://10.0.0.221:8080/video"
        cap = cv2.VideoCapture(ip) 
        cv2.namedWindow("Video",cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Video",600,600)

        # Get the camera resolution
        self.width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

        # Print the camera resolution
        print("Camera Resolution: {} x {}".format(self.width, self.height))

        while cap.isOpened():
            success, frame = cap.read()  # Read a frame from the camera
            
            if success:
                # Convert the OpenCV frame to a ROS Image message
                cv2.imshow("Video", frame)
                stamp = self.get_clock().now().to_msg()     #time stamp used for both publishers for sync
                
                msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                msg.header.stamp= stamp
                msg.header.frame_id = 'camera_frame'
                

                camera_info = CameraInfo()
                camera_info.header.stamp = stamp
                camera_info.header.frame_id = 'camera_info'
                camera_info.height = self.height  # Fill in your camera's resolution
                camera_info.width = self.width   # Fill in your camera's resolution
                camera_info.distortion_model = 'pinhole'
                camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]  # Distortion parameters

                '''
                camera_info.k has random values for now
                Once detections are successful will update with the real values
                '''
                camera_info.k = [500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0]  # Intrinsic matrix
                
                self.publisher2.publish(camera_info)
                self.publisher1.publish(msg)
                self.get_logger().info('Publishing camera info message')
            else:
                self.get_logger().error('Failed to read frame from the camera.')
            
            if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
            
        cap.release()
        cv2.destroyAllWindows()


        

def main(args=None):
    rclpy.init(args=args)
    video_publisher = VideoPublisher()
    rclpy.spin(video_publisher)
    video_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
