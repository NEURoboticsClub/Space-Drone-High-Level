import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class VideoPublisher(Node):
    def __init__(self):
        super().__init__('video_publisher')
        self.publisher = self.create_publisher(Image, '/camera/image_raw', 10)
        self.timer = self.create_timer(0.1, self.publish_frame)
        self.bridge = CvBridge()
        

    def publish_frame(self):
        ip = "http://10.110.136.58:8080/video"
        cap = cv2.VideoCapture(ip) 
        cv2.namedWindow("Video",cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Video",600,600)

        while cap.isOpened():
            success, frame = cap.read()  # Read a frame from the camera
            if success:
                # Convert the OpenCV frame to a ROS Image message
                cv2.imshow("Video", frame)
                msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'camera_frame'
                # Publish the Image message
                self.publisher.publish(msg)
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