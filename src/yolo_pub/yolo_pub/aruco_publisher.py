'''
Code used from https://github.com/niconielsen32/ComputerVision/tree/master/ArUco
and also referred to his YT videos
'''

#imports
import numpy as np
import time
import cv2
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

#dictionary for tags
ARUCO_DICT = {
	"DICT_4X4_50": cv2.aruco.DICT_4X4_50,
	"DICT_4X4_100": cv2.aruco.DICT_4X4_100,
	"DICT_4X4_250": cv2.aruco.DICT_4X4_250,
	"DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
	"DICT_5X5_50": cv2.aruco.DICT_5X5_50,
	"DICT_5X5_100": cv2.aruco.DICT_5X5_100,
	"DICT_5X5_250": cv2.aruco.DICT_5X5_250,
	"DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
	"DICT_6X6_50": cv2.aruco.DICT_6X6_50,
	"DICT_6X6_100": cv2.aruco.DICT_6X6_100,
	"DICT_6X6_250": cv2.aruco.DICT_6X6_250,
	"DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
	"DICT_7X7_50": cv2.aruco.DICT_7X7_50,
	"DICT_7X7_100": cv2.aruco.DICT_7X7_100,
	"DICT_7X7_250": cv2.aruco.DICT_7X7_250,
	"DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
	"DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
	"DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
	"DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
	"DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
	"DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
}

#display detecting bounding box
def aruco_display(corners, ids, rejected, image):
	if len(corners) > 0:
		
		ids = ids.flatten()
		
		for (markerCorner, markerID) in zip(corners, ids):
			
			corners = markerCorner.reshape((4, 2))
			(topLeft, topRight, bottomRight, bottomLeft) = corners
			
			topRight = (int(topRight[0]), int(topRight[1]))
			bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
			bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
			topLeft = (int(topLeft[0]), int(topLeft[1]))

			#draw bounding box lines
			cv2.line(image, topLeft, topRight, (0, 255, 0), 2)
			cv2.line(image, topRight, bottomRight, (0, 255, 0), 2)
			cv2.line(image, bottomRight, bottomLeft, (0, 255, 0), 2)
			cv2.line(image, bottomLeft, topLeft, (0, 255, 0), 2)
			
			#draw center of the box
			cX = int((topLeft[0] + bottomRight[0]) / 2.0)
			cY = int((topLeft[1] + bottomRight[1]) / 2.0)
			cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)
			
			#display id number
			cv2.putText(image, str(markerID),(topLeft[0], topLeft[1] - 10), cv2.FONT_HERSHEY_SIMPLEX,
				0.5, (0, 255, 0), 2)
			print("[Inference] ArUco marker ID: {}".format(markerID))
			
	return image

#calculate pose of the detected tag
def pose_estimation(corners, marker_size, matrix_coefficients, distortion_coefficients):
    
	#define 3d object points
    marker_points = np.array([[-marker_size / 2, marker_size / 2, 0],
                              [marker_size / 2, marker_size / 2, 0],
                              [marker_size / 2, -marker_size / 2, 0],
                              [-marker_size / 2, -marker_size / 2, 0]], dtype=np.float32)
    trash = []
    rvecs = []
    tvecs = []

	#use solvePnP function for pose estimation
    for c in corners:
            _, R, t = cv2.solvePnP(marker_points, c, matrix_coefficients, distortion_coefficients, False, cv2.SOLVEPNP_IPPE_SQUARE)
            rvecs.append(R)
            tvecs.append(t)
            trash.append(_)

    return rvecs, tvecs, trash



class Publisher(Node):
	
    def __init__(self):
        super().__init__('Publisher')
        self.publisher = self.create_publisher(Float32, 'tag_poses', 10)
        timer_period = 2
        self.timer = self.create_timer(timer_period, self.cb_publish_pose)
		

    def cb_publish_pose(self):
		
        #define the markers to be detected
        aruco_type = "DICT_5X5_100"

        arucoDict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[aruco_type])
        arucoParams = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(arucoDict, arucoParams)

        #capture images from the camera
        ip = "http://192.168.1.119:8080/video"		#external camera, use 0 for default camera on the computer
        cap = cv2.VideoCapture(ip)

        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 500)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 500)

        #calibration values for the camera
        calibration_matrix = np.array(((1484.001664055822, 0, 945.7762352668258),
                                    (0,1484.001664055822, 520.8557474898012),
                                    (0,0,1)))
        distortion_params = np.array((-0.0179606305852,0.180716809921,-0.00015874041681,-0.00190823724612,-0.333257089362))


        while cap.isOpened():
            
            ret, img = cap.read()
            h, w, _ = img.shape

            img = cv2.resize(img, (500, 500), interpolation=cv2.INTER_CUBIC)
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)	#convert the image to grayscale for detection
            corners, ids, rejected = detector.detectMarkers(gray)	#detect
            marker_size = 100	#define marker size in proper units (subject to change)

            if ids is not None:
                #obtain rotational and translation vectors wrt tags
                rvecs, tvecs, _ = pose_estimation(corners, marker_size, calibration_matrix, distortion_params)
                
                #draw axes, box and print information of each tag in the frame
                for i in range(len(ids)):
                    cv2.aruco.drawDetectedMarkers(img, corners)
                    cv2.drawFrameAxes(img, calibration_matrix, distortion_params, rvecs[i], tvecs[i], 5)
                    print(f"ID: {ids[i]}, Rvec: {rvecs[i]}, Tvec: {tvecs[i]}")
                    
                    #placeholder for published msgs
                    msg = Float32()
                    msg.data = tvecs[0] #just publishing the first element in tvecs
                    print('Publishing data')
                    self.publisher.publish(msg)
                    

            cv2.imshow("Image", img)

            #press q to quit
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break

        cv2.destroyAllWindows()
        cap.release()

def main():
    rclpy.init()
    pub = Publisher()
    rclpy.spin(pub)
    
    pub.destroy_node(pub)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
