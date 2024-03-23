import cv2
from djitellopy import Tello

drone = Tello()
drone.connect()
drone.streamon()
print(drone.get_udp_video_address())
print(drone.get_battery())
# cap = cv2.VideoCapture("udp://@0.0.0.0:11111")

num = 0

while True:

    img = drone.get_frame_read().frame
    # succes, img = cap.read()

    k = cv2.waitKey(5)
    if k == ord('q'):
        break

    if k == 27:
        break
    elif k == ord('s'): # wait for 's' key to save and exit
        cv2.imwrite('images/img' + str(num) + '.png', img)
        print("image saved!")
        num += 1

    cv2.imshow('Img',img)

# Release and destroy all windows before termination
# cap.release()

cv2.destroyAllWindows()
drone.streamoff()