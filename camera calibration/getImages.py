import cv2
import os

current_dir = os.path.dirname(os.path.abspath(__file__))

cap = cv2.VideoCapture(0)
_, frame = cap.read()

num = 0

# # parameters to draw chessboard points
# chessboardSize = (6,9)
# frameSize = (frame.shape[1],frame.shape[0])
# criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

while cap.isOpened():

    succes, img = cap.read()

    k = cv2.waitKey(5)

    # # Find the chess board corners
    # gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # ret, corners = cv2.findChessboardCorners(gray, chessboardSize, None, cv2.CALIB_CB_FAST_CHECK)

    # if ret == True:
    #     corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
    #     # Draw and display the corners
    #     cv2.drawChessboardCorners(img, chessboardSize, corners2, ret)
    #     cv2.imshow('img', img)
    #     cv2.waitKey(1000)

    if k == 27:
        break
    elif k == ord('s'): # wait for 's' key to save and exit
        cv2.imwrite(os.path.join(current_dir,'images/img' + str(num) + '.png'), img)
        print("image saved!")
        num += 1

    cv2.imshow('Img',img)

# Release and destroy all windows before termination
cap.release()

cv2.destroyAllWindows()
