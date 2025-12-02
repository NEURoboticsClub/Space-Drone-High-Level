import cv2
import numpy as np
import glob

def compute_disparity():
    # Load calibration
    data = np.load("stereo_calib.npz")
    mtxL, distL, mtxR, distR = data["mtxL"], data["distL"], data["mtxR"], data["distR"]
    RL, RR, PL, PR, Q = data["RL"], data["RR"], data["PL"], data["PR"], data["Q"]

    # Open cameras
    cap_left = cv2.VideoCapture(0)
    cap_right = cv2.VideoCapture(1)

    # Init rectification maps
    h, w = int(cap_left.get(cv2.CAP_PROP_FRAME_HEIGHT)), int(cap_left.get(cv2.CAP_PROP_FRAME_WIDTH))
    mapLx, mapLy = cv2.initUndistortRectifyMap(mtxL, distL, RL, PL, (w,h), cv2.CV_32FC1)
    mapRx, mapRy = cv2.initUndistortRectifyMap(mtxR, distR, RR, PR, (w,h), cv2.CV_32FC1)

    stereo = cv2.StereoSGBM_create(numDisparities=64, blockSize=9)

    while True:
        retL, frameL = cap_left.read()
        retR, frameR = cap_right.read()
        if not retL or not retR:
            break

        # Rectify
        rectL = cv2.remap(frameL, mapLx, mapLy, cv2.INTER_LINEAR)
        rectR = cv2.remap(frameR, mapRx, mapRy, cv2.INTER_LINEAR)

        grayL = cv2.cvtColor(rectL, cv2.COLOR_BGR2GRAY)
        grayR = cv2.cvtColor(rectR, cv2.COLOR_BGR2GRAY)

        # Disparity
        disp = stereo.compute(grayL, grayR).astype(np.float32) / 16.0
        disp_vis = cv2.normalize(disp, None, 0, 255, cv2.NORM_MINMAX)
        disp_vis = np.uint8(disp_vis)

        cv2.imshow("Disparity", disp_vis)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap_left.release()
    cap_right.release()
    cv2.destroyAllWindows()
