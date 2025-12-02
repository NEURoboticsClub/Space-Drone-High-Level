import cv2
import numpy as np
import glob

def stereo_calibrate():
    chessboard_size = (9,6)
    square_size = 0.025 

    objp = np.zeros((np.prod(chessboard_size),3), np.float32)
    objp[:,:2] = np.indices(chessboard_size).T.reshape(-1,2)
    objp *= square_size

    objpoints = []
    imgpoints_left = []
    imgpoints_right = []

    images_left = sorted(glob.glob("calib/left_*.png"))
    images_right = sorted(glob.glob("calib/right_*.png"))

    for imgL, imgR in zip(images_left, images_right):
        grayL = cv2.imread(imgL, cv2.IMREAD_GRAYSCALE)
        grayR = cv2.imread(imgR, cv2.IMREAD_GRAYSCALE)

        retL, cornersL = cv2.findChessboardCorners(grayL, chessboard_size)
        retR, cornersR = cv2.findChessboardCorners(grayR, chessboard_size)

        if retL and retR:
            objpoints.append(objp)
            imgpoints_left.append(cornersL)
            imgpoints_right.append(cornersR)

    retL, mtxL, distL, _, _ = cv2.calibrateCamera(objpoints, imgpoints_left, grayL.shape[::-1], None, None)
    retR, mtxR, distR, _, _ = cv2.calibrateCamera(objpoints, imgpoints_right, grayR.shape[::-1], None, None)

    flags = cv2.CALIB_FIX_INTRINSIC
    retS, _, _, _, _, R, T, E, F = cv2.stereoCalibrate(
        objpoints, imgpoints_left, imgpoints_right,
        mtxL, distL, mtxR, distR, grayL.shape[::-1],
        criteria=(cv2.TERM_CRITERIA_MAX_ITER+cv2.TERM_CRITERIA_EPS, 100, 1e-5),
        flags=flags
    )

    RL, RR, PL, PR, Q, _, _ = cv2.stereoRectify(mtxL, distL, mtxR, distR, grayL.shape[::-1], R, T)

    np.savez("stereo_calib.npz", mtxL=mtxL, distL=distL, mtxR=mtxR, distR=distR,
             R=R, T=T, RL=RL, RR=RR, PL=PL, PR=PR, Q=Q)

    print("Stereo calibration complete and saved.")
