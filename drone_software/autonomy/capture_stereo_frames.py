import cv2

def capture_images():
    cap_left = cv2.VideoCapture(0)  
    cap_right = cv2.VideoCapture(1)  
    i = 0
    while True:
        retL, frameL = cap_left.read()
        retR, frameR = cap_right.read()
        if not retL or not retR:
            break

        cv2.imshow("Left", frameL)
        cv2.imshow("Right", frameR)

        key = cv2.waitKey(1)
        if key == ord('s'):  
            cv2.imwrite(f"calib/left_{i}.png", frameL)
            cv2.imwrite(f"calib/right_{i}.png", frameR)
            print(f"Saved pair {i}")
            i += 1
        elif key == ord('q'):
            break

    cap_left.release()
    cap_right.release()
    cv2.destroyAllWindows()

