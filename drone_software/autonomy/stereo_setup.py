import os
import sys

if __name__ == "__main__":
    arming_dir = os.path.join(os.path.dirname(__file__), '..', 'autonomy')
    arming_dir = os.path.abspath(arming_dir)
    sys.path.insert(0, arming_dir)
    from capture_stereo_frames import capture_images
    from compute_disparity import compute_disparity
    from stereo_calibration import stereo_calibrate

    capture_images()
    commute_disparity()
    stereo_calibrate()
