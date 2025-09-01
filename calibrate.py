import os
from pathlib import Path
import numpy as np
import cv2 as cv
from glob import glob

src_dir = Path(__file__).resolve().parent
root_proj_dir = src_dir


calibration_file = str(src_dir) + '/camera_params.csv'

checker_rows_cols = (7, 9)
# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((checker_rows_cols[1] * checker_rows_cols[0],3), np.float32)
objp[:,:2] = np.mgrid[0:checker_rows_cols[0],0:checker_rows_cols[1]].T.reshape(-1,2)

'''
OpenCV documentation specifies "at least 10 test patterns for camera calibration".
source: https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html
'''
success_threshold = 10
photos = glob(str(root_proj_dir) + '/photos/calibrate/*.jpg')

print("Found {} images to calibrate camera".format(len(photos)))

img_points = []
obj_points = []

real_img_size = None
window_size = (11, 11)
zero_zone = (-1, -1)
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

for photo in photos:
    image = cv.imread(photo)
    gray_image = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
    ret, corners = cv.findChessboardCorners(gray_image, checker_rows_cols, None)
    if ret:
        print("Found corners in {}".format(photo))

        if real_img_size is None:
            real_img_size = (gray_image.shape[1], gray_image.shape[0])

        refined_corners = cv.cornerSubPix(gray_image,
                                          corners,
                                          window_size,
                                          zero_zone,
                                          criteria)
        obj_points.append(objp)
        img_points.append(refined_corners)

        # show the points in a downscaled image (optional)
        '''
        cv.drawChessboardCorners(image, checker_rows_cols, refined_corners, ret)
        height, width = image.shape[:2] 
        reshape = (int(width/4), int(height/4))
        scaled = cv.resize(image, reshape)
        cv.imshow("show corners", scaled)
        cv.waitKey(0)
        cv.destroyAllWindows()
        '''

    else:
        print("Couldn't find corners in {}, skipping...".format(photo))
        '''
        height, width = image.shape[:2] 
        reshape = (int(width/4), int(height/4))
        scaled = cv.resize(image, reshape)
        cv.imshow("show corners", scaled)
        cv.waitKey(0)
        cv.destroyAllWindows()
        '''


if len(img_points) < success_threshold:
    print("FAILURE: Found less than 10 photos with corners...")
    exit(1)


flags = (cv.CALIB_ZERO_TANGENT_DIST |
         cv.CALIB_FIX_K1 |
         cv.CALIB_FIX_K2 |
         cv.CALIB_FIX_K3 |
         cv.CALIB_FIX_K4 |
         cv.CALIB_FIX_K5 |
         cv.CALIB_FIX_K6)

ret, K, dcoeffs, rvecs, tvecs = cv.calibrateCamera(obj_points,
                                                   img_points,
                                                   real_img_size,
                                                   cameraMatrix=None,
                                                   distCoeffs=np.zeros(6),
                                                   flags=flags)

print()
print('all units below measured in pixels:')
print('  fx = {}'.format(K[0,0]))
print('  fy = {}'.format(K[1,1]))
print('  cx = {}'.format(K[0,2]))
print('  cy = {}'.format(K[1,2]))
print()
print('writing to file: {}'.format(calibration_file))


file_exists = True
if not os.path.exists(calibration_file):
    os.mknod(calibration_file)
    file_exists = False


with open(calibration_file, 'w') as f:
    if file_exists:
        f.truncate(0) # erase contents
    f.write(str(K[0,0]) + ',')
    f.write(str(K[1,1]) + ',')
    f.write(str(K[0,2]) + ',')
    f.write(str(K[1,2]) + '\n')



