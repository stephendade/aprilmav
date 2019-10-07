#!/usr/bin/env python3
'''
Camera calibration.

Whilst the capture loop is running, show a
n*m chessboard in front on the camera.

It is important that the camera sees the chessboard
in a variety of rotations and skew angles.

All chessboard detected images will be stored as "cal_N.jpg"

'''
import argparse
import numpy
import cv2
import time
import sys

from picamera import PiCamera


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("-cbcol", type=int, default=6, help="Number of chessboard columns-1")
    parser.add_argument("-cbrow", type=int, default=9, help="Number of chessboard rows-1")
    parser.add_argument("-mode", type=int, default=5, help="PiCam sensor mode")
    parser.add_argument("-framerate", type=int, default=30, help="PiCam framerate")
    parser.add_argument("-rotation", type=int, default=180, help="PiCam rotation (roll) (degrees)")
    parser.add_argument("-hres", type=int, default=800, help="PiCam vertical resolution")
    parser.add_argument("-vres", type=int, default=608, help="PiCam horizontal resolution")
    parser.add_argument("-staticmode", action="store_true", help="Use photo rather than video mode")
    args = parser.parse_args()
    
    # sanity check the resolution
    if args.hres % 16 != 0 or args.vres % 16 != 0:
        print("Error: Resolution must be divisible by 16")
        sys.exit(0)

    # initialize the camera
    camera = PiCamera(resolution=(args.hres, args.vres), framerate=args.framerate,sensor_mode=args.mode)
    camera.rotation = args.rotation
    
    # Chessboard rows and cols
    cbcol = args.cbcol
    cbrow = args.cbrow

    # allow the camera to warmup
    time.sleep(1)

    # Current image
    image = numpy.empty((camera.resolution[0] * camera.resolution[1] * 3,),
                        dtype=numpy.uint8)

    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = numpy.zeros((cbrow * cbcol, 3), numpy.float32)
    objp[:, :2] = numpy.mgrid[0:cbcol, 0:cbrow].T.reshape(-1, 2)
    
    objpoints = [] # 3d point in real world space
    imgpoints = [] # 2d points in image plane.

    print("Starting 30 image capture at ~1/sec...")

    for i in range(1, 30):
        # grab an image from the camera
        camera.capture(image, format="bgr", use_video_port=not args.staticmode)

        # and convert to greyscale
        image = image.reshape((camera.resolution[1], camera.resolution[0], 3))
        grey = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        print("Got image {0}/30".format(i))
       
        # process
        ret, corners = cv2.findChessboardCorners(grey, (cbcol, cbrow), flags=cv2.CALIB_CB_ADAPTIVE_THRESH)
        if (ret):
            print("Found chessboard in image {0}/30".format(i))
            corners2 = cv2.cornerSubPix(grey,corners,(11,11),(-1,-1),(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01))
            objpoints.append(objp)
            imgpoints.append(corners2)
            cv2.imwrite("cal_{0}.jpg".format(i), image)
            
        time.sleep(1)
        
    # and process
    if len(imgpoints) < 15:
        print("Error: Less than 15 images with detected chessboard. Aborting")
    else:
        print("Got images, processing...")
        ret, K, D, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, grey.shape[::-1], None, None)
        
        print("------Calibration success-------")
        print("K = {0}".format(K))
        print("CameraParams = {0}".format(( K[0,0], K[1,1], K[0,2], K[1,2] )))
        
        print("Put the following in camera.yaml:")
        print("<profilename>:")
        print("  cam_params: !!python/tuple [{0}, {1}, {2}, {3}]".format(K[0,0], K[1,1], K[0,2], K[1,2]))
        print("  resolution: !!python/tuple [{0}, {1}]".format(camera.resolution[0], camera.resolution[1]))
        print("  rotation: {0}".format(camera.rotation))
        print("  use_video_port: {0}".format(not args.staticmode))
        print("  sensor_mode: {0}".format(camera.sensor_mode))
        print("  framerate: {0}".format(camera.framerate))
