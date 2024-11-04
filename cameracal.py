#!/usr/bin/env python3
'''
Camera calibration.

Whilst the capture loop is running, show a
n*m chessboard in front on the camera.

It is important that the camera sees the chessboard
in a variety of rotations and skew angles.

All chessboard detected images will be stored as "cal_N.jpg"

This can either run over the Ras Pi camera in realtime OR over
a folder of pre-captured images from any camera

'''
import argparse
import numpy
import cv2
import os
import glob

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--cbcol", type=int, default=6, help="Number of chessboard columns-1")
    parser.add_argument("--cbrow", type=int, default=9, help="Number of chessboard rows-1")
    parser.add_argument("--folder", type=str, default=None, help="Use a folder of images instead of camera")
    parser.add_argument("--fisheye", action="store_true", help="Use Fisheye calibration model")
    parser.add_argument("--halfres", action="store_true", help="Use half resolution")
    args = parser.parse_args()

    # initialize the camera
    from lib import cameraFile
    camera = cameraFile.FileCamera(args.folder)

    # Chessboard rows and cols
    cbcol = args.cbcol
    cbrow = args.cbrow

    # Image dimensions
    imgDim = None

    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = numpy.zeros((1, cbrow*cbcol, 3), numpy.float32)
    objp[0, :, :2] = numpy.mgrid[0:cbcol, 0:cbrow].T.reshape(-1, 2)
    shape = None

    objpoints = []  # 3d point in real world space
    imgpoints = []  # 2d points in image plane.

    print("Starting 30 image capture at ~1/sec...")

    # how many loops
    loops = camera.getNumberImages()

    for i in range(loops):
        # grab an image from the camera
        grey = camera.getImage()

        if args.halfres:
            grey = cv2.resize(grey, None, fx= 0.5, fy= 0.5, interpolation= cv2.INTER_AREA)

        if i == 0:
            # get image dimensions
            imgDim = grey.shape[::-1]

        # we're out of images
        if grey is None:
            break

        print("Got image {0}/{1}".format(i, loops))

        # process
        ret, corners = cv2.findChessboardCorners(grey, (cbcol, cbrow), flags=cv2.CALIB_CB_ADAPTIVE_THRESH)
        if (ret):
            print("Found chessboard in image {0}/{1}".format(i, loops))
            shape = grey.shape[::-1]
            corners2 = cv2.cornerSubPix(grey, corners, (11, 11), (-1, -1),
                                        (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01))
            objpoints.append(objp)
            imgpoints.append(corners2)
            # cv2.imwrite("cal_{0}.jpg".format(i), grey)

        # time.sleep(1)

    # close camera
    camera.close()

    # and process
    if len(imgpoints) < 10:
        print("Error: Less than 10 (Got {0}) images with detected chessboard. Aborting".format(len(imgpoints)))
    else:
        print("Got images, processing...")
        K = numpy.zeros((3, 3))
        D = numpy.zeros((4, 1))
        rvecs = [numpy.zeros((1, 1, 3), dtype=numpy.float64) for i in range(len(imgpoints))]
        tvecs = [numpy.zeros((1, 1, 3), dtype=numpy.float64) for i in range(len(imgpoints))]

        if args.fisheye:
            calibration_flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC+cv2.fisheye.CALIB_CHECK_COND+cv2.fisheye.CALIB_FIX_SKEW

            rms, _, _, _, _ = \
                cv2.fisheye.calibrate(
                    objpoints,
                    imgpoints,
                    grey.shape[::-1],
                    K,
                    D,
                    rvecs,
                    tvecs,
                    calibration_flags,
                    (cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6)
                )
        else:
            ret, K, D, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, shape, None, None)

        print("------Calibration success-------")
        print("K = {0}".format(K))
        print("CameraParams = {0}".format((K[0, 0], K[1, 1], K[0, 2], K[1, 2])))
        if args.fisheye:
            print("D = {0}".format(D))
            print("CameraParamsD = {0}".format((D[0][0], D[1][0], D[2][0], D[3][0])))

        print("Put the following in camera.yaml:")
        print("<profilename>:")
        print("  cam_params: !!python/tuple [{0}, {1}, {2}, {3}]".format(K[0, 0], K[1, 1], K[0, 2], K[1, 2]))
        if args.fisheye:
            print("  cam_paramsD: !!python/tuple [[{0}], [{1}], [{2}], [{3}]]".format(D[0][0], D[1][0],
                                                                                      D[2][0], D[3][0]))
        if args.halfres:
            print("  resolution: !!python/tuple [{0}, {1}]".format(imgDim[0]*2, imgDim[1]*2))
        else:
            print("  resolution: !!python/tuple [{0}, {1}]".format(imgDim[0], imgDim[1]))
        print("  fisheye: {0}".format(args.fisheye))
        print("  halfres: {0}".format(args.halfres))

        if args.fisheye:
            # Show un-distortion of fisheye to user
            img = cv2.imread(glob.glob(os.path.join(args.folder, "*.jpg"))[0])
            if args.halfres:
                img = cv2.resize(img, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)
            dim1 = img.shape[:2][::-1]  # dim1 is the dimension of input image to un-distort
            map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, numpy.eye(3), K, dim1, cv2.CV_16SC2)
            undistorted_img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)

            cv2.imshow("undistorted", undistorted_img)
            cv2.imshow("none undistorted", img)
            print("Type 0 into image window to exit")
            cv2.waitKey(0)
            cv2.destroyAllWindows()
