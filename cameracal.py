#!/usr/bin/env python3
'''
Camera calibration.

Use "capture_test.py" to capture images of a
n*m chessboard in front on the camera.

It is important that the camera sees the chessboard
in a variety of rotations and skew angles.

This can be run over a folder of pre-captured images from any camera

'''
import argparse
import os
import cv2
import numpy
import queue
from concurrent.futures import ThreadPoolExecutor


def getImagepoints(image, i, loops, cbcol, cbrow, objpoints, imgpoints):
    ret, corners = cv2.findChessboardCorners(
        image, (cbcol, cbrow), flags=cv2.CALIB_CB_ADAPTIVE_THRESH)
    if ret:
        print("Found chessboard in image {0}/{1}".format(i, loops))
        corners2 = cv2.cornerSubPix(image, corners, (11, 11), (-1, -1),
                                    (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01))
        objpoints.put(objp)
        imgpoints.put(corners2)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--cbcol", type=int, default=6,
                        help="Number of chessboard columns-1")
    parser.add_argument("--cbrow", type=int, default=9,
                        help="Number of chessboard rows-1")
    parser.add_argument("--folder", type=str, default=None,
                        help="Use a folder of images")
    parser.add_argument("--fisheye", action="store_true",
                        help="Use Fisheye calibration model")
    parser.add_argument("--halfres", action="store_true",
                        help="Use half resolution")
    args = parser.parse_args()

    # initialize the camera
    from drivers import cameraFile
    camera = cameraFile.FileCamera(None, args.folder)

    # Chessboard rows and cols
    cbcol = args.cbcol
    cbrow = args.cbrow

    # Image dimensions
    imgDim = None

    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = numpy.zeros((1, cbrow*cbcol, 3), numpy.float32)
    objp[0, :, :2] = numpy.mgrid[0:cbcol, 0:cbrow].T.reshape(-1, 2)

    objpoints = queue.Queue()  # 3d point in real world space
    imgpoints = queue.Queue()  # 2d points in image plane.

    # how many loops
    loops = camera.getNumberImages()

    # Use multithreading to speed up, but limit to number of cores - 1
    max_workers = max(os.cpu_count() - 1, 1)
    executor = ThreadPoolExecutor(max_workers=max_workers)
    futures = []

    for i in range(loops):
        # grab an image from the camera
        (grey, timestamp) = camera.getImage(get_raw=True)

        if args.halfres:
            grey = cv2.resize(grey, None, fx=0.5, fy=0.5,
                              interpolation=cv2.INTER_AREA)

        if i == 0:
            # get image dimensions
            imgDim = grey.shape[::-1]

        # we're out of images
        if grey is None:
            break

        print("Got image {0}/{1}".format(i, loops))

        futures.append(executor.submit(getImagepoints, grey, i, loops, cbcol, cbrow, objpoints, imgpoints))

        # start blocking if there are too many threads
        if len(futures) >= max_workers:
            for future in futures:
                future.result()
            futures = []

    # close camera
    camera.close()

    # wait for all threads to finish
    for future in futures:
        future.result()

    # and process
    imgpointsList = list(imgpoints.queue)
    objpointsList = list(objpoints.queue)
    if len(imgpointsList) < 10:
        print("Error: Less than 10 (Got {0}) images with detected chessboard. Aborting".format(
            len(imgpointsList)))
    else:
        print("Got images, processing...")
        K = numpy.zeros((3, 3))
        D = numpy.zeros((4, 1))
        rvecs = [numpy.zeros((1, 1, 3), dtype=numpy.float64)
                 for i in range(len(imgpointsList))]
        tvecs = [numpy.zeros((1, 1, 3), dtype=numpy.float64)
                 for i in range(len(imgpointsList))]

        if args.fisheye:
            calibration_flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC + \
                cv2.fisheye.CALIB_CHECK_COND+cv2.fisheye.CALIB_FIX_SKEW

            rms, _, _, _, _ = \
                cv2.fisheye.calibrate(
                    objpointsList,
                    imgpointsList,
                    imgDim,
                    K,
                    D,
                    rvecs,
                    tvecs,
                    calibration_flags,
                    (cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6)
                )
        else:
            ret, K, D, rvecs, tvecs = cv2.calibrateCamera(
                objpointsList, imgpointsList, imgDim, None, None)

        print("------Calibration success-------")
        print("K = {0}".format(K))
        print("CameraParams = {0}".format(
            (K[0, 0], K[1, 1], K[0, 2], K[1, 2])))
        if args.fisheye:
            print("D = {0}".format(D))
            print("CameraParamsD = {0}".format(
                (D[0][0], D[1][0], D[2][0], D[3][0])))

        print("Put the following in camera.yaml:")
        print("<profilename>:")
        print("  cam_params: !!python/tuple [{0}, {1}, {2}, {3}]".format(K[0, 0], K[1, 1], K[0, 2], K[1, 2]))
        if args.fisheye:
            print("  cam_paramsD: !!python/tuple [{0}, {1}, {2}, {3}]".format(D[0][0], D[1][0],
                                                                                      D[2][0], D[3][0]))
        if args.halfres:
            print("  resolution: !!python/tuple [{0}, {1}]".format(imgDim[0]*2, imgDim[1]*2))
        else:
            print("  resolution: !!python/tuple [{0}, {1}]".format(imgDim[0], imgDim[1]))
        print("  fisheye: {0}".format(args.fisheye))
        print("  halfres: {0}".format(args.halfres))
        print("  cam_driver: <>")
        print("  model: <>")
        print("  rotationRelVehicle: !!python/tuple [0, 0, 0]   #roll-pitch-yaw in degrees")
        print("  positionRelVehicle: !!python/tuple [0, 0, 0]   #fwd-right-down (NED) in meters")

        if args.fisheye:
            # Show un-distortion of fisheye to user for 1st image
            images = [
                os.path.join(args.folder, file)
                for file in os.listdir(args.folder)
                if file.endswith(('.png', '.jpg'))
            ]
            images.sort()
            img = cv2.imread(images[0])
            if args.halfres:
                img = cv2.resize(img, None, fx=0.5, fy=0.5,
                                 interpolation=cv2.INTER_AREA)
            # dim1 is the dimension of input image to un-distort
            dim1 = img.shape[:2][::-1]
            map1, map2 = cv2.fisheye.initUndistortRectifyMap(
                K, D, numpy.eye(3), K, dim1, cv2.CV_16SC2)
            undistorted_img = cv2.remap(
                img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)

            cv2.namedWindow("Corrected", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("Corrected", 500, 500)
            cv2.imshow("Corrected", undistorted_img)
            cv2.namedWindow("Original", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("Original", 500, 500)
            cv2.imshow("Original", img)
            print("Type 0 into image window to exit")
            cv2.waitKey(0)
            cv2.destroyAllWindows()
