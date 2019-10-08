#!/usr/bin/env python3
'''
Camera Capture performance test

All images will be stored as "capture_N.jpg"

'''

import time
import numpy
import cv2
import argparse
import yaml

from lib import cameraPi


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("-camera", type=str, default="PiCamV2LowRes", help="Camera profile in camera.yaml")
    parser.add_argument("-loop", type=int, default=10, help="Capture this many frames")
    args = parser.parse_args()
    
    print("Initialising")

    # Open camera settings
    with open('camera.yaml', 'r') as stream:
        parameters = yaml.load(stream, Loader=yaml.FullLoader)

    # initialize the camera
    camera = cameraPi.cameraPi(parameters[args.camera])

    # allow the camera to warmup
    time.sleep(2)

    print("Starting {0} image capture...".format(args.loop))

    for i in range(args.loop):
        myStart = time.time()

        imageBW = camera.getImage()

        # get time to capture and convert
        print("Time to capture = {0}".format(time.time() - myStart))

        # write image to file - don't time this
        cv2.imwrite("capture_{0}.jpg".format(i), imageBW)

