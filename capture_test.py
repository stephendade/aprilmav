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

from picamera import PiCamera


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("-camera", type=str, default="PiCamV2LowRes", help="Camera profile in camera.yaml")
    parser.add_argument("-loop", type=int, default=10, help="Capture this many frames")
    args = parser.parse_args()
    
    print("Initialising")

    # Open camera settings
    with open('camera.yaml', 'r') as stream:
        parameters = yaml.load(stream, Loader=yaml.FullLoader)
    camParams = parameters[args.camera]

    # initialize the camera
    camera = PiCamera(resolution=camParams['resolution'], framerate=camParams['framerate'],sensor_mode=camParams['sensor_mode'])
    camera.rotation = camParams['rotation']

    # allow the camera to warmup
    time.sleep(2)

    # Current image
    image = numpy.empty((camera.resolution[0] * camera.resolution[1] * 3,),
                        dtype=numpy.uint8)

    print("Starting {0} image capture...".format(args.loop))

    for i in range(args.loop):
        myStart = time.time()

        # grab an image from the camera
        camera.capture(image, format="bgr", use_video_port=camParams['use_video_port'])

        # and convert to greyscale
        image = image.reshape((camera.resolution[1], camera.resolution[0], 3))
        imageBW = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # get time to capture and convert
        print("Time to capture = {0}".format(time.time() - myStart))

        # write image to file - don't time this
        cv2.imwrite("capture_{0}.jpg".format(i), image)

