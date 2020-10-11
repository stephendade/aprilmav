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
import os
import json

from lib import cameraArduCamUC580

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("-camera", type=str, default="ArduCamUC580", help="Camera profile in camera.yaml")
    parser.add_argument("-loop", type=int, default=20, help="Capture this many frames")
    parser.add_argument("-folder", type=str, default="capture", help="Put capture into this folder")
    args = parser.parse_args()
    
    print("Initialising Camera")

    # Open camera settings
    with open('camera.yaml', 'r') as stream:
        parameters = yaml.load(stream, Loader=yaml.FullLoader)
        
    #create the capture folder if required
    try:
        os.makedirs(os.path.join(".", args.folder))
    except FileExistsError:
        pass
    
    # initialize the camera
    camera = cameraArduCamUC580.cameraUC580(parameters[args.camera])
        
    print("Starting {0} image capture...".format(args.loop))
    

    for i in range(args.loop):
        myStart = time.time()

        imageBW = camera.getImage()

        # get time to capture and convert
        print("Time to capture = {0:.0f}ms".format((time.time() - myStart)*1000))

        # write image to file - don't time this
        cv2.imwrite(os.path.join(".", args.folder, "capture_{:04d}.jpg".format(i)),imageBW, [cv2.IMWRITE_JPEG_QUALITY, 99])

    # close camera
    camera.close()
    