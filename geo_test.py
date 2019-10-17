#!/usr/bin/env python3
'''
This script localises any visible Apriltags and
calulates the camera's rotation and position as a
transformation matrix T_CamToWorld, relative to it's
starting location.

Requires a Raspberry Pi Camera V2.

Note camera settings are specific to the camera model and
settings. Use cameracal.py to generate new settings and put
into camera.yaml
'''
import time
import math
import numpy
import cv2
import yaml
import argparse

from apriltags3py.apriltags3 import Detector
from lib.geo import tagDB

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("-tagSize", type=int, default=115, help="Apriltag size in mm")
    parser.add_argument("-camera", type=str, default="PiCamV2LowRes", help="Camera profile in camera.yaml")
    parser.add_argument("-loop", type=int, default=20, help="Capture and process this many frames")
    parser.add_argument("-maxerror", type=int, default=100, help="Maximum pose error to use, in n*E-8 units")
    parser.add_argument("-folder", type=str, default=None, help="Use a folder of images instead of camera")
    args = parser.parse_args()
    
    print("Initialising")

    # Open camera settings
    with open('camera.yaml', 'r') as stream:
        parameters = yaml.load(stream, Loader=yaml.FullLoader)
    camParams = parameters[args.camera]
        
    # initialize the camera
    if args.folder == None:
        from lib import cameraPi
        camera = cameraPi.cameraPi(parameters[args.camera])
    else:
        from lib import cameraFile
        camera = cameraFile.FileCamera(args.folder)

    # allow the camera to warmup
    time.sleep(2)

    at_detector = Detector(searchpath=['apriltags3py/apriltags/lib', 'apriltags3py/apriltags/lib'],
                           families='tagStandard41h12',
                           nthreads=3,
                           quad_decimate=1.0,
                           quad_sigma=0.2,
                           refine_edges=1,
                           decode_sharpening=0.2,
                           debug=0)

    # All tags live in here
    tagPlacement = tagDB(0, 0, -0.07)
    
    # how many loops
    loops = camera.getNumberImages() if camera.getNumberImages() else args.loop

    print("Starting {0} image capture and process...".format(loops))

    for i in range(loops):
        print("--------------------------------------")
        
        myStart = time.time()

        # grab an image from the camera
        file = camera.getFileName()
        imageBW = camera.getImage()
        
        # we're out of images
        if imageBW is None:
            break
            
        # AprilDetect
        tags = at_detector.detect(imageBW, True, camParams['cam_params'], args.tagSize/1000)

        # add any new tags to database, or existing one to duplicates
        tagsused = 0
        for tag in tags:
            if tag.pose_err < args.maxerror*1e-8:
                tagsused += 1
                tagPlacement.addTag(tag)
                              
        tagPlacement.getBestTransform()

        #x = left
        print("Pos {0}, Rot = {3} in {1} with {4}/{2} tags".format(tagPlacement.getCurrentPosition().round(3), file, len(tags), tagPlacement.getCurrentRotation().round(0), tagsused))
        #print("Time to capture, detect and localise = {0:.3f} sec, using {2}/{1} tags".format(time.time() - myStart, len(tags), len(tagPlacement.tagDuplicatesT)))
        
        tagPlacement.newFrame()
                
                        
        #cv2.imwrite("detect_{0}.jpg".format(i), image)

