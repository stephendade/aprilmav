#!/usr/bin/env python3
'''
Apriltag capture and detection performance test.

For debugging, also shows distance to each detected tag.

Distance is relative to the camera's sensor in 3 dimensions.

All images will be stored as "process_N.jpg".
'''

import time
import numpy
import cv2
import yaml
import argparse

from apriltags3py.apriltags3 import Detector
from lib.geo import getTransform, getPos, getRotation


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("-camera", type=str, default="PiCamV2LowRes", help="Camera profile in camera.yaml")
    parser.add_argument("-loop", type=int, default=10, help="Process this many frames")
    parser.add_argument("-tagSize", type=int, default=115, help="Apriltag size in mm")
    parser.add_argument("-folder", type=str, default=None, help="Use a folder of images instead of camera")
    parser.add_argument("-outfile", type=str, default="processed.csv", help="Output tag data to this file")
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
                           quad_decimate=2.0,
                           quad_sigma=0.0,
                           refine_edges=1,
                           decode_sharpening=0.25,
                           debug=0)
                           
    # how many loops
    loops = camera.getNumberImages() if camera.getNumberImages() else args.loop
    
    print("Starting {0} image capture and process...".format(loops))
    
    outfile = open(args.outfile,"w+")
    outfile.write("{0},{1},{2},{3},{4},{5},{6},{7},{8}\n".format("Filename", "TagID", "PosX", "PosY", "PosZ", "RotX", "RotY", "RotZ", "PoseErr"))

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

        # get time to capture and convert
        print("Time to capture and detect = {0:.3f} sec, found {1} tags".format(time.time() - myStart, len(tags)))

        # write image to file with tag details - don't time this
        print("File: {0}".format(file))
        for tag in tags:
                        
            T_TagToCam = getTransform(tag)
            tagpos = getPos(T_TagToCam)
            tagrot = getRotation(T_TagToCam)
            
            print("Tag {0} pos = ({1}, {2}, {3})m".format(tag.tag_id, round(tagpos[0], 3), round(tagpos[1], 3), round(tagpos[2], 3)))
            outfile.write("{0},{1},{2},{3},{4},{5},{6},{7},{8}\n".format(file,
                                                                     tag.tag_id,
                                                                     round(tagpos[0], 3),
                                                                     round(tagpos[1], 3),
                                                                     round(tagpos[2], 3),
                                                                     round(tagrot[0], 3),
                                                                     round(tagrot[1], 3),
                                                                     round(tagrot[2], 3),
                                                                     tag.pose_err))
            

        #cv2.imwrite("detect_{0}.jpg".format(i), imageBW)
    outfile.close()

