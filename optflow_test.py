#!/usr/bin/env python3
'''
Optical flow capture and detection performance test.

'''

import time
import numpy
import cv2
import yaml
import argparse

from importlib import import_module
from dt_apriltags import Detector
from lib.geo import getPos, getTransform, getRotation
from lib.optflow import OptFlow

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--camera", type=str, default="PiCamV2FullFoV", help="Camera profile in camera.yaml")
    parser.add_argument("--loop", type=int, default=10, help="Process this many frames")
    parser.add_argument("--folder", type=str, default=None, help="Use a folder of images instead of camera")
    parser.add_argument("--outfile", type=str, default="optflow.csv", help="Output optical flow data to this file")
    parser.add_argument('--gui', dest='gui', default=False, action='store_true', help="Show frames in GUI")
    args = parser.parse_args()
    
    print("Initialising")

    # Open camera settings
    with open('camera.yaml', 'r') as stream:
        parameters = yaml.load(stream, Loader=yaml.FullLoader)
    camParams = parameters[args.camera]

    # initialize the camera
    camera = None
    if args.folder:
        from lib import cameraFile
        camera = cameraFile.FileCamera(args.folder)
    else:
        try:
            print(parameters[args.camera]['cam_name'])
            mod = import_module("lib." + parameters[args.camera]['cam_name'])
            camera = mod.camera(parameters[args.camera])
        except (ImportError, KeyError):
            print('No camera with the name {0}, exiting'.format(args.camera))
            sys.exit(0)
    
    # allow the camera to warmup
    time.sleep(2)

    # Optical flow lives here
    flow = OptFlow(False, args.gui)
                           
    # how many loops
    loops = camera.getNumberImages() if camera.getNumberImages() else args.loop
    
    print("Starting {0} image capture and process...".format(loops))
    
    outfile = open(args.outfile,"w+")
    outfile.write("{0},{1},{2},{3},{4},{5}\n".format("Filename", "FlowX", "FlowY", "NumPoints", "Std devX", "Std devY"))

    # Need to reconstruct K and D
    if camParams['fisheye']:
        K = numpy.zeros((3, 3))
        D = numpy.zeros((4, 1))
        K[0,0] = camParams['cam_params'][0]
        K[1,1] = camParams['cam_params'][1]
        K[0,2] = camParams['cam_params'][2]
        K[1,2] = camParams['cam_params'][3]
        K[2,2] = 1
        D[0][0] = camParams['cam_paramsD'][0]
        D[1][0] = camParams['cam_paramsD'][1]
        D[2][0] = camParams['cam_paramsD'][2]
        D[3][0] = camParams['cam_paramsD'][3]
            
    for i in range(loops):
        print("--------------------------------------")
        myStart = time.time()

        # grab an image from the camera
        file = camera.getFileName()
        imageBW = camera.getImage()
        
        # we're out of images
        if imageBW is None:
            break
            
        # AprilDetect, after accounting for distortion  (if fisheye)
        if camParams['fisheye']:
            dim1 = imageBW.shape[:2][::-1]  #dim1 is the dimension of input image to un-distort
            map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, numpy.eye(3), K, dim1, cv2.CV_16SC2)
            undistorted_img = cv2.remap(imageBW, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
            flow.newFrame(undistorted_img, None)
        else:
            flow.newFrame(imageBW, None)
        if file:
            print("File: {0}".format(file))
        
        # get time to capture and convert
        print("Time to capture and detect = {0:.3f} sec".format(time.time() - myStart))
        
        print("Flow X = {0:.0f}px, Y = {1:.0f}px".format(flow.flowspeed[0], flow.flowspeed[1]))
        outfile.write("{0},{1:.0f},{2:.0f},{3},{4:.1f},{5:.1f}\n".format(file,
                                                                 flow.flowspeed[0],
                                                                 flow.flowspeed[1],
                                                                 len(flow.p0),
                                                                 flow.flowstd[0],
                                                                 flow.flowstd[1],
                                                                 ))
            

    outfile.close()

