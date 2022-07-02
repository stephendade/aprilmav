#!/usr/bin/env python3
'''
Apriltag capture and detection performance test.

For debugging, also shows distance to each detected tag.

Distance is relative to the camera's sensor in 3 dimensions.

'''

import time
import numpy
import cv2
import yaml
import argparse

from importlib import import_module
from dt_apriltags import Detector
from lib.geo import getPos, getTransform, getRotation


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--camera", type=str, default="PiCamV2FullFoV", help="Camera profile in camera.yaml")
    parser.add_argument("--loop", type=int, default=10, help="Process this many frames")
    parser.add_argument("--tagSize", type=int, default=96, help="Apriltag size in mm")
    parser.add_argument("--folder", type=str, default=None, help="Use a folder of images instead of camera")
    parser.add_argument("--outfile", type=str, default="processed.csv", help="Output tag data to this file")
    parser.add_argument("--decimation", type=int, default=2, help="Apriltag decimation")
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

    at_detector = Detector(searchpath=['apriltags3py/apriltags/lib', 'apriltags3py/apriltags/lib'],
                           families='tagStandard41h12',
                           nthreads=3,
                           quad_decimate=args.decimation,
                           quad_sigma=0.0,
                           refine_edges=1,
                           decode_sharpening=0.25,
                           debug=0)
                           
    # how many loops
    loops = camera.getNumberImages() if camera.getNumberImages() else args.loop
    
    print("Starting {0} image capture and process...".format(loops))
    
    outfile = open(args.outfile,"w+")
    outfile.write("{0},{1},{2},{3},{4},{5},{6},{7},{8}\n".format("Filename", "TagID", "PosX (left)", "PosY (up)", "PosZ (fwd)", "RotX (pitch)", "RotY (yaw)", "RotZ (roll)", "PoseErr"))

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
           
            tags = at_detector.detect(undistorted_img, True, camParams['cam_params'], args.tagSize/1000)
        else:
            tags = at_detector.detect(imageBW, True, camParams['cam_params'], args.tagSize/1000)
            
        if file:
            print("File: {0}".format(file))
        
        # get time to capture and convert
        print("Time to capture and detect = {0:.3f} sec, found {1} tags".format(time.time() - myStart, len(tags)))
        
        for tag in tags:
                        
            tagpos = getPos(getTransform(tag))
            tagrot = getRotation(getTransform(tag))
            
            print("Tag {0} pos = {1} m, Rot = {2} deg".format(tag.tag_id, tagpos.round(3), tagrot.round(1)))
            outfile.write("{0},{1},{2:.3f},{3:.3f},{4:.3f},{5:.1f},{6:.1f},{7:.1f},{8}\n".format(file,
                                                                     tag.tag_id,
                                                                     tagpos[0],
                                                                     tagpos[1],
                                                                     tagpos[2],
                                                                     tagrot[0],
                                                                     tagrot[1],
                                                                     tagrot[2],
                                                                     tag.pose_err))
            

    outfile.close()

