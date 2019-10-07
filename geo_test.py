#!/usr/bin/env python3
'''
This script localises any visible Apriltags and
calulates the camera's rotation and position as a
transformation matrix T_CamToWorld, relative to it's
starting location.

Requires a Raspberry Pi Camera V2 and 0.16m Apriltags.

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

from picamera import PiCamera
from apriltags3-py.apriltags3 import Detector

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("-tagSize", type=int, default=160, help="Apriltag size in mm")
    parser.add_argument("-camera", type=str, default="PiCamV2LowRes", help="Camera profile in camera.yaml")
    parser.add_argument("-loop", type=int, default=20, help="Capture and process this many frames")
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

    at_detector = Detector(searchpath=['apriltags/lib', 'apriltags/lib64'],
                           families='tag36h11',
                           nthreads=3,
                           quad_decimate=1.0,
                           quad_sigma=0.0,
                           refine_edges=1,
                           decode_sharpening=0.25,
                           debug=0)

    # Current image
    image = numpy.empty((camera.resolution[0] * camera.resolution[1] * 3,),
                        dtype=numpy.uint8)
                        
    # Current pos and orientation of camera in world frame
    T_CamToWorld = numpy.array( numpy.eye((4)) )
    # Tag positions (T), world frame
    # Key is the tag ID
    tagPlacement = {}

    print("Starting {0} image capture and process...".format(args.loop))

    for i in range(args.loop):
        print("--------------------------------------")
        
        myStart = time.time()

        # grab an image from the camera
        camera.capture(image, format="bgr", use_video_port=camParams['use_video_port'])

        # and convert to greyscale
        image = image.reshape((camera.resolution[1], camera.resolution[0], 3))
        imageBW = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # AprilDetect
        tags = at_detector.detect(imageBW, True, camParams['cam_params'], args.tagSize/1000)

        # Tags in camera frame
        # Key is the tag ID
        tagDuplicatesT = {}
        tagDuplicatesPrev = {}

        # add any new tags to database, or existing one to duplicates
        for tag in tags:
            if tag.tag_id not in tagPlacement and tag.pose_err < 3e-7:
                # tag is in cur camera frame
                T_TagToCam = numpy.array( numpy.eye((4)) )
                T_TagToCam[0:3, 0:3] = numpy.array(tag.pose_R)
                tag.pose_t = numpy.array(tag.pose_t)
                T_TagToCam[0:3, 3] = tag.pose_t.reshape(3)
                tagPlacement[tag.tag_id] = T_CamToWorld @ T_TagToCam
                print("Added Tag ID {0}, Qual {2}, T =\n {1}".format(tag.tag_id, tagPlacement[tag.tag_id].round(3), tag.pose_err))
            elif tag.pose_err < 3e-7:
                # get tag's last pos, in camera frame
                T_TagToCam = numpy.array( numpy.eye((4)) )
                T_TagToCam[0:3, 0:3] = numpy.array(tag.pose_R)
                tag.pose_t = numpy.array(tag.pose_t)
                T_TagToCam[0:3, 3] = tag.pose_t.reshape(3)
                
                # save tag positions in Camera frame at time t for the duplicate
                tagDuplicatesT[tag.tag_id] = T_TagToCam
                # and t-1 for the original
                               
        # get the least cost transform from the common points at time t-1 to time t
        # cost is the sum of position error between the common points, with the t-1 points
        # projected forward to t
        if len(tagDuplicatesT) > 0:
            bestTransform = -1
            lowestCost = 999
            # Use each tag pair as a guess for the correct transform - lowest cost wins
            for tagid, tagT in tagDuplicatesT.items():
                # t is the time now, t-1 is the previous frame - where T_CamToWorld is at this point
                # tag is the same world position at both orig and duplicate
                # World frame is time-independent
                # RT(World)(Orig) = T(World <- Cam_t) * RT(Cam_t)(Duplicate)
                # and:
                # T(World <- Cam_t-1) = T(World <- Cam_t) * T(Cam_t <- Cam_t-1)
                #
                # Thus
                # T(World <- Cam_t) = T(World <- Cam_t-1) * T(Cam_t <- Cam_t-1)^-1
                # then
                # RT(World)(Orig) = T(World <- Cam_t-1) * T(Cam_t <- Cam_t-1)^-1 * RT(Cam_t)(Duplicate)
                # Thus
                # T(Cam_t <- Cam_t-1) = T(World <- Cam_t-1) * RT(Cam_t)(Duplicate) * RT(World)(Orig)^-1
                Ttprevtocur = T_CamToWorld @ tagT @ numpy.linalg.inv(tagPlacement[tagid])
                
                # and figure out summed distances between transformed new point to old (in world frame)
                summeddist = 0
                for tagidj, tagTj in tagDuplicatesT.items():
                    newpoint = (Ttprevtocur @ tagPlacement[tagidj]) - (T_CamToWorld @ tagTj)
                    summeddist += math.sqrt(math.pow(newpoint[0,3], 2) + math.pow(newpoint[1,3], 2) + math.pow(newpoint[2,3], 2))
                    
                print("Summed DeltaDist3D (Tag {1})= {0} cm".format(round(summeddist*100, 1), tagid))
                if lowestCost > summeddist:
                    lowestCost = summeddist
                    bestTransform = Ttprevtocur
                    
            #we have the lowest cost transform (need inverse)
            T_CamToWorld = numpy.linalg.inv(bestTransform) @ T_CamToWorld
            print("T_CamToWorld is\n{0}".format(T_CamToWorld.round(3)))
            print("Time to capture and detect = {0:.3f} sec, found {1} tags".format(time.time() - myStart, len(tags)))
                
                        
        #cv2.imwrite("detect_{0}.jpg".format(i), image)

