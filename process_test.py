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

from picamera import PiCamera
from apriltags3-py.apriltags3 import Detector


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("-camera", type=str, default="PiCamV2LowRes", help="Camera profile in camera.yaml")
    parser.add_argument("-loop", type=int, default=10, help="Process this many frames")
    parser.add_argument("-tagSize", type=int, default=160, help="Apriltag size in mm")
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
                           quad_decimate=2.0,
                           quad_sigma=0.0,
                           refine_edges=1,
                           decode_sharpening=0.25,
                           debug=0)
    
    # Current image
    image = numpy.empty((camera.resolution[0] * camera.resolution[1] * 3,),
                        dtype=numpy.uint8)

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

        # get time to capture and convert
        print("Time to capture and detect = {0:.3f} sec, found {1} tags".format(time.time() - myStart, len(tags)))

        # write image to file with tag details - don't time this
        for tag in tags:
            for idx in range(len(tag.corners)):
                cv2.line(image, tuple(tag.corners[idx-1, :].astype(int)), tuple(tag.corners[idx, :].astype(int)), (0, 255, 0))
            cv2.putText(image, str(tag.tag_id),
                        org=(tag.corners[0, 0].astype(int)+10,tag.corners[0, 1].astype(int)+10),
                        fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                        fontScale=0.8,
                        color=(0, 0, 255))
                        
            T_TagToCam = numpy.array( numpy.eye((4)) )
            T_TagToCam[0:3, 0:3] = numpy.array(tag.pose_R)
            tag.pose_t = numpy.array(tag.pose_t)
            T_TagToCam[0:3, 3] = tag.pose_t.reshape(3)
            
            print("Tag {0} pos = ({1}, {2}, {3})m".format(tag.tag_id, round(tag.pose_t[0][0], 3), round(tag.pose_t[1][0], 3), round(tag.pose_t[2][0], 3)))

        cv2.imwrite("detect_{0}.jpg".format(i), image)

