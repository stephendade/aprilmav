#!/usr/bin/env python3
'''
Main script. Will use Apriltags to localise position and send via mavlink

'''
import time
from importlib import import_module
from statistics import mean
from collections import deque
import argparse
import threading
import signal
import sys
import os

import yaml
import numpy
import cv2

from dt_apriltags import Detector
from pymavlink import mavutil

from lib.geo import tagDB
from lib.videoStream import videoThread
from lib.saveStream import saveThread

exit_event = threading.Event()

def signal_handler(signum, frame):
    exit_event.set()

class statusThread(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.lastFiveProTimes = deque(maxlen=5)
        self.pos = (0, 0, 0)
        self.rot = (0, 0, 0)
        self.pktSent = 0

    def updateData(self, proTime, newPos, newRot, pktWasSent):
        self.lastFiveProTimes.append(proTime)
        # left, up, fwd, pitch, yaw, roll ---> fwd, -left, -up, roll, -pitch, -yaw
        #z, -x, -y, rz, -rx, -ry
        self.pos = (newPos[2], -newPos[0], -newPos[1])
        self.rot = numpy.rad2deg((newRot[2], -newRot[0], -newRot[1]))
        if pktWasSent:
            self.pktSent += 1

    def run(self):
        while(True):
            if len(self.lastFiveProTimes) > 0:
                fps = 1/mean(self.lastFiveProTimes)
            else:
                fps = 0
            print("Status: {0:.1f}fps, PosNED = {1}, PosRPY = {2}, Packets sent = {3}".format(fps, self.pos, self.rot, self.pktSent))
            if exit_event.wait(timeout=2):
                return

class mavThread(threading.Thread):
    def __init__(self, device, baud, source_system):
        threading.Thread.__init__(self)
        self.device = device
        self.baud = baud
        self.source_system = source_system
        self.timestamp = 0
        self.lock = threading.Lock()
        self.conn = None
        self.goodToSend = False
        self.reset_counter = True
        
    def run(self):
        # Start mavlink connection
        try:
            self.conn = mavutil.mavlink_connection(self.device, autoreconnect=True, source_system=self.source_system,
                                                   baud=self.baud, force_connected=False)
        except Exception as msg:
            print("Failed to start mavlink connection on %s: %s" % (self.device, msg,))
            raise
            
        # wait for the heartbeat msg to find the system ID. Need to exit from here too
        while True:
            if self.conn.wait_heartbeat(timeout=0.5) != None:
                # Got a hearbeart, go to next loop
                self.goodToSend = True
                break
            if exit_event.is_set():
                return
        
        print("Got Heartbeat from APM (system %u component %u)" % (self.conn.target_system, self.conn.target_system))
        
        while not exit_event.is_set():
            msg = self.conn.recv_match(blocking=True, timeout=0.5)
            if msg != None:
                if self.conn.timestamp < self.timestamp:
                    print("Reset timestamp")
                    with self.lock:
                        self.timestamp = self.conn.timestamp
            
    def getTimestamp(self):
        with self.lock:
            return self.timestamp
            
    def sendPos(self, x, y, z, rx, ry, rz, t):
        # Send a vision pos estimate
        # https://mavlink.io/en/messages/common.html#VISION_POSITION_ESTIMATE
        # ArduPilot Frame is NED - so need to convert from AprilMAV's (left, up, fwd)
        # left, up, fwd, pitch, yaw, roll ---> fwd, -left, -up, roll, -pitch, -yaw 
        #if self.getTimestamp() > 0:
        if self.goodToSend:
            # estimate error - approx 0.005 in pos and 2 in angle
            #posErr = cbrtf(sq(covariance[0])+sq(covariance[6])+sq(covariance[11]));
            #angErr = cbrtf(sq(covariance[15])+sq(covariance[18])+sq(covariance[20]));
            self.conn.mav.vision_position_estimate_send(t, z, -x, -y, rz, -rx, -ry, covariance=[0.005, 0, 0, 0, 0, 0, 0.005, 0, 0, 0, 0, 0.005, 0, 0, 0, 2, 0, 0, 2, 0, 2], reset_counter=self.reset_counter)
            #Reset counter only needs be be true for first sent packet
            self.reset_counter = False
            return True
        else:
            return False
        #else:
        #    #print("Can't send")
        #    return False

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--tagSize", type=int, default=96, help="Apriltag size in mm")
    parser.add_argument("--camera", type=str, default="GenericUSB", help="Camera profile in camera.yaml")
    parser.add_argument("--maxerror", type=int, default=400, help="Maximum pose error to use, in n*E-8 units")
    parser.add_argument("--outfile", type=str, default="geo_test_results.csv", help="Output tag data to this file")
    parser.add_argument("--device", type=str, default="udpin:127.0.0.1:14550", help="MAVLink connection string")
    parser.add_argument("--baud", type=int, default=115200, help="MAVLink baud rate, if using serial")
    parser.add_argument("--source-system", type=int, default=255, help="MAVLink Source system")
    parser.add_argument("--imageFolder", type=str, default="", help="Save processed images to this folder")
    parser.add_argument("--video", type=int, default=5000, help="Output video to port, 0 to disable")
    args = parser.parse_args()
    
    print("Initialising")

    # Open camera settings/
    with open('camera.yaml', 'r') as stream:
        parameters = yaml.load(stream, Loader=yaml.FullLoader)
    camParams = parameters[args.camera]
        
    # initialize the camera
    camera = None
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
                           quad_decimate=4.0,
                           quad_sigma=0.4,
                           refine_edges=1,
                           decode_sharpening=1,
                           debug=0)

    # All tags live in here
    tagPlacement = tagDB(0, 0, 0, False)
    
    outfile = open(args.outfile, "w+")
    # left, up, fwd, pitch, yaw, roll
    outfile.write("{0},{1},{2},{3},{4},{5},{6}\n".format("Filename", "PosX (m)", "PosY (m)", "PosZ (m)", "RotX (rad)", "RotY (rad)", "RotZ (rad)"))

    # Need to reconstruct K and D if using fisheye lens
    if camParams['fisheye']:
        K = numpy.zeros((3, 3))
        D = numpy.zeros((4, 1))
        K[0, 0] = camParams['cam_params'][0]
        K[1, 1] = camParams['cam_params'][1]
        K[0, 2] = camParams['cam_params'][2]
        K[1, 2] = camParams['cam_params'][3]
        K[2, 2] = 1
        D[0][0] = camParams['cam_paramsD'][0]
        D[1][0] = camParams['cam_paramsD'][1]
        D[2][0] = camParams['cam_paramsD'][2]
        D[3][0] = camParams['cam_paramsD'][3]
        
        dim1 = None
              
    signal.signal(signal.SIGINT, signal_handler)
    
    # Start MAVLink comms thread
    thread1 = mavThread(args.device, args.baud, args.source_system)
    thread1.start()
    
    # Start Status thread
    threadStatus = statusThread()
    threadStatus.start()
    
    # Start save image thread, if desired
    threadSave = None
    if args.imageFolder != "":
        threadSave = saveThread(args.imageFolder, exit_event)
        threadSave.start()    
      
    # video stream out  
    threadVideo = None
    if args.video != 0:
        threadVideo = videoThread(args.video, exit_event)
        threadVideo.start()  
        
    i = 0
    while True:
        #print("--------------------------------------")
        
        myStart = time.time()

        # grab an image (and timestamp) from the camera
        file = camera.getFileName()
        timestamp = int(round(time.time() * 1000000))
        #print("Timestamp of capture = {0}".format(timestamp))
        imageBW = camera.getImage()
        i += 1
        
        # we're out of images
        if imageBW is None:
            break
            
        # AprilDetect, after accounting for distortion (if fisheye)
        if camParams['fisheye'] and dim1 is None:
            #Only need to get mapping at first frame
            dim1 = imageBW.shape[:2][::-1]  #dim1 is the dimension of input image to un-distort
            map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, numpy.eye(3), K, dim1, cv2.CV_16SC2) 
        if camParams['fisheye']:
            undistorted_img = cv2.remap(imageBW, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)      
            tags = at_detector.detect(undistorted_img, True, camParams['cam_params'], args.tagSize/1000)
        else:
            tags = at_detector.detect(imageBW, True, camParams['cam_params'], args.tagSize/1000)

        # add any new tags to database, or existing one to duplicates
        tagsused = 0
        for tag in tags:
            if tag.pose_err < args.maxerror*1e-8:
                tagsused += 1
                tagPlacement.addTag(tag)
                              
        tagPlacement.getBestTransform()

        if file:
            print("File: {0}".format(file))
        
        posn = tagPlacement.getCurrentPosition()
        rot = tagPlacement.getCurrentRotation(radians=True)
        rotDeg = tagPlacement.getCurrentRotation()
        outfile.write("{0},{1:.3f},{2:.3f},{3:.3f},{4:.1f},{5:.1f},{6:.1f}\n".format(file, posn[0], posn[1], posn[2], rot[0], rot[1], rot[2]))

        #print("Time to capture, detect and localise = {0:.3f} sec, using {2}/{1} tags".format(time.time() - myStart, len(tags), len(tagPlacement.tagDuplicatesT)))
        
        
        # Create and send MAVLink packet
        wasSent = thread1.sendPos(posn[0], posn[1], posn[2], rot[0], rot[1], rot[2], timestamp)
        
        # Send to status thread
        threadStatus.updateData(time.time() - myStart, (posn[0], posn[1], posn[2]), (rotDeg[0], rotDeg[1], rotDeg[2]), wasSent)
        
        # Send to save thread
        if threadSave:
            threadSave.save_queue.put((imageBW, os.path.join(".", args.imageFolder, "processed_{:04d}.jpg".format(i)), posn, rotDeg, tags))
        
        # Get ready for next frame
        tagPlacement.newFrame()
        
        # Send to video stream, if option
        if threadVideo:
            threadVideo.frame_queue.put((imageBW, posn, rotDeg, tags))

        if exit_event.is_set():
            break



