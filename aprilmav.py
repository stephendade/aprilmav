#!/usr/bin/env python3
'''
Main script. Will use Apriltags to localise position and send via mavlink

'''
import time
import numpy
import cv2
import yaml
import argparse
import threading
import signal
import queue
import os
from collections import deque
from statistics import mean

from importlib import import_module
from dt_apriltags import Detector
from lib.geo import tagDB
from pymavlink import mavutil

exit_event = threading.Event()

def signal_handler(signum, frame):
    exit_event.set()


# Separate thread for saving images, in order to not delay image capture
class saveThread(threading.Thread):
    def __init__(self, folder):
        threading.Thread.__init__(self)
        self.save_queue = queue.Queue()
        
        #create the capture folder if required
        try:
            os.makedirs(os.path.join(".", folder))
        except FileExistsError:
            pass
        
    def run(self):
        while True:
            if self.save_queue.empty():
                continue
            (image, filename, posn, rot, tags) = self.save_queue.get()
            # add in data (colour)
            imageColour = cv2.cvtColor(image,cv2.COLOR_GRAY2RGB)
            #outfile.write("{0},{1:.3f},{2:.3f},{3:.3f},{4:.1f},{5:.1f},{6:.1f}\n".format(file, posn[0], posn[1], posn[2], rot[0], rot[1], rot[2]))
            cv2.putText(imageColour, "Pos (m) = {0:.3f}, {1:.3f}, {2:.3f}".format(posn[0], posn[1], posn[2]), (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
            cv2.putText(imageColour, "Rot (deg) = {0:.1f}, {1:.1f}, {2:.1f}".format(rot[0], rot[1], rot[2]), (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
            imageColour = self.labelTags(imageColour, tags)
            cv2.imwrite(filename, imageColour, [cv2.IMWRITE_JPEG_QUALITY, 99])

            #print("Saved {0}".format(filename))
            if exit_event.wait(timeout=0.01):
                return
                
    def labelTags(self, image, tags):
        # Label the tags in the image
        # loop over the AprilTag detection results
        for r in tags:
	        # extract the bounding box (x, y)-coordinates for the AprilTag
	        # and convert each of the (x, y)-coordinate pairs to integers
	        (ptA, ptB, ptC, ptD) = r.corners
	        ptB = (int(ptB[0]), int(ptB[1]))
	        ptC = (int(ptC[0]), int(ptC[1]))
	        ptD = (int(ptD[0]), int(ptD[1]))
	        ptA = (int(ptA[0]), int(ptA[1]))
	        # draw the bounding box of the AprilTag detection
	        cv2.line(image, ptA, ptB, (0, 255, 0), 2)
	        cv2.line(image, ptB, ptC, (0, 255, 0), 2)
	        cv2.line(image, ptC, ptD, (0, 255, 0), 2)
	        cv2.line(image, ptD, ptA, (0, 255, 0), 2)
	        # draw the center (x, y)-coordinates of the AprilTag
	        (cX, cY) = (int(r.center[0]), int(r.center[1]))
	        cv2.circle(image, (cX, cY), 5, (0, 0, 255), -1)
	        # draw the tag ID 
	        cv2.putText(image, str(r.tag_id), (ptA[0] + 10, ptA[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 1)
	        return image
    
class statusThread(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.lastFiveProTimes = deque(maxlen=5)
        self.pos = (0,0,0)
        self.pktSent = 0
        
    def updateData(self, proTime, newPos, pktWasSent):
        self.lastFiveProTimes.append(proTime)
        self.pos = newPos
        if pktWasSent:
            self.pktSent += 1
            
    def run(self):
        while(True):
            if len(self.lastFiveProTimes) > 0:
                fps = 1/mean(self.lastFiveProTimes)
            else:
                fps = 0
            print("Status: {0:.1f}fps, Pos = {1}, Packets sent = {2}".format(fps, self.pos, self.pktSent))
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
            msg = self.conn.recv_match(blocking=True,timeout=0.5)
            if msg != None:
                if self.conn.timestamp < self.timestamp:
                    print("Reset timestamp")
                    with self.lock:
                        self.timestamp = self.conn.timestamp
            
    def getTimestamp(self):
        with self.lock:
            return self.timestamp
            
    def sendPos(self, x, y, z, roll, pitch, yaw, t):
        # Send a vision pos estimate
        # https://mavlink.io/en/messages/common.html#VISION_POSITION_ESTIMATE
        # ArduPilot Frame is NED - so need to convert from AprilMAV's (left, up, fwd)
        #if self.getTimestamp() > 0:
        if self.goodToSend:
            # estimate error - approx 0.005 in pos and 2 in angle
            #posErr = cbrtf(sq(covariance[0])+sq(covariance[6])+sq(covariance[11]));
            #angErr = cbrtf(sq(covariance[15])+sq(covariance[18])+sq(covariance[20]));
            self.conn.mav.vision_position_estimate_send(t, z, -x, -y, roll, -pitch, -yaw, covariance=[0.005,0,0,0,0,0,0.005,0,0,0,0,0.005,0,0,0,2,0,0,2,0,2], reset_counter=0)
            return True
        else:
            return False
        #else:
        #    #print("Can't send")
        #    return False

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("-tagSize", type=int, default=96, help="Apriltag size in mm")
    parser.add_argument("-camera", type=str, default="GenericUSB", help="Camera profile in camera.yaml")
    parser.add_argument("-maxerror", type=int, default=400, help="Maximum pose error to use, in n*E-8 units")
    parser.add_argument("-outfile", type=str, default="geo_test_results.csv", help="Output tag data to this file")
    parser.add_argument("-device", type=str, default="udpin:127.0.0.1:14550", help="MAVLink connection string")
    parser.add_argument("-baud", type=int, default=115200, help="MAVLink baud rate, if using serial")
    parser.add_argument("-source-system", type=int, default=255, help="MAVLink Source system")
    parser.add_argument("-imageFolder", type=str, default="", help="Save processed images to this folder")
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
    
    outfile = open(args.outfile,"w+")
    outfile.write("{0},{1},{2},{3},{4},{5},{6}\n".format("Filename", "PosX (left)", "PosY (up)", "PosZ (fwd)", "RotX", "RotY", "RotZ"))

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
        threadSave = saveThread(args.imageFolder)
        threadSave.start()        
        
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
        if camParams['fisheye']:
            dim1 = imageBW.shape[:2][::-1]  #dim1 is the dimension of input image to un-distort
            map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, numpy.eye(3), K, dim1, cv2.CV_16SC2)
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
        rot = tagPlacement.getCurrentRotation()
        outfile.write("{0},{1:.3f},{2:.3f},{3:.3f},{4:.1f},{5:.1f},{6:.1f}\n".format(file, posn[0], posn[1], posn[2], rot[0], rot[1], rot[2]))

        #print("Time to capture, detect and localise = {0:.3f} sec, using {2}/{1} tags".format(time.time() - myStart, len(tags), len(tagPlacement.tagDuplicatesT)))
        
        
        # Create and send MAVLink packet
        wasSent = thread1.sendPos(posn[0], posn[1], posn[2], rot[0], rot[1], rot[2], timestamp)
        
        # Send to status thread
        threadStatus.updateData(time.time() - myStart, (posn[0], posn[1], posn[2]), wasSent)
        
        # Send to save thread
        if threadSave:
            threadSave.save_queue.put((imageBW, os.path.join(".", args.imageFolder, "processed_{:04d}.jpg".format(i)), posn, rot, tags))
        
        # Get ready for next frame
        tagPlacement.newFrame()
        
        if exit_event.is_set():
            break



