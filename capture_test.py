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
from pymavlink import mavutil
import threading
import json

from lib import cameraPi

class MAVLinkThread(threading.Thread):
    def __init__(self,mavstr, baudrate):
        self.isRunning = True
        self.mavstr = mavstr
        self.baudrate = baudrate
        self.mavstream = None
        # The attitude in the aeronautical frame (right-handed, Z-down, X-front, Y-right).
        self.attitude = [0,0,0]
        
        threading.Thread.__init__(self)

    def run(self):
        print("Initialising MAVLink stream")
        self.mavstream = mavutil.mavlink_connection(self.mavstr, baud=self.baudrate)
        self.mavstream.wait_heartbeat()
        print("Heartbeat from APM (system %u component %u)" % (self.mavstream.target_system, self.mavstream.target_system))
        while self.isRunning:
            msg = self.mavstream.recv_match(blocking=True)
            if msg.get_type() == "ATTITUDE":
                self.attitude = [msg.roll, msg.pitch, msg.yaw]
                #print(self.attitude)
                
        self.mavstream.close()

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("-camera", type=str, default="PiCamV2FullFoV", help="Camera profile in camera.yaml")
    parser.add_argument("-loop", type=int, default=20, help="Capture this many frames")
    parser.add_argument("-folder", type=str, default="capture", help="Put capture into this folder")
    parser.add_argument("-mavstring", type=str, default="/dev/ttySC1", help="MAVLink connection string")
    parser.add_argument("-mavversion", choices=['1', '2'], default="2", help="MAVLink Version (1, 2)")
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
    
    # Start MAVLink thread
    x = MAVLinkThread(args.mavstring, 921500)
    x.start()

    # initialize the camera
    camera = cameraPi.cameraPi(parameters[args.camera])
        
    print("Starting {0} image capture...".format(args.loop))
    
    # Init file
    rot_mapping = {}
    a_file = open(os.path.join(".", args.folder,'rot.json'), "w")
    json.dump(rot_mapping, a_file)
    a_file.close()

    for i in range(args.loop):
        myStart = time.time()

        imageBW = camera.getImage()

        # get time to capture and convert
        rot_mapping["capture_{:02d}.jpg".format(i)] = x.attitude
        print("Time to capture = {0:.0f}ms, att={1}".format((time.time() - myStart)*1000, x.attitude))

        # write image to file - don't time this
        cv2.imwrite(os.path.join(".", args.folder, "capture_{:02d}.jpg".format(i)),imageBW)
        
        # Write orientation from FC
        a_file = open(os.path.join(".", args.folder,'rot.json'), "w")
        json.dump(rot_mapping, a_file)
        a_file.close()

    # close camera
    camera.close()
    
    # Close MAvLink
    x.isRunning = False
    x.join()
