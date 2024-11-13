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

from pyapriltags import Detector
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
        self.pos = newPos
        self.rot = newRot
        self.pktSent = pktWasSent

    def run(self):
        while True:
            if len(self.lastFiveProTimes) > 0:
                fps = 1/mean(self.lastFiveProTimes)
            else:
                fps = 0
            print("Status: {0:.1f}fps, PosNED = {1}, PosRPY = {2}, Packets sent = {3}".format(
                fps, self.pos, self.rot, self.pktSent))
            if exit_event.wait(timeout=2):
                return


class mavThread(threading.Thread):
    def __init__(self, device, baud, source_system):
        threading.Thread.__init__(self)
        self.device = device
        self.baud = baud
        self.source_system = source_system
        self.heartbeatTimestamp = time.time()
        self.lock = threading.Lock()
        self.conn = None
        self.goodToSend = False
        self.reset_counter = 0
        self.pos = (0, 0, 0)
        self.speed = (0, 0, 0)
        self.rot = (0, 0, 0)
        self.time = 0
        self.pktSent = 0
        self.target_system = 1
        self.origin_lat = -35.363261
        self.origin_lon = 149.165230
        self.origin_alt = 0.001
        self.posDelta = (0, 0, 0)
        self.rotDelta = (0, 0, 0)

    def updateData(self, newPos, newRot, t, posDelta, rotDelta):
        with self.lock:
            if self.time != 0:
                # time is in usec here, remember to convert to sec
                self.speed = numpy.array(posDelta)/ (1E-6 * (t - self.time))
            self.pos = newPos
            self.rot = newRot
            self.time = t
            self.posDelta = posDelta
            self.rotDelta = rotDelta

    def run(self):
        # Start mavlink connection
        try:
            self.conn = mavutil.mavlink_connection(self.device, autoreconnect=True, source_system=self.source_system,
                                                   baud=self.baud, force_connected=False,
                                                   source_component=mavutil.mavlink.MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY)
        except Exception as msg:
            print("Failed to start mavlink connection on %s: %s" %
                  (self.device, msg,))
            raise

        # wait for the heartbeat msg to find the system ID. Need to exit from here too
        # We are sending a heartbeat signal too, to allow ardupilot to init the comms channel
        while True:
            self.sendHeartbeat()
            if self.conn.wait_heartbeat(timeout=0.5) is not None:
                # Got a hearbeart, go to next loop
                self.goodToSend = True
                break
            if exit_event.is_set():
                return

        print("Got Heartbeat from APM (system %u component %u)" %
              (self.conn.target_system, self.conn.target_system))
        self.send_msg_to_gcs("Starting")

        while True:
            # self.conn.recv_match(blocking=True, timeout=0.5)
            # loop at 20 Hz
            time.sleep(0.05)
            self.sendPos()
            self.sendSpeed()
            # self.sendPosDelta()
            self.sendHeartbeat()
            if exit_event.is_set():
                self.send_msg_to_gcs("Stopping")
                return
            # check if we need to send the EKF origin
            msg = self.conn.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
            if msg and msg.lat == 0 and msg.lon == 0:
                print("Setting EKF origin")
                self.set_default_global_origin()

    def sendHeartbeat(self):
        # send heartbeat if more than 1 sec since last message
        if (self.heartbeatTimestamp + 1) < time.time():
            self.conn.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                                         mavutil.mavlink.MAV_AUTOPILOT_GENERIC,
                                         0,
                                         0,
                                         0)
            self.heartbeatTimestamp = time.time()

    def getPktSent(self):
        with self.lock:
            return self.pktSent

    # https://mavlink.io/en/messages/common.html#STATUSTEXT
    def send_msg_to_gcs(self, text_to_be_sent):
        # MAV_SEVERITY: 0=EMERGENCY 1=ALERT 2=CRITICAL 3=ERROR, 4=WARNING, 5=NOTICE, 6=INFO, 7=DEBUG, 8=ENUM_END
        text_msg = 'AprilMAV: ' + text_to_be_sent
        self.conn.mav.statustext_send(
            mavutil.mavlink.MAV_SEVERITY_INFO, text_msg.encode())

    # Send a mavlink SET_GPS_GLOBAL_ORIGIN message (http://mavlink.org/messages/common#SET_GPS_GLOBAL_ORIGIN), for the EKF origin

    def set_default_global_origin(self):
        current_time_us = int(round(time.time() * 1000000))
        self.conn.mav.set_gps_global_origin_send(self.target_system,
                                                 int(self.origin_lat*1.0e7),
                                                 int(self.origin_lon*1.0e7),
                                                 int(self.origin_alt*1.0e3),
                                                 current_time_us)

    def sendPos(self):
        # Send a vision pos estimate
        # https://mavlink.io/en/messages/common.html#VISION_POSITION_ESTIMATE
        # if self.getTimestamp() > 0:
        if self.goodToSend:
            current_time_us = int(round(time.time() * 1000000))
            # estimate error - approx 0.01m in pos and 0.5deg in angle
            cov_pose = 0.01
            cov_twist = 0.5
            covariance = numpy.array([cov_pose, 0, 0, 0, 0, 0,
                                      cov_pose, 0, 0, 0, 0,
                                      cov_pose, 0, 0, 0,
                                      cov_twist, 0, 0,
                                      cov_twist, 0,
                                      cov_twist])
            with self.lock:
                self.conn.mav.vision_position_estimate_send(
                    current_time_us, self.pos[0], self.pos[1], self.pos[2], self.rot[0], self.rot[1], self.rot[2],
                    covariance, reset_counter=self.reset_counter)
                self.pktSent += 1

    def sendSpeed(self):
        # Send a vision speed estimate
        # https://mavlink.io/en/messages/common.html#VISION_SPEED_ESTIMATE
        if self.goodToSend:
            current_time_us = int(round(time.time() * 1000000))
            # estimate error - approx 0.05m/s in pos
            cov_pose = 0.05
            covariance = numpy.array([cov_pose, 0, 0,
                                      0, cov_pose, 0,
                                      0, 0, cov_pose])
            with self.lock:
                self.conn.mav.vision_speed_estimate_send(
                    current_time_us, self.speed[0], self.speed[1], self.speed[2], covariance,
                    reset_counter=self.reset_counter)
                self.pktSent += 1

    def sendPosDelta(self):
        # Send a vision pos delta
        # https://mavlink.io/en/messages/ardupilotmega.html#VISION_POSITION_DELTA
        if self.goodToSend:
            with self.lock:
                current_time_us = int(round(time.time() * 1000000))
                delta_time_us = current_time_us - self.time
                current_confidence_level = 80

                # Send the message
                self.conn.mav.vision_position_delta_send(
                    current_time_us,    # us: Timestamp (UNIX time or time since system boot)
                    delta_time_us,	    # us: Time since last reported camera frame
                    self.rotDelta,    # float[3] in radian: Defines a rotation vector in body frame that rotates the vehicle from the previous to the current orientation
                    self.posDelta,   # float[3] in m: Change in position from previous to current frame rotated into body frame (0=forward, 1=right, 2=down)
                    current_confidence_level  # Normalized confidence value from 0 to 100.
                )
                self.pktSent += 1


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--tagSize", type=int, default=93,
                        help="Apriltag size in mm")
    parser.add_argument("--camera", type=str, default="GenericUSB",
                        help="Camera profile in camera.yaml")
    parser.add_argument("--maxerror", type=int, default=400,
                        help="Maximum pose error to use, in n*E-8 units")
    parser.add_argument("--outfile", type=str, default="geo_test_results.csv",
                        help="Output tag data to this file")
    parser.add_argument(
        "--device", type=str, default="udpin:127.0.0.1:14550", help="MAVLink connection string")
    parser.add_argument("--baud", type=int, default=115200,
                        help="MAVLink baud rate, if using serial")
    parser.add_argument("--source-system", type=int,
                        default=1, help="MAVLink Source system")
    parser.add_argument("--imageFolder", type=str, default="",
                        help="Save processed images to this folder")
    parser.add_argument("--video", type=int, default=0,
                        help="Output video to port, 0 to disable")
    parser.add_argument("--decimation", type=int,
                        default=2, help="Apriltag decimation")
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
                           nthreads=4,
                           quad_decimate=args.decimation,
                           quad_sigma=0.4,
                           refine_edges=1,
                           decode_sharpening=1,
                           debug=0)

    # All tags live in here
    tagPlacement = tagDB(False)

    outfile = open(args.outfile, "w+")
    # left, up, fwd, pitch, yaw, roll
    outfile.write("{0},{1},{2},{3},{4},{5},{6}\n".format(
        "Filename", "PosX (m)", "PosY (m)", "PosZ (m)", "RotX (rad)", "RotY (rad)", "RotZ (rad)"))

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
    threadMavlink = mavThread(args.device, args.baud, args.source_system)
    threadMavlink.start()

    # Start Status thread
    threadStatus = statusThread()
    threadStatus.start()

    # Start save image thread, if desired
    threadSave = None
    if args.imageFolder != "":
        threadSave = saveThread(args.imageFolder, exit_event)
        threadSave.start()

    # video stream out, if desired
    threadVideo = None
    if args.video != 0:
        threadVideo = videoThread(args.video, exit_event)
        threadVideo.start()

    i = 0
    while True:
        # print("--------------------------------------")

        myStart = time.time()

        # grab an image (and timestamp in usec) from the camera
        # estimate 50usec from timestamp to frame capture on next line
        timestamp = int(round(time.time() * 1000000)) + 50
        file = camera.getFileName()
        # print("Timestamp of capture = {0}".format(timestamp))
        imageBW = camera.getImage()
        i += 1

        # we're out of images
        if imageBW is None:
            break

        # AprilDetect, after accounting for distortion (if fisheye)
        if camParams['fisheye'] and dim1 is None:
            # Only need to get mapping at first frame
            # dim1 is the dimension of input image to un-distort
            dim1 = imageBW.shape[:2][::-1]
            map1, map2 = cv2.fisheye.initUndistortRectifyMap(
                K, D, numpy.eye(3), K, dim1, cv2.CV_16SC2)
        if camParams['fisheye']:
            imageBW = cv2.remap(
                imageBW, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
            # tags = at_detector.detect(undistorted_img, True, camParams['cam_params'], args.tagSize/1000)
        # else:
        tags = at_detector.detect(
            imageBW, True, camParams['cam_params'], args.tagSize/1000)

        # add any new tags to database, or existing one to duplicates
        tagsused = 0
        for tag in tags:
            if tag.pose_err < args.maxerror*1e-8:
                tagsused += 1
                tagPlacement.addTag(tag)

        tagPlacement.getBestTransform()

        if file:
            print("File: {0}".format(file))

        # get current location and rotation state of vehicle in ArduPilot NED format (rel camera)
        (posD, rotD) = tagPlacement.getArduPilotNED()
        (posR, rotR) = tagPlacement.getArduPilotNED(radians=True)
        (posRDelta, rotRDelta) = tagPlacement.getArduPilotNEDDelta(radians=True)

        outfile.write("{0},{1:.3f},{2:.3f},{3:.3f},{4:.1f},{5:.1f},{6:.1f}\n".format(
            file, posR[0], posR[1], posR[2], rotR[0], rotR[1], rotR[2]))

        # print("Time to capture, detect and localise = {0:.3f} sec, using {2}/{1} tags".format(time.time() - myStart, len(tags), len(tagPlacement.tagDuplicatesT)))

        # Create and send MAVLink packet
        threadMavlink.updateData(posR, rotR, timestamp, posRDelta, rotRDelta)
        # wasSent = threadMavlink.sendPos(posR[0], posR[1], posR[2], rotR[0], rotR[1], rotR[2], timestamp)

        # Send to status thread
        threadStatus.updateData(time.time(
        ) - myStart, (posD[0], posD[1], posD[2]), (rotD[0], rotD[1], rotD[2]), threadMavlink.getPktSent())

        # Send to save thread
        if threadSave:
            threadSave.save_queue.put((imageBW, os.path.join(
                ".", args.imageFolder, "processed_{:04d}.jpg".format(i)), posD, rotD, tags))

        # Get ready for next frame
        tagPlacement.newFrame()

        # Send to video stream, if option
        if threadVideo:
            threadVideo.frame_queue.put((imageBW, posD, rotD, tags))

        if exit_event.is_set():
            break
