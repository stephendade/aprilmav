#!/usr/bin/env python3
'''
Main script. Will use Apriltags to localise position and send via mavlink

'''
import time
from statistics import mean
from collections import deque
import argparse
import threading
import signal
import os

import numpy

from pymavlink import mavutil

from modules.common import do_multi_capture_detection, loadCameras, tryCheckCuda
from modules.geo import tagDB
from modules.videoStream import videoThread
from modules.saveStream import saveThread

exit_event = threading.Event()


def signal_handler(signum, frame):
    """
    Signal handler for exit
    """
    exit_event.set()


class statusThread(threading.Thread):
    """
    Thread for printing performance stats to console
    """
    def __init__(self):
        threading.Thread.__init__(self)
        self.lastFiveProTimes = deque(maxlen=5)
        self.pos = (0, 0, 0)
        self.rot = (0, 0, 0)
        self.pktSent = 0

    def updateData(self, proTime, newPos, newRot, pktWasSent):
        '''Sync data with thread'''
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
    '''Thread to handle all MAVLink communications with flight controller'''
    def __init__(self, device, baud, source_system):
        threading.Thread.__init__(self)
        self.device = device
        self.baud = baud
        self.source_system = source_system
        self.lock = threading.Lock()
        self.conn = None
        self.goodToSend = False
        self.reset_counter = 0
        self.pktSent = 0
        self.target_system = 1
        self.origin_lat = -35.363261
        self.origin_lon = 149.165230
        self.origin_alt = 0.01

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
            # loop at 1 Hz
            time.sleep(1)
            with self.lock:
                self.sendHeartbeat()
                if exit_event.is_set():
                    self.send_msg_to_gcs("Stopping")
                    return
                # check if we need to send the EKF origin
                msg = self.conn.recv_match(
                    type='GLOBAL_POSITION_INT', blocking=False)
                if msg and msg.lat == 0 and msg.lon == 0:
                    print("Setting EKF origin")
                    self.set_default_global_origin()

    def sendHeartbeat(self):
        '''send heartbeat if more than 1 sec since last message'''
        self.conn.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                                     mavutil.mavlink.MAV_AUTOPILOT_GENERIC,
                                     0,
                                     0,
                                     0)

    def getPktSent(self):
        '''Return the last packet send'''
        with self.lock:
            return self.pktSent

    def send_msg_to_gcs(self, text_to_be_sent):
        '''Send a STATUSTEXT message to the GCS
        https://mavlink.io/en/messages/common.html#STATUSTEXT
        '''
        # MAV_SEVERITY: 0=EMERGENCY 1=ALERT 2=CRITICAL 3=ERROR, 4=WARNING, 5=NOTICE, 6=INFO, 7=DEBUG, 8=ENUM_END
        text_msg = 'AprilMAV: ' + text_to_be_sent
        self.conn.mav.statustext_send(
            mavutil.mavlink.MAV_SEVERITY_INFO, text_msg.encode())

    def set_default_global_origin(self):
        '''Send a mavlink SET_GPS_GLOBAL_ORIGIN message for the EKF origin
        http://mavlink.org/messages/common#SET_GPS_GLOBAL_ORIGIN
        '''
        current_time_us = int(round(time.time() * 1000000))
        self.conn.mav.set_gps_global_origin_send(self.target_system,
                                                 int(self.origin_lat*1.0e7),
                                                 int(self.origin_lon*1.0e7),
                                                 int(self.origin_alt*1.0e3),
                                                 current_time_us)

    def sendPos(self, pos, rot):
        '''Send a vision pos estimate
        https://mavlink.io/en/messages/common.html#VISION_POSITION_ESTIMATE
        '''
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
                    current_time_us, pos[0], pos[1], pos[2], rot[0], rot[1], rot[2],
                    covariance, reset_counter=self.reset_counter)
                self.pktSent += 1

    def sendSpeed(self, vehSpeed):
        '''Send a vision speed estimate
        https://mavlink.io/en/messages/common.html#VISION_SPEED_ESTIMATE
        '''
        if self.goodToSend:
            current_time_us = int(round(time.time() * 1000000))
            # estimate error - approx 0.05m/s in pos
            cov_pose = 0.05
            covariance = numpy.array([cov_pose, 0, 0,
                                      0, cov_pose, 0,
                                      0, 0, cov_pose])
            with self.lock:
                self.conn.mav.vision_speed_estimate_send(
                    current_time_us, vehSpeed[0], vehSpeed[1], vehSpeed[2], covariance,
                    reset_counter=self.reset_counter)
                self.pktSent += 1


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--tagSize", type=int, default=94,
                        help="Apriltag size in mm")
    parser.add_argument("--camera", type=str, default="GenericUSB",
                        help="Camera profile in camera.yaml")
    parser.add_argument("--maxError", type=int, default=400,
                        help="Maximum pose error to use, in n*E-8 units")
    parser.add_argument("--outFile", type=str, default="geo_test_results.csv",
                        help="Output tag data to this file")
    parser.add_argument(
        "--device", type=str, default="udpin:127.0.0.1:14550", help="MAVLink connection string")
    parser.add_argument("--baud", type=int, default=115200,
                        help="MAVLink baud rate, if using serial")
    parser.add_argument("--source-system", type=int,
                        default=1, help="MAVLink Source system")
    parser.add_argument("--outputFolder", type=str, default="",
                        help="Save processed images to this folder")
    parser.add_argument("--video", type=str, default='',
                        help="Output video to IP:port")
    parser.add_argument("--decimation", type=int,
                        default=2, help="Apriltag decimation")
    parser.add_argument("--outliers", type=int,
                        default=5, help="Reject any outlier positions, based on last N frames")
    parser.add_argument('--extraOpt', dest='extraOpt', help="Optimise best position better",
                        default=False, action='store_true')
    parser.add_argument('--cuda', dest='cuda', help="Use OpenCV CUDA Extensions",
                        default=False, action='store_true')
    parser.add_argument("--tagFamily", type=str, default="tagStandard41h12",
                        help="Apriltag family")
    parser.add_argument("--multiCamera", type=str, default=None,
                        help="multiple cameras using the specified yaml file")
    parser.add_argument('--tagEngine', dest='tagEngine', help="Tag detector engine",
                        default='PyAprilTags', choices=['OpenCV', 'PyAprilTags', 'JetsonPVA'])
    parser.add_argument('--R', type=float, default=0.15,
                        help="EKF measurement uncertainty, in m")
    parser.add_argument('--Ppos', type=float, default=0.01,
                        help="EKF position uncertainty, in m")
    parser.add_argument('--PVel', type=float, default=0.3,
                        help="EKF velocity uncertainty, in m/s")
    parser.add_argument('--PAccel', type=float, default=2,
                        help="EKF acceleration uncertainty, in m/s^2")
    args = parser.parse_args()

    print("Initialising")

    tryCheckCuda(args.cuda)

    # Open camera settings and load camera(s)
    CAMERAS = loadCameras(args.multiCamera, args.camera, None, args.cuda,
                          args.tagSize, args.tagFamily, args.decimation, args.tagEngine)

    # allow the camera to warmup
    time.sleep(2)

    # All tags live in here
    tagPlacement = tagDB(slidingWindow=args.outliers, extraOpt=args.extraOpt, R=args.R,
                         Ppos=args.Ppos, PVel=args.PVel, PAccel=args.PAccel)

    # left, up, fwd, pitch, yaw, roll
    with open(args.outFile, "w+", encoding="utf-8") as outFile:
        outFile.write("Filename,Timestamp,")
        outFile.write("PosX (m),PosY (m),PosZ (m),")
        outFile.write("RotX (rad),RotY (rad),RotZ (rad),")
        outFile.write("VelX (m/s),VelY (m/s), VelZ (m/s)\n")
    # Need to reconstruct K and D if using fisheye lens
    dim1 = None
    map1 = None
    map2 = None

    signal.signal(signal.SIGINT, signal_handler)

    # Start MAVLink comms thread
    threadMavlink = mavThread(args.device, args.baud, args.source_system)
    threadMavlink.start()

    # Start Status thread
    threadStatus = statusThread()
    threadStatus.start()

    # Start save image thread, if desired
    threadSave = None
    threadSave_prev_timestamp = 0
    i = 0
    if args.outputFolder != "":
        threadSave = saveThread(args.outputFolder, exit_event, CAMERAS, compression=3)
        threadSave.start()

    # video stream out, if desired
    threadVideo = None
    if args.video != '':
        threadVideo = videoThread(args.video, exit_event)
        threadVideo.start()

    while True:
        # Capture images from all cameras (in parallel)
        do_multi_capture_detection(CAMERAS, False, True)
        # check for any bad captures
        shouldExit = False
        for CAMERA in CAMERAS:
            if CAMERA.imageBW is None:
                print("Bad capture from {0}. Exiting...".format(CAMERA.camName))
                shouldExit = True
        if shouldExit:
            break
        timestamp = numpy.mean([CAMERA.image_timestamp for CAMERA in CAMERAS])

        # feed tags into tagPlacement
        for CAMERA in CAMERAS:
            for tag in CAMERA.tags:
                if tag.pose_err < args.maxError*1e-8:
                    tagPlacement.addTag(tag, CAMERA.T_CamtoVeh)

        tagPlacement.getBestTransform(timestamp)

        # get current location and rotation state of vehicle in ArduPilot NED format (rel camera)
        posR = tagPlacement.reportedPos
        rotR = tagPlacement.reportedRot
        rotD = numpy.rad2deg(tagPlacement.reportedRot)
        speed = tagPlacement.reportedVelocity

        with open(args.outFile, "a", encoding="utf-8") as outFile:
            outFile.write("{0},".format(timestamp))
            outFile.write("{0:.3f},{1:.3f},{2:.3f},".format(posR[0], posR[1], posR[2]))
            outFile.write("{0:.3f},{1:.3f},{2:.3f},".format(rotR[0], rotR[1], rotR[2]))
            outFile.write("{0:.3f},{1:.3f},{2:.3f}\n".format(speed[0], speed[1], speed[2]))
        # print("Time to capture, detect and localise = {0:.3f} sec, using {2}/{1} tags".format(time.time() - myStart,

        # Create and send MAVLink packet at same rate as camera
        threadMavlink.sendPos(posR, rotR)
        threadMavlink.sendSpeed(speed)

        # Send to status thread
        threadStatus.updateData(time.time() - timestamp,
                                (posR[0], posR[1], posR[2]),
                                (rotD[0], rotD[1], rotD[2]),
                                threadMavlink.getPktSent())

        # Send to save thread. Limit to 10fps
        if threadSave and (timestamp - threadSave_prev_timestamp) > 0.1:
            if args.multiCamera:
                for CAMERA in CAMERAS:
                    threadSave.save_queue.put((CAMERA.imageBW, os.path.join(
                        ".", args.outputFolder, CAMERA.camName, "processed_{:04d}.png".format(i)), posR, rotD,
                        CAMERA.tags))
            else:
                for CAMERA in CAMERAS:
                    threadSave.save_queue.put((CAMERA.imageBW, os.path.join(
                        ".", args.outputFolder, "processed_{:04d}.png".format(i)), posR, rotD,
                        CAMERA.tags))
            threadSave_prev_timestamp = timestamp
            i += 1

        # Get ready for next frame
        tagPlacement.newFrame()

        # Send to video stream, if option.
        if threadVideo:
            threadVideo.frame_queue.put((CAMERA.imageBW, CAMERA.tags, posR, rotD))

        if exit_event.is_set():
            break

    # close camera
    for CAMERA in CAMERAS:
        CAMERA.close()

    exit_event.set()
