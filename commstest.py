#!/usr/bin/env python3
'''
Main script. Will use Apriltags to localise position and send via mavlink

'''
import time
import argparse
import threading
import signal

import numpy

from pymavlink import mavutil

exit_event = threading.Event()


def signal_handler(signum, frame):
    '''Handle exit signal'''
    exit_event.set()


class mavThread(threading.Thread):
    '''Thread to handle all MAVLink communications with flight controller'''
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
        '''Sync data to this thread'''
        with self.lock:
            if self.time != 0:
                # time is in usec here, remember to convert to sec
                self.speed = numpy.array(posDelta) / (1E-6 * (t - self.time))
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

        # wait for the heartbeat msg to find the system ID of the autopilot. Need to exit from here too
        # We are sending a heartbeat signal too, to allow ardupilot to init the comms channel
        while True:
            self.sendHeartbeat()
            if self.conn.wait_heartbeat(timeout=0.5) is not None and self.source_system in self.conn.sysid_state:
                # Got a hearbeart, go to next loop
                self.goodToSend = True
                # self.target_system = self.conn.target_system
                self.conn.target_component = 1
                self.conn.target_system = 1
                break
            if exit_event.is_set():
                return

        print("Got Heartbeat from APM (system %u component %u)" %
              (self.conn.target_system, self.conn.target_component))
        self.send_msg_to_gcs("Starting")

        while True:
            # msg = self.conn.recv_match(blocking=True, timeout=0.5)
            # loop at 20 Hz
            time.sleep(0.05)
            self.sendPos()
            self.sendSpeed()
            self.sendPosDelta()
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
        '''send heartbeat and EKF origin messages if more than 3 sec since last message'''
        if (self.heartbeatTimestamp + 3) < time.time():
            self.conn.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                                         mavutil.mavlink.MAV_AUTOPILOT_GENERIC,
                                         0,
                                         0,
                                         0)
            self.heartbeatTimestamp = time.time()

    def getPktSent(self):
        '''Return the last packet send'''
        with self.lock:
            return self.pktSent

    def send_msg_to_gcs(self, text_to_be_sent):
        '''Send a STATUSTEXT message to the GCS
        https://mavlink.io/en/messages/common.html#STATUSTEXT
        '''
        # MAV_SEVERITY: 0=EMERGENCY 1=ALERT 2=CRITICAL 3=ERROR, 4=WARNING, 5=NOTICE, 6=INFO, 7=DEBUG, 8=ENUM_END
        text_msg = 'AprilMAVTest: ' + text_to_be_sent
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

    def sendPos(self):
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
                    current_time_us, self.pos[0], self.pos[1], self.pos[2],
                    self.rot[0], self.rot[1], self.rot[2],
                    covariance, reset_counter=self.reset_counter)
                self.pktSent += 1

    def sendSpeed(self):
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
                    current_time_us, self.speed[0], self.speed[1], self.speed[2],
                    covariance, reset_counter=self.reset_counter)
                self.pktSent += 1

    def sendPosDelta(self):
        '''Send a vision pos delta
        https://mavlink.io/en/messages/ardupilotmega.html#VISION_POSITION_DELTA
        '''
        if self.goodToSend:
            with self.lock:
                current_time_us = int(round(time.time() * 1000000))
                delta_time_us = current_time_us - self.time
                current_confidence_level = 80

                # Send the message
                self.conn.mav.vision_position_delta_send(
                    # us: Timestamp (UNIX time or time since system boot)
                    current_time_us,
                    delta_time_us,	    # us: Time since last reported camera frame
                    # float[3] in radian: Defines a rotation vector in body frame that rotates the vehicle from the
                    # previous to the current orientation
                    self.rotDelta,
                    # float[3] in m: Change in position from previous to current frame rotated into body frame
                    # (0=forward, 1=right, 2=down)
                    self.posDelta,
                    # Normalized confidence value from 0 to 100.
                    current_confidence_level
                )
                self.pktSent += 1


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--device", type=str, default="udpin:192.168.1.124:15020", help="MAVLink connection string")
    parser.add_argument("--baud", type=int, default=115200,
                        help="MAVLink baud rate, if using serial")
    parser.add_argument("--source-system", type=int,
                        default=1, help="MAVLink Source system")
    args = parser.parse_args()

    print("Initialising")

    # allow the camera to warmup
    time.sleep(2)

    signal.signal(signal.SIGINT, signal_handler)

    # Start MAVLink comms thread
    threadMavlink = mavThread(args.device, args.baud, args.source_system)
    threadMavlink.start()

    while True:
        # grab an image (and timestamp in usec) from the camera
        # estimate 50usec from timestamp to frame capture on next line
        timestamp = int(round(time.time() * 1000000)) + 50

        # Create and send MAVLink packet
        threadMavlink.updateData(
            [0, 0, 0], [0, 0, 0], timestamp, [0, 0, 0], [0, 0, 0])
        # wasSent = threadMavlink.sendPos(posR[0], posR[1], posR[2], rotR[0], rotR[1], rotR[2], timestamp)
        time.sleep(0.5)

        if exit_event.is_set():
            break
