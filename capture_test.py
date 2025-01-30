#!/usr/bin/env python3
'''
Camera Capture performance test

All images will be stored as "<timestamp in ms>.png"

'''

import signal
import time
import argparse
import os
import queue
import threading
import cv2

from modules.common import loadCameras
from modules.videoStream import videoThread

save_queue = queue.Queue()
shouldExit = False

exit_event = threading.Event()


def signal_handler(signum, frame):
    """
    Signal handler for exit
    """
    exit_event.set()


def save_threadfunc():
    '''Separate thread for saving images, in order to not delay image capture'''
    while True:
        if save_queue.empty():
            continue
        (image, filename) = save_queue.get()
        cv2.imwrite(filename, image, [cv2.IMWRITE_PNG_COMPRESSION, 0])
        print("Saved {0}".format(filename))
        if save_queue.empty() and shouldExit:
            break
    print("Exited save thread")


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--camera", type=str, default="GenericUSB",
                        help="Camera profile in camera.yaml")
    parser.add_argument("--loop", type=int, default=20,
                        help="Capture this many frames")
    parser.add_argument("--delay", type=int, default=50,
                        help="Delay by N millisec between frame captures")
    parser.add_argument("--outputFolder", type=str, default="",
                        help="Put captured images into this folder")
    parser.add_argument("--video", type=str, default='',
                        help="Output video to this IP:port")
    args = parser.parse_args()

    print("Initialising Camera")

    # Open camera settings and load camera(s)
    CAMERAS = loadCameras(None, args.camera, None, None)

    # create the capture folder if required
    if args.outputFolder != "":
        try:
            os.makedirs(os.path.join(".", args.outputFolder))
        except FileExistsError:
            pass

    print("Starting {0} image capture...".format(args.loop))
    signal.signal(signal.SIGINT, signal_handler)

    if args.outputFolder != "":
        worker = threading.Thread(target=save_threadfunc, args=())
        worker.daemon = True
        worker.start()

    # video stream out, if desired
    threadVideo = None
    if args.video != '':
        threadVideo = videoThread(args.video, exit_event)
        threadVideo.start()

    for i in range(args.loop):
        (imageBW, timestamp) = CAMERAS[0].getImage(get_raw=True)

        # get time to capture and convert
        print("Captured {0:.0f}.png in {1:.0f}ms ({2}/{3})".format(
            timestamp*1000, (time.time() - timestamp)*1000, i, args.loop))

        time.sleep(args.delay/1000)

        # write image to save queue as (image, filename) tuple
        # note timestamp stored as millisec
        if args.outputFolder != "":
            save_queue.put((imageBW, os.path.join(
                ".", args.outputFolder, "{0:.0f}.png".format(timestamp*1000))))

        # Send to video stream, if option
        if threadVideo:
            threadVideo.frame_queue.put((imageBW, None, None, None))

        if exit_event.is_set():
            break

    # close camera
    CAMERAS[0].close()

    exit_event.set()

    # wait for images to finish saving
    if args.outputFolder != "":
        shouldExit = True
        worker.join()
