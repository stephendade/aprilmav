#!/usr/bin/env python3
'''
Camera Capture performance test

All images will be stored as "<timestamp in ms>.png"

'''

import time
import argparse
import os
import sys
import queue
import threading
from importlib import import_module
import cv2
import yaml

save_queue = queue.Queue()
shouldExit = False


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
    parser.add_argument("--folder", type=str, default="capture",
                        help="Put capture into this folder")
    args = parser.parse_args()

    print("Initialising Camera")

    # Open camera settings
    with open('camera.yaml', 'r', encoding="utf-8") as stream:
        parameters = yaml.load(stream, Loader=yaml.FullLoader)

    # create the capture folder if required
    try:
        os.makedirs(os.path.join(".", args.folder))
    except FileExistsError:
        pass

    # initialize the camera
    camera = None
    try:
        print(parameters[args.camera]['cam_name'])
        mod = import_module("lib." + parameters[args.camera]['cam_name'])
        camera = mod.camera(parameters[args.camera])
    except (ImportError, KeyError):
        print('No camera with the name {0}, exiting'.format(args.camera))
        sys.exit(0)

    print("Starting {0} image capture...".format(args.loop))

    worker = threading.Thread(target=save_threadfunc, args=())
    worker.daemon = True
    worker.start()

    for i in range(args.loop):
        (imageBW, timestamp) = camera.getImage()

        # get time to capture and convert
        print("Captured {1:04d}.png in {0:.0f}ms".format(
            (time.time() - timestamp)*1000, i))

        time.sleep(args.delay/1000)

        # write image to save queue as (image, filename) tuple
        # note timestamp stored as millisec
        save_queue.put((imageBW, os.path.join(
            ".", args.folder, "{0:.0f}.png".format(timestamp*1000))))

    # close camera
    camera.close()

    # wait for images to finish saving
    shouldExit = True
    worker.join()
