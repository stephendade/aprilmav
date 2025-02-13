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

from modules.common import do_multi_capture, get_average_timestamps, loadCameras
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
    parser.add_argument("--multiCamera", type=str, default=None,
                        help="multiple cameras using the specified yaml file")
    args = parser.parse_args()

    print("Initialising Camera")

    # Open camera settings and load camera(s)
    CAMERAS = loadCameras(args.multiCamera, args.camera, None, None)

    # create the capture folder(s) if required
    # note multi-camera will create a folder for each camera
    if args.outputFolder != "":
        try:
            if args.multiCamera:
                for CAMERA in CAMERAS:
                    os.makedirs(os.path.join(
                        ".", args.outputFolder, CAMERA.camName))
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
        # Capture images from all cameras (in parallel)
        img_by_cam = {}

        img_by_cam = do_multi_capture(CAMERAS, True)
        timestamp = get_average_timestamps(img_by_cam)

        # get time to capture and convert
        for CAMERA in CAMERAS:
            print("{0} captured {1:.0f}.png in {2:.0f}ms ({3}/{4})".format(
                CAMERA.camName, img_by_cam[CAMERA.camName][1]*1000,
                (time.time() - timestamp)*1000, i, args.loop))

        time.sleep(args.delay/1000)

        # write image to save queue as (image, filename) tuple
        # note timestamp stored as millisec
        if args.outputFolder != "":
            if args.multiCamera:
                for CAMERA in CAMERAS:
                    save_queue.put((img_by_cam[CAMERA.camName][0],
                                    os.path.join(".", args.outputFolder,
                                                 CAMERA.camName,
                                                 "{0:.0f}.png".format(img_by_cam[CAMERA.camName][1]*1000))))
            else:
                save_queue.put((img_by_cam[CAMERAS[0].camName][0], os.path.join(
                    ".", args.outputFolder, "{0:.0f}.png".format(img_by_cam[CAMERAS[0].camName][1]*1000))))

        # Send to video stream, if option. Combine all images into one
        if threadVideo:
            finalImage = None
            for CAMERA in CAMERAS:
                cam_image = img_by_cam[CAMERA.camName][0]
                if args.multiCamera:
                    # overlay camera name on the image
                    cv2.putText(cam_image, CAMERA.camName, (10, 45),
                                cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 0, 0), 2)
                    # put a border around the image
                    cam_image = cv2.copyMakeBorder(
                        cam_image, 5, 5, 5, 5, cv2.BORDER_CONSTANT, value=(255, 255, 255))
                if finalImage is None:
                    finalImage = cam_image
                else:
                    finalImage = cv2.hconcat([finalImage, cam_image])
            threadVideo.frame_queue.put((finalImage, None, None, None))

        if exit_event.is_set():
            break

    # close camera
    for CAMERA in CAMERAS:
        CAMERA.close()

    exit_event.set()

    # wait for images to finish saving
    if args.outputFolder != "":
        shouldExit = True
        print("Waiting for images to be saved to file...")
        worker.join()

print("Done")
