#!/usr/bin/env python3
'''
Apriltag capture and detection performance test.

For debugging, also shows distance to each detected tag.

Distance is relative to the camera's sensor in 3 dimensions.

'''

import signal
import threading
import time
import argparse
import os
from collections import defaultdict
import concurrent.futures

import numpy
from pyapriltags import Detector

from modules.geo import getPos, getTransform, getRotation
from modules.common import loadCameras

exit_event = threading.Event()


def signal_handler(signum, frame):
    """
    Signal handler for exit
    """
    exit_event.set()


def capture_image(CAMERA):
    """
    Captures an image using the provided CAMERA object.

    Args:
        CAMERA: An object representing the camera, which must have the methods
                getFileName() and getImage().

    Returns:
        tuple: A tuple containing:
            - camName (str): The name of the camera.
            - imageBW (numpy.ndarray or None): The captured image in black and white, or None if no image is captured.
            - filename (str or None): The filename of the captured image, or None if no image is captured.
    """
    filename = CAMERA.getFileName()
    (imageBW, _) = CAMERA.getImage()

    # we're out of images
    if imageBW is None:
        return CAMERA.camName, None, None

    return CAMERA.camName, imageBW, filename


def main(mainargs):
    print("Initialising")

    # Open camera settings and load camera(s)
    CAMERAS = loadCameras(mainargs.multiCamera, mainargs.camera, mainargs.inputFolder, mainargs.jetson)

    # allow the camera to warmup
    time.sleep(2)

    at_detector = Detector(searchpath=['apriltags3py/apriltags/lib', 'apriltags3py/apriltags/lib'],
                           families=mainargs.tagFamily,
                           nthreads=max(1, os.cpu_count() - 1),
                           quad_decimate=mainargs.decimation,
                           quad_sigma=0.0,
                           refine_edges=1,
                           decode_sharpening=0.25,
                           debug=0)

    # how many loops. If using a file input, just use first camera
    loops = CAMERAS[0].getNumberImages() if CAMERAS[0].getNumberImages() else mainargs.loop

    print("Starting {0} image capture and process...".format(loops))
    signal.signal(signal.SIGINT, signal_handler)

    with open(mainargs.outFile, "w+", encoding="utf-8") as outFile:
        outFile.write("{0},{1},{2},{3},{4},{5},{6},{7},{8},{9}\n".format("Filename", "CameraName", "TagID",
                                                                         "PosX (fwd)", "PosY (right)", "PosZ (down)",
                                                                         "RotX (roll)", "RotY (pitch)", "RotZ (yaw)",
                                                                         "PoseErr"))

    # hold all pose errors to get average at end:
    all_pose_error = []
    all_tags = defaultdict(list)

    for i in range(loops):
        print("--------------------------------------")
        # Capture images from all cameras (in parallel)
        img_by_cam = {}
        tags_by_cam = {}

        timestamp = time.time()
        with concurrent.futures.ThreadPoolExecutor() as executor:
            futures = {executor.submit(capture_image, CAMERA): CAMERA for CAMERA in CAMERAS}
            for future in concurrent.futures.as_completed(futures):
                cam_name, imageBW, filename = future.result()
                if imageBW is not None:
                    img_by_cam[cam_name] = imageBW
                    # print("Camera {0} capture time is {1:.1f}ms".format(cam_name, 1000*(time.time() - timestamp)))
                else:
                    break

        # Detect tags in each camera
        for CAMERA in CAMERAS:
            # AprilDetect, after accounting for distortion  (if fisheye)
            tags = at_detector.detect(img_by_cam[CAMERA.camName], True, CAMERA.KFlat, mainargs.tagSize/1000)
            tags_by_cam[CAMERA.camName] = tags
            if filename:
                print("File: {0} ({1}/{2})".format(filename, i, loops))

        # get time to capture and convert
        print("Time to capture and detect = {0:.1f} ms. ".format(1000*(time.time() - timestamp)))
        for CAMERA in CAMERAS:
            print("Camera {0} found {1} tags. ".format(CAMERA.camName, len(tags_by_cam[CAMERA.camName])))

        # Convert to vehicle frame and add to list
        for CAMERA in CAMERAS:
            for tag in tags_by_cam[CAMERA.camName]:
                tag_Veh = CAMERA.T_CamtoVeh @ getTransform(tag)

                tagpos = getPos(tag_Veh)
                tagrot = getRotation(tag_Veh)
                all_pose_error.append(tag.pose_err*1E8)
                all_tags[tag.tag_id].append(tagpos)

                print("Cam {0}, Tag {1} pos = {2} m, Rot = {3} deg. ErrE8 = {4:.4f}".format(CAMERA.camName,
                                                                                            tag.tag_id,
                                                                                            tagpos.round(3),
                                                                                            tagrot.round(1),
                                                                                            tag.pose_err*1E8))
                with open(mainargs.outFile, "a", encoding="utf-8") as outFile:
                    outFile.write("{0},{1},{2},".format(filename, CAMERA.camName, tag.tag_id))
                    outFile.write("{0:.3f},{1:.3f},{2:.3f},{3:.1f},{4:.1f},{5:.1f},{6}\n".format(tagpos[0],
                                                                                                 tagpos[1],
                                                                                                 tagpos[2],
                                                                                                 tagrot[0],
                                                                                                 tagrot[1],
                                                                                                 tagrot[2],
                                                                                                 tag.pose_err))

        if exit_event.is_set():
            break

    # print out statistics
    print("--------------------------------------")
    if len(all_pose_error) > 0:
        print("Pose error (1E8) mean: {0:.3f} and Std dev {1:.3f}".format(numpy.mean(all_pose_error),
                                                                          numpy.std(all_pose_error)))
    # Compute statistics for each tag
    for tag_id, posns in all_tags.items():
        # Convert to NumPy arrays
        posns_array = numpy.array(posns)
        translation_mean = numpy.mean(posns_array, axis=0)
        translation_std = numpy.std(posns_array, axis=0)
        print("Tag ID {0} mean: {1} and Std dev {2}".format(tag_id, translation_mean,
                                                            translation_std))

    # close camera
    for CAMERA in CAMERAS:
        CAMERA.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--camera", type=str, default="PiCamV2FullFoV",
                        help="Camera profile in camera.yaml")
    parser.add_argument("--loop", type=int, default=10,
                        help="Process this many frames")
    parser.add_argument("--tagSize", type=int, default=94,
                        help="Apriltag size in mm")
    parser.add_argument("--tagFamily", type=str, default="tagStandard41h12",
                        help="Apriltag family")
    parser.add_argument("--inputFolder", type=str, default=None,
                        help="Use a folder of images instead of live camera")
    parser.add_argument("--outFile", type=str, default="processed.csv",
                        help="Output tag data to this file")
    parser.add_argument("--decimation", type=int,
                        default=2, help="Apriltag decimation")
    parser.add_argument('--jetson', dest='jetson', help="Use Jetson hardware acceleration",
                        default=False, action='store_true')
    parser.add_argument("--multiCamera", type=str, default=None,
                        help="multiple cameras using the specified yaml file")
    args = parser.parse_args()
    main(args)
