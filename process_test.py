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
from collections import defaultdict

import numpy

from modules.geo import getPos, getTransform, getRotation
from modules.common import do_multi_capture, get_average_timestamps, loadCameras, get_num_images, tryCheckCuda
from modules.aprilDetect import aprilDetect, tagEngines

exit_event = threading.Event()


def signal_handler(signum, frame):
    """
    Signal handler for exit
    """
    exit_event.set()


def main(mainargs):
    print("Initialising")

    tryCheckCuda(mainargs.cuda)

    # Open camera settings and load camera(s)
    CAMERAS = loadCameras(mainargs.multiCamera, mainargs.camera, mainargs.inputFolder, mainargs.cuda)

    # allow the camera to warmup
    time.sleep(2)

    at_detector = aprilDetect(mainargs.tagSize, mainargs.tagFamily, mainargs.decimation, mainargs.tagEngine)

    # how many loops. If using a file input, just use min images
    loops = get_num_images(CAMERAS, mainargs.loop)

    print("Starting {0} image capture and process...".format(loops))
    signal.signal(signal.SIGINT, signal_handler)

    with open(mainargs.outFile, "w+", encoding="utf-8") as outFile:
        outFile.write("{0},{1},{2},{3},{4},{5},{6},{7},{8},{9}\n".format("Filename", "CameraName", "TagID",
                                                                         "PosX (fwd)", "PosY (right)", "PosZ (down)",
                                                                         "RotX (roll)", "RotY (pitch)", "RotZ (yaw)",
                                                                         "PoseErr"))

    # hold all pose errors to get average at end:
    all_pose_error = []
    all_tags_pos = defaultdict(list)
    all_tags_rot = defaultdict(list)

    for i in range(loops):
        print("--------------------------------------")
        # Capture images from all cameras (in parallel)
        img_by_cam = {}
        tags_by_cam = {}

        img_by_cam = do_multi_capture(CAMERAS)
        # check for any bad captures
        shouldExit = False
        for CAMERA in CAMERAS:
            if img_by_cam[CAMERA.camName][0] is None:
                print("Bad capture from {0}. Exiting".format(CAMERA.camName))
                shouldExit = True
        if shouldExit:
            break
        if mainargs.inputFolder:
            timestamp = time.time()
        else:
            timestamp = get_average_timestamps(img_by_cam)

        # Detect tags in each camera
        for CAMERA in CAMERAS:
            # AprilDetect, after accounting for distortion  (if fisheye)
            if at_detector.tagEngine == tagEngines.OpenCV:
                tags = at_detector.detect(img_by_cam[CAMERA.camName][0], CAMERA.K)
            else:
                tags = at_detector.detect(img_by_cam[CAMERA.camName][0], CAMERA.KFlat)

            tags_by_cam[CAMERA.camName] = tags
            if img_by_cam[CAMERA.camName][2]:
                print("File: {0} ({1}/{2})".format(img_by_cam[CAMERA.camName][2], i + 1, loops))
            else:
                print("Capture {0}: ({1}/{2})".format(CAMERA.camName, i + 1, loops))

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
                all_tags_pos[tag.tag_id].append(tagpos)
                all_tags_rot[tag.tag_id].append(tagrot)

                print("Cam {0}, Tag {1} pos = {2} m, Rot = {3} deg. ErrE8 = {4:.4f}".format(CAMERA.camName,
                                                                                            tag.tag_id,
                                                                                            tagpos.round(3),
                                                                                            tagrot.round(1),
                                                                                            tag.pose_err*1E8))
                with open(mainargs.outFile, "a", encoding="utf-8") as outFile:
                    outFile.write("{0},{1},{2},".format(img_by_cam[CAMERA.camName][2], CAMERA.camName, tag.tag_id))
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
    for tag_id, posn in all_tags_pos.items():
        # Convert to NumPy arrays
        posns_array = numpy.array(posn)
        translation_mean = numpy.mean(posns_array, axis=0)
        translation_std = numpy.std(posns_array, axis=0)
        print("Tag ID Pos {0} mean: {1} and Std dev {2}".format(tag_id, translation_mean,
                                                                translation_std))
    for tag_id, rots in all_tags_rot.items():
        # Convert to NumPy arrays
        rots_array = numpy.array(rots)
        rot_mean = numpy.mean(rots_array, axis=0)
        rot_std = numpy.std(rots_array, axis=0)
        print("Tag ID Rot {0} mean: {1} and Std dev {2}".format(tag_id, rot_mean, rot_std))
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
    parser.add_argument('--cuda', dest='cuda', help="Use OpenCV CUDA Extensions",
                        default=False, action='store_true')
    parser.add_argument("--multiCamera", type=str, default=None,
                        help="multiple cameras using the specified yaml file")
    parser.add_argument('--tagEngine', dest='tagEngine', help="Tag detector engine",
                        default='PyAprilTags', choices=['OpenCV', 'PyAprilTags'])
    args = parser.parse_args()
    main(args)
