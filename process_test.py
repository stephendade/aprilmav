#!/usr/bin/env python3
'''
Apriltag capture and detection performance test.

For debugging, also shows distance to each detected tag.

Distance is relative to the camera's sensor in 3 dimensions.

'''

import time
import argparse
import os
from collections import defaultdict

import numpy

from pyapriltags import Detector
from modules.geo import getPos, getTransform, getRotation
from modules.common import loadCameras


def main(args):
    print("Initialising")

    # Open camera settings and load camera(s)
    CAMERAS = loadCameras(args.multiCamera, args.camera, args.inputFolder, args.jetson)

    # allow the camera to warmup
    time.sleep(2)

    at_detector = Detector(searchpath=['apriltags3py/apriltags/lib', 'apriltags3py/apriltags/lib'],
                           families=args.tagFamily,
                           nthreads=max(1, os.cpu_count() - 1),
                           quad_decimate=args.decimation,
                           quad_sigma=0.0,
                           refine_edges=1,
                           decode_sharpening=0.25,
                           debug=0)

    # how many loops. If using a file input, just use first camera
    loops = CAMERAS[0].getNumberImages() if CAMERAS[0].getNumberImages() else args.loop

    print("Starting {0} image capture and process...".format(loops))

    with open(args.outFile, "w+", encoding="utf-8") as outFile:
        outFile.write("{0},{1},{2},{3},{4},{5},{6},{7},{8}\n".format("Filename", "TagID", "PosX (left)", "PosY (up)",
                                                                    "PosZ (fwd)", "RotX (pitch)", "RotY (yaw)",
                                                                    "RotZ (roll)", "PoseErr"))

    # hold all pose errors to get average at end:
    all_pose_error = []
    all_tags = defaultdict(list)

    for i in range(loops):
        print("--------------------------------------")
        # grab an image from the cameras
        tags = []
        image_timestamp_camera = []
        for CAMERA in CAMERAS:
            file = CAMERA.getFileName()
            (imageBW, timestamp) = CAMERA.getImage()

            # we're out of images
            if imageBW is None:
                break

            image_timestamp_camera.append((imageBW, timestamp, CAMERA.camParams['cam_params']))

        # do Apriltag detection and pose estimation on all captured images
        for imageBW, timestamp, camParam in image_timestamp_camera:
            # AprilDetect, after accounting for distortion  (if fisheye)
            tags.extend(at_detector.detect(imageBW, True, camParam, args.tagSize/1000))

            if file:
                print("File: {0} ({1}/{2})".format(file, i, loops))

            # get time to capture and convert
            print("Time to capture and detect = {0:.1f} ms, found {1} tags".format(
                1000*(time.time() - timestamp), len(tags)))

        for tag in tags:

            tagpos = getPos(getTransform(tag))
            tagrot = getRotation(getTransform(tag))
            all_pose_error.append(tag.pose_err*1E8)
            all_tags[tag.tag_id].append(tagpos)

            print("Tag {0} pos = {1} m, Rot = {2} deg. ErrE8 = {3:.4f}".format(tag.tag_id, tagpos.round(3),
                                                                               tagrot.round(1), tag.pose_err*1E8))
            with open(args.outFile, "w+", encoding="utf-8") as outFile:
                outFile.write("{0},{1},{2:.3f},{3:.3f},{4:.3f},{5:.1f},{6:.1f},{7:.1f},{8}\n".format(file,
                                                                                                     tag.tag_id,
                                                                                                     tagpos[0],
                                                                                                     tagpos[1],
                                                                                                     tagpos[2],
                                                                                                     tagrot[0],
                                                                                                     tagrot[1],
                                                                                                     tagrot[2],
                                                                                                     tag.pose_err))
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
