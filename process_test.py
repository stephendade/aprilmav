#!/usr/bin/env python3
'''
Apriltag capture and detection performance test.

For debugging, also shows distance to each detected tag.

Distance is relative to the camera's sensor in 3 dimensions.

'''

from concurrent.futures import ThreadPoolExecutor, as_completed
import time
import argparse
import sys
import os
from importlib import import_module
from collections import defaultdict

import numpy
import yaml

from modules.geo import getPos, getTransform, getRotation


def main(args):
    print("Initialising")

    # Open camera settings
    with open(args.yaml, 'r', encoding="utf-8") as stream:
        parameters = yaml.load(stream, Loader=yaml.FullLoader)

    # initialize the camera(s)
    CAMERA = []
    if args.multicamera != "":
        workers = int(os.cpu_count() / len(args.multicamera.split(","))) - 1
        for cam in args.multicamera.split(","):
            try:
                print(parameters[cam]['cam_driver'])
                mod = import_module("drivers." + parameters[cam]['cam_driver'])
                CAMERA.append(mod.camera(parameters[cam], args.decimation,
                                         workers, args.tagSize/1000))
            except (ImportError, KeyError):
                print('No camera with the name {0}, exiting'.format(cam))
                sys.exit(0)
    elif args.folder:
        workers = os.cpu_count() - 1
        from drivers import cameraFile
        CAMERA.append(cameraFile.FileCamera(parameters[args.camera], args.folder,
                                            args.decimation, workers, args.tagSize/1000))
    else:
        try:
            workers = os.cpu_count() - 1
            print(parameters[args.camera]['cam_driver'])
            mod = import_module("drivers." + parameters[args.camera]['cam_driver'])
            CAMERA.append(mod.camera(parameters[args.camera], args.decimation,
                                     workers, args.tagSize/1000))
        except (ImportError, KeyError):
            print('No camera with the name {0}, exiting'.format(args.camera))
            sys.exit(0)

    # allow the cameras to warmup
    time.sleep(2)

    # how many loops
    # loops = CAMERA.getNumberImages() if CAMERA.getNumberImages() else args.loop
    loops = min([C.getNumberImages() for C in CAMERA]) if CAMERA[0].getNumberImages() else args.loop

    print("Starting {0} image capture and process...".format(loops))

    with open(args.outfile, "w+", encoding="utf-8") as outfile:
        outfile.write("{0},{1},{2},{3},{4},{5},{6},{7},{8}\n".format("Filename", "TagID", "PosX (left)", "PosY (up)",
                                                                     "PosZ (fwd)", "RotX (pitch)", "RotY (yaw)",
                                                                     "RotZ (roll)", "PoseErr"))

    # hold all pose errors to get average at end:
    all_pose_error = []
    all_tags = defaultdict(list)
    for i in range(loops):
        print("--------------------------------------")
        tags = []
        executor = ThreadPoolExecutor(max_workers=len(CAMERA))
        futures = []
        for CAM in CAMERA:
            # grab an image from the camera
            file = CAM.getFileName()
            if file:
                print("File: {0} ({1}/{2})".format(file, i, loops))
            # (imageBW, timestamp) = CAM.getImage()
            futures.append(executor.submit(CAM.getApriltagsandImage))

        # wait for all threads to finish
        for future in as_completed(futures):
            tags, timestamp = future.result()

            # get time to capture and convert
            print("Time to capture and detect = {0:.1f} ms, found {1} tags".format(
                1000*(time.time() - timestamp), len(tags)))

            # TODO: figure out how to works with all the tags, with different pos/rot
            for tag in tags:

                tagpos = getPos(getTransform(tag))
                tagrot = getRotation(getTransform(tag))
                all_pose_error.append(tag.pose_err*1E8)
                all_tags[tag.tag_id].append(tagpos)

                print("Tag {0} pos = {1} m, Rot = {2} deg. ErrE8 = {3:.4f}".format(tag.tag_id, tagpos.round(3),
                                                                                   tagrot.round(1), tag.pose_err*1E8))
                with open(args.outfile, "w+", encoding="utf-8") as outfile:
                    outfile.write("{0},{1},{2:.3f},{3:.3f},{4:.3f},{5:.1f},{6:.1f},{7:.1f},{8}\n".format(file,
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
    parser.add_argument("--folder", type=str, default=None,
                        help="Use a folder of images instead of camera")
    parser.add_argument("--outfile", type=str, default="processed.csv",
                        help="Output tag data to this file")
    parser.add_argument("--decimation", type=int,
                        default=2, help="Apriltag decimation")
    parser.add_argument("--multicamera", type=str, default="",
                        help="csv list of camera profiles to use. Overrides --camera")
    parser.add_argument("--yaml", type=str, default="camera.yaml",
                        help="Camera parameters file")
    args = parser.parse_args()
    main(args)
