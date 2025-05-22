#!/usr/bin/env python3
'''
This script localises any visible Apriltags and
calulates the camera's rotation and position as a
transformation matrix T_CamToWorld, relative to it's
starting location.

Note camera settings are specific to the camera model and
settings. Use cameracal.py to generate new settings and put
into camera.yaml
'''
import time
import argparse
import threading
import signal
import os
import numpy

from modules.common import do_multi_capture, get_average_timestamps, loadCameras, get_num_images, tryCheckCuda
from modules.geo import tagDB
from modules.saveStream import saveThread
from modules.aprilDetect import aprilDetect, tagEngines

exit_event = threading.Event()


def signal_handler(signum, frame):
    """
    Signal handler for exit
    """
    exit_event.set()


def main(args):
    print("Initialising")

    tryCheckCuda(args.cuda)

    signal.signal(signal.SIGINT, signal_handler)

    # Open camera settings and load camera(s)
    CAMERAS = loadCameras(args.multiCamera, args.camera, args.inputFolder, args.cuda)

    # allow the camera to warmup
    time.sleep(2)

    at_detector = aprilDetect(args.tagSize, args.tagFamily, args.decimation, args.tagEngine)

    # All tags live in here
    tagPlacement = tagDB(slidingWindow=args.outliers, extraOpt=args.extraOpt)

    # how many loops
    loops = get_num_images(CAMERAS, args.loop)

    # Start save image thread, if desired
    threadSave = None
    if args.outputFolder != "":
        threadSave = saveThread(args.outputFolder, exit_event, CAMERAS)
        threadSave.start()

    print("Starting {0} image capture and process...".format(loops))

    with open(args.outFile, "w+", encoding="utf-8") as outFile:
        outFile.write("Filename,")
        outFile.write("PosX (m),PosY (m),PosZ (m),")
        outFile.write("RotX (rad),RotY (rad),RotZ (rad),")
        outFile.write("VelX (m/s),VelY (m/s), VelZ (m/s),")
        outFile.write("tagsUsed,")
        outFile.write("timestamp (sec)\n")

    # GUI
    if args.gui:
        from modules.gui import GUI
        AprilGUI = GUI()

    # Timing stats
    allTimeCapture = []
    allTimeRectify = []
    allTimeDetect = []
    allTimeLocalise = []
    allTimeTotal = []

    for i in range(loops):
        print("--------------------------------------")
        # Capture images from all cameras (in parallel)
        img_by_cam = {}
        tags_by_cam = {}

        timestamp = time.time()
        img_by_cam = do_multi_capture(CAMERAS)
        # check for any bad captures
        shouldExit = False
        for CAMERA in CAMERAS:
            if img_by_cam[CAMERA.camName][0] is None:
                print("Bad capture from {0}. Exiting".format(CAMERA.camName))
                shouldExit = True
        if shouldExit:
            break
        if args.inputFolder:
            frameTime = time.time()
        else:
            frameTime = get_average_timestamps(img_by_cam)
        # get the mean capture and rectify times
        allTimeCapture.append(numpy.mean([img_by_cam[CAMERA.camName][3] for CAMERA in CAMERAS]))
        allTimeRectify.append(numpy.mean([img_by_cam[CAMERA.camName][4] for CAMERA in CAMERAS]))

        # Detect tags in each camera
        detectStart = time.time()
        tags_by_cam = do_multi_detect(CAMERAS, img_by_cam, args.tagSize, args.tagFamily,
                                      args.decimation, args.tagEngine)
        #for CAMERA in CAMERAS:
        #    tags = aprilTagDetectStatic(args.tagSize, args.tagFamily, args.decimation, args.tagEngine,
        #                                img_by_cam[CAMERA.camName][0], CAMERA.K, CAMERA.KFlat)
        #    tags_by_cam[CAMERA.camName] = tags
        allTimeDetect.append(time.time() - detectStart)

        for CAMERA in CAMERAS:
            if img_by_cam[CAMERA.camName][2]:
                print("File: {0} ({1}/{2})".format(img_by_cam[CAMERA.camName][2], i + 1, loops))
            else:
                print("Capture {0}: ({1}/{2})".format(CAMERA.camName, i + 1, loops))
            print("Camera {0} found {1} tags. ".format(CAMERA.camName, len(tags_by_cam[CAMERA.camName])))

        # feed tags into tagPlacement
        localiseStart = time.time()
        for CAMERA in CAMERAS:
            for tag in tags_by_cam[CAMERA.camName]:
                if tag.pose_err < args.maxError*1e-8:
                    tagPlacement.addTag(tag, CAMERA.T_CamtoVeh)

        tagPlacement.getBestTransform(frameTime)

        posR = tagPlacement.reportedPos
        rotR = tagPlacement.reportedRot
        rotD = numpy.rad2deg(tagPlacement.reportedRot)
        speed = tagPlacement.reportedVelocity

        allTimeLocalise.append(time.time() - localiseStart)
        allTimeTotal.append(time.time() - timestamp)

        with open(args.outFile, "a", encoding="utf-8") as outFile:
            if args.inputFolder:
                for CAMERA in CAMERAS:
                    file = img_by_cam[CAMERA.camName][2]
                    outFile.write("{0}".format(file))
                outFile.write(",")
            else:
                outFile.write(",")
            outFile.write("{0:.4f},{1:.4f},{2:.4f},".format(posR[0], posR[1], posR[2]))
            outFile.write("{0:.3f},{1:.3f},{2:.3f},".format(rotR[0], rotR[1], rotR[2]))
            outFile.write("{0:.3f},{1:.3f},{2:.3f},".format(speed[0], speed[1], speed[2]))
            outFile.write("{0},".format(' '.join(str(id) for id in tagPlacement.tagDuplicatesT.keys())))
            outFile.write("{0:.6f}\n".format(frameTime))

        # Update the live graph
        if args.gui:
            AprilGUI.update(posR, rotD)
            AprilGUI.updateImage(img_by_cam, tags_by_cam)

        totalTags = sum(len(tags) for tags in tags_by_cam.values())
        print("Time to capture, detect, localise = {0:.2f} ms, {1}/{2} tags".format(1000*(allTimeTotal[-1]),
                                                                                    len(tagPlacement.tagDuplicatesT),
                                                                                    totalTags))

        # Send to save thread
        if threadSave:
            # threadSave.save_queue.put((imageBW, os.path.join(
            #     ".", args.outputFolder, "processed_{:04d}.png".format(i)), posR, rotD, tags))
            if args.multiCamera:
                for CAMERA in CAMERAS:
                    threadSave.save_queue.put((img_by_cam[CAMERA.camName][0], os.path.join(
                        ".", args.outputFolder, CAMERA.camName, "processed_{:04d}.png".format(i)), posR, rotD,
                        tags_by_cam[CAMERA.camName]))
            else:
                for CAMERA in CAMERAS:
                    threadSave.save_queue.put((img_by_cam[CAMERA.camName][0], os.path.join(
                        ".", args.outputFolder, "processed_{:04d}.png".format(i)), posR, rotD,
                        tags_by_cam[CAMERA.camName]))

        tagPlacement.newFrame()

        if exit_event.is_set():
            break

    # print out the positions of all the tags
    print("----Final tag positions----")
    tagPlacement.printTags()

    # print out the average timing stats
    print("----Timing stats per frame----")
    print("Capture (per camera mean): {0:.2f} ms".format(1000*numpy.mean(allTimeCapture)))
    print("Rectify (per camera mean): {0:.2f} ms".format(1000*numpy.mean(allTimeRectify)))
    print("Detect: {0:.2f} ms".format(1000*numpy.mean(allTimeDetect)))
    print("Localise: {0:.2f} ms".format(1000*numpy.mean(allTimeLocalise)))
    totalTime = numpy.mean(allTimeTotal)
    print("Total (all cameras): {0:.2f} ms / {1:.0f} FPS".format(1000*totalTime, 1/totalTime))

    # close camera
    for CAMERA in CAMERAS:
        CAMERA.close()

    exit_event.set()

    # Tags
    if args.gui:
        AprilGUI.on_end()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--tagSize", type=int, default=96,
                        help="Apriltag size in mm")
    parser.add_argument("--camera", type=str, default="PiCamV2FullFoV",
                        help="Camera profile in camera.yaml")
    parser.add_argument("--loop", type=int, default=20,
                        help="Capture and process this many frames")
    parser.add_argument("--maxError", type=int, default=400,
                        help="Maximum pose error to use, in n*E-8 units")
    parser.add_argument("--inputFolder", type=str, default=None,
                        help="Use a folder of images instead of live camera")
    parser.add_argument("--outFile", type=str, default="geo_test_results.csv",
                        help="Output position data to this file")
    parser.add_argument('--gui', dest='gui',
                        default=False, action='store_true')
    parser.add_argument("--decimation", type=int,
                        default=2, help="Apriltag decimation")
    parser.add_argument("--outliers", type=int,
                        default=5, help="Reject any outlier positions, based on last N frames")
    parser.add_argument("--outputFolder", type=str, default="",
                        help="Save processed images to this folder")
    parser.add_argument('--extraOpt', dest='extraOpt', help="Optimise best position better",
                        default=False, action='store_true')
    parser.add_argument('--cuda', dest='cuda', help="Use OpenCV CUDA Extensions",
                        default=False, action='store_true')
    parser.add_argument("--tagFamily", type=str, default="tagStandard41h12",
                        help="Apriltag family")
    parser.add_argument("--multiCamera", type=str, default=None,
                        help="multiple cameras using the specified yaml file")
    parser.add_argument('--tagEngine', dest='tagEngine', help="Tag detector engine",
                        default='PyAprilTags', choices=['OpenCV', 'PyAprilTags', 'JetsonVPI'])
    args = parser.parse_args()

    main(args)
