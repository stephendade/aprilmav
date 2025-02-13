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

from pyapriltags import Detector
from modules.common import loadCameras
from modules.geo import tagDB
from modules.saveStream import saveThread

exit_event = threading.Event()


def signal_handler(signum, frame):
    """
    Signal handler for exit
    """
    exit_event.set()


def main(args):
    print("Initialising")

    signal.signal(signal.SIGINT, signal_handler)

    # Open camera settings and load camera(s)
    CAMERAS = loadCameras(None, args.camera, args.inputFolder, args.jetson)

    # allow the camera to warmup
    time.sleep(2)

    at_detector = Detector(searchpath=['apriltags3py/apriltags/lib', 'apriltags3py/apriltags/lib'],
                           families=args.tagFamily,
                           nthreads=max(1, os.cpu_count() - 1),
                           quad_decimate=args.decimation,
                           quad_sigma=0.4,
                           refine_edges=1,
                           decode_sharpening=1,
                           debug=0)

    # All tags live in here
    tagPlacement = tagDB(slidingWindow=args.averaging, extraOpt=args.extraOpt)

    # how many loops
    loops = CAMERAS[0].getNumberImages() if CAMERAS[0].getNumberImages() else args.loop

    # Start save image thread, if desired
    threadSave = None
    if args.outputFolder != "":
        threadSave = saveThread(args.outputFolder, exit_event)
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
    fig = None
    axMap = None
    axHeight = None
    lineVehicle = None
    lineTag = None
    lineHeight = []
    coordsX = []
    coordsY = []
    coordsZ = []
    coordsFile = []
    if args.gui:
        print("plotting")
        import matplotlib.pyplot as plt
        # Create a figure containing a single axes.
        fig, (axMap, axHeight) = plt.subplots(2)
        axMap.set_xlim(-4, 4)
        axMap.set_ylim(-4, 4)
        axMap.grid()
        axHeight.set_xlim(0, loops-1)
        axHeight.set_ylim(-4, 4)
        axMap.set(xlabel='Left (m)', ylabel='Fwd (m)', title='Horizonal Map')
        axHeight.set(xlabel='', ylabel='Up (m)', title='Vehicle Height')
        # Plot some data on the axes.
        lineVehicle, = axMap.plot(coordsX, coordsY)
        lineTag, = axMap.plot([], [], 'bs')
        lineHeight, = axHeight.plot([], [])

        plt.show(block=False)

    for i in range(loops):
        print("--------------------------------------")

        # grab an image from the camera
        startTime = time.time()
        file = CAMERAS[0].getFileName()
        (imageBW, timestamp) = CAMERAS[0].getImage()

        # we're out of images
        if imageBW is None:
            break

        # AprilDetect, after accounting for distortion (if fisheye)
        tags = at_detector.detect(
            imageBW, True, CAMERAS[0].KFlat, args.tagSize/1000)

        # add any new tags to database, or existing one to duplicates
        tagsused = []
        for tag in tags:
            if tag.pose_err < args.maxError*1e-8:
                tagsused.append(tag)
                tagPlacement.addTag(tag, CAMERAS[0].T_CamtoVeh)

        tagPlacement.getBestTransform(timestamp)

        if file:
            print("File: {0} ({1}/{2})".format(file, i, loops))

        posR = tagPlacement.reportedPos
        rotR = tagPlacement.reportedRot
        rotD = numpy.rad2deg(tagPlacement.reportedRot)
        speed = tagPlacement.reportedVelocity

        with open(args.outFile, "a", encoding="utf-8") as outFile:
            outFile.write("{0},".format(file))
            outFile.write("{0:.4f},{1:.4f},{2:.4f},".format(posR[0], posR[1], posR[2]))
            outFile.write("{0:.3f},{1:.3f},{2:.3f},".format(rotR[0], rotR[1], rotR[2]))
            outFile.write("{0:.3f},{1:.3f},{2:.3f},".format(speed[0], speed[1], speed[2]))
            outFile.write("{0},".format(' '.join(str(id) for id in tagPlacement.tagDuplicatesT.keys())))
            outFile.write("{0:.6f}\n".format(timestamp))

        # Update the live graph
        if args.gui:
            coordsX.append(posR[2])
            coordsY.append(posR[0])
            coordsZ.append(posR[1])
            coordsFile.append(i)
            lineVehicle.set_xdata(coordsX)
            lineVehicle.set_ydata(coordsY)
            lineTag.set_xdata(tagPlacement.getTagPoints(2))
            lineTag.set_ydata(tagPlacement.getTagPoints(0))
            lineHeight.set_xdata(coordsFile)
            lineHeight.set_ydata(coordsZ)
            plt.draw()
            fig.canvas.flush_events()

        print("Time to capture, detect, localise = {0:.2f} ms, {2}/{1} tags".format(1000*(time.time() - startTime),
                                                                                    len(tags),
                                                                                    len(tagPlacement.tagDuplicatesT)))

        # Send to save thread
        if threadSave:
            threadSave.save_queue.put((imageBW, os.path.join(
                ".", args.outputFolder, "processed_{:04d}.png".format(i)), posR, rotD, tags))

        tagPlacement.newFrame()

        if exit_event.is_set():
            break

    # close camera
    for CAMERA in CAMERAS:
        CAMERA.close()

    exit_event.set()

    # Tags
    if args.gui:
        for tagid, tag in tagPlacement.getTagdb().items():
            axMap.annotate("T{0} ({1:.3f})m".format(
                tagid, tag[1, 3]), (tag[2, 3]+0.1, tag[0, 3]+0.1))
        print("Waiting for plot window to be closed")
        plt.show()


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
                        help="Output tag data to this file")
    parser.add_argument('--gui', dest='gui',
                        default=False, action='store_true')
    parser.add_argument("--decimation", type=int,
                        default=2, help="Apriltag decimation")
    parser.add_argument("--averaging", type=int,
                        default=5, help="Use moving average of N frames")
    parser.add_argument("--outputFolder", type=str, default="",
                        help="Save processed images to this folder")
    parser.add_argument('--extraOpt', dest='extraOpt', help="Optimise best position better",
                        default=False, action='store_true')
    parser.add_argument('--jetson', dest='jetson', help="Use Jetson hardware acceleration",
                        default=False, action='store_true')
    parser.add_argument("--tagFamily", type=str, default="tagStandard41h12",
                        help="Apriltag family")
    args = parser.parse_args()

    main(args)
