#!/usr/bin/env python3
'''
This script localises any visible Apriltags and
calulates the camera's rotation and position as a
transformation matrix T_CamToWorld, relative to it's
starting location.

Requires a Raspberry Pi Camera V2.

Note camera settings are specific to the camera model and
settings. Use cameracal.py to generate new settings and put
into camera.yaml
'''
import time
import math
import numpy
import cv2
import yaml
import argparse

from dt_apriltags import Detector
from transforms3d.euler import mat2euler
from lib.geo import tagDB

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("-tagSize", type=int, default=200, help="Apriltag size in mm")
    parser.add_argument("-camera", type=str, default="PiCamV2FullFoV", help="Camera profile in camera.yaml")
    parser.add_argument("-loop", type=int, default=20, help="Capture and process this many frames")
    parser.add_argument("-maxerror", type=int, default=500, help="Maximum pose error to use, in n*E-8 units")
    parser.add_argument("-folder", type=str, default=None, help="Use a folder of images instead of camera")
    parser.add_argument("-outfile", type=str, default="positions.csv", help="Output tag data to this file")
    parser.add_argument('--gui', dest='gui', default=True, action='store_true')
    args = parser.parse_args()
    
    print("Initialising")

    # Open camera settings
    with open('camera.yaml', 'r') as stream:
        parameters = yaml.load(stream, Loader=yaml.FullLoader)
    camParams = parameters[args.camera]
        
    # initialize the camera
    if args.folder == None:
        from lib import cameraPi
        camera = cameraPi.cameraPi(parameters[args.camera])
    else:
        from lib import cameraFile
        camera = cameraFile.FileCamera(args.folder)

    # allow the camera to warmup
    time.sleep(2)

    at_detector = Detector(searchpath=['apriltags3py/apriltags/lib', 'apriltags3py/apriltags/lib'],
                           families='tagStandard41h12',
                           nthreads=3,
                           quad_decimate=1.0,
                           quad_sigma=0.4,
                           refine_edges=1,
                           decode_sharpening=0,
                           debug=0)

    # All tags live in here
    tagPlacement = tagDB(0, 0, 0)
    
    # how many loops
    loops = camera.getNumberImages() if camera.getNumberImages() else args.loop

    print("Starting {0} image capture and process...".format(loops))
    
    outfile = open(args.outfile,"w+")
    outfile.write("{0},{1},{2},{3}\n".format("Filename", "PosX (left)", "PosY (up)", "PosZ (fwd)"))
    
    #GUI
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
        fig, (axMap, axHeight) = plt.subplots(2)  # Create a figure containing a single axes.
        axMap.set_xlim(-4, 4)
        axMap.set_ylim(-4, 4)
        axMap.grid()
        axHeight.set_xlim(0, loops-1)
        axHeight.set_ylim(-4, 4)
        axMap.set(xlabel='Left (m)', ylabel='Fwd (m)', title='Horizonal Map')
        axHeight.set(xlabel='', ylabel='Up (m)', title='Vehicle Height')
        lineVehicle, = axMap.plot(coordsX, coordsY)  # Plot some data on the axes.
        lineTag, = axMap.plot([], [], 'bs')
        lineHeight, = axHeight.plot([], [])

        plt.show(block = False)

    for i in range(loops):
        print("--------------------------------------")
        
        myStart = time.time()

        # grab an image from the camera
        file = camera.getFileName()
        imageBW = camera.getImage()
        
        # we're out of images
        if imageBW is None:
            break
            
        # AprilDetect
        tags = at_detector.detect(imageBW, True, camParams['cam_params'], args.tagSize/1000)

        # add any new tags to database, or existing one to duplicates
        tagsused = 0
        for tag in tags:
            if tag.pose_err < args.maxerror*1e-8:
                tagsused += 1
                tagPlacement.addTag(tag)
                              
        tagPlacement.getBestTransform()

        #x = left
        print("File {1} with {0}/{2} tags".format(tagsused, file, len(tags)))
        
        posn = tagPlacement.getCurrentPosition()
        outfile.write("{0},{1:.3f},{2:.3f},{3:.3f}\n".format(file, posn[0], posn[1], posn[2]))
        
        #Update the live graph
        coordsX.append(posn[2])
        coordsY.append(posn[0])
        coordsZ.append(posn[1])
        coordsFile.append(i)
        lineVehicle.set_xdata(coordsX)
        lineVehicle.set_ydata(coordsY)
        lineTag.set_xdata(tagPlacement.getTagPoints(2))
        lineTag.set_ydata(tagPlacement.getTagPoints(0))
        lineHeight.set_xdata(coordsFile)
        lineHeight.set_ydata(coordsZ)
        plt.draw()
        fig.canvas.flush_events()

        #print("Time to capture, detect and localise = {0:.3f} sec, using {2}/{1} tags".format(time.time() - myStart, len(tags), len(tagPlacement.tagDuplicatesT)))
        
        tagPlacement.newFrame()
                
                        
        #cv2.imwrite("detect_{0}.jpg".format(i), image)

# Tags
for tagid, tag in tagPlacement.getTagdb().items():
    axMap.annotate("T{0} ({1:.3f})m".format(tagid, tag[1,3]), (tag[2,3]+0.1, tag[0,3]+0.1))
print("Waiting for plot window to be closed")
plt.show()