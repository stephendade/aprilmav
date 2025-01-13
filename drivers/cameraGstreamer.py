'''
Camera Interfacing for a GStreamer pipeline via OpenCV
'''

import time
import cv2
from .cameraBase import cameraBase


class camera(cameraBase):
    '''A Camera setup and capture class for a GStreamer source via OpenCV'''

    def __init__(self, camParams, aprildecimation=1, aprilthreads=1, tagSize=0.1):
        '''Initialise the camera, based on a dict of settings'''
        super().__init__(camParams, aprildecimation, aprilthreads, tagSize)

        if camParams['resolution'][0] % 16 != 0 or camParams['resolution'][1] % 16 != 0:
            print("Error: Camera resolution must be divisible by 16")
            return

        # Check if OpenCV has GStreamer enabled
        if 'GStreamer:                   YES' not in cv2.getBuildInformation():
            print("Error: OpenCV is not built with GStreamer support")
            return

        # Construct the GStreamer pipeline string
        gst_pipeline = (
            f"{self.camParams['model']} ! "
            f"video/x-raw, width=(int){self.camParams['resolution'][0]}, height=(int){self.camParams['resolution'][1]}, format=(string)BGRx ! "
            f"videoconvert ! video/x-raw, format=(string)BGR ! "
            f"appsink"
        )

        print(gst_pipeline)

        self.camera = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)

        if not self.camera.isOpened():
            print("Error: Unable to open camera with GStreamer pipeline")
            return

        time.sleep(2)

    def getNumberImages(self):
        '''Placeholder for a method to get the number of images captured'''
        pass

    def getFileName(self):
        '''Get current file in camera'''
        return None

    def getImage(self):
        ''' Capture a single image from the Camera '''

        timestamp = time.time()
        return_value, image = self.camera.read()
        imageBW = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        imageBW = self.maybeDoFishEyeConversion(imageBW)

        return (imageBW, timestamp)

    def close(self):
        ''' close the camera'''
        del self.camera
