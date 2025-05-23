'''
Camera Interfacing for the Raspberry Pi Camera (V2)
'''

import time
import numpy
import cv2

from picamera import PiCamera
from .cameraBase import cameraBase


class camera(cameraBase):
    '''A Camera setup and capture class for the PiCamV2'''

    def __init__(self, camParams, tagSize, tagFamily, decimation, tagEngine, camName=""):
        '''Initialise the camera, based on a dict of settings'''
        super().__init__(camParams, tagSize, tagFamily, decimation, tagEngine, use_cuda=False, camName=camName)

        self.camera = PiCamera(resolution=camParams['resolution'], framerate=camParams['framerate'],
                               sensor_mode=camParams['sensor_mode'])

        self.image = numpy.empty(
            (self.camera.resolution[0] * self.camera.resolution[1] * 3,), dtype=numpy.uint8)

        # Set exposure mode to the desired value
        self.camera.exposure_mode = 'sports'

        time.sleep(2)

    def getNumberImages(self):
        '''Get number of loaded images'''
        return None

    def getFileName(self):
        '''Get current file in camera'''
        return None

    def getImage(self, get_raw=False):
        ''' Capture a single image from the Camera '''

        timestamp = time.time()
        self.camera.capture(self.image, format="bgr",
                            use_video_port=self.camParams['use_video_port'])

        # and convert to OpenCV greyscale format
        self.image = self.image.reshape(
            (self.camera.resolution[1], self.camera.resolution[0], 3))
        timestamp_capture = time.time()
        self.image = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)

        if not get_raw:
            self.image = self.maybedoImageEnhancement(self.image)
            self.image = self.maybeDoFishEyeConversion(self.image)
        timestamp_rectify = time.time()

        return (self.image, timestamp, timestamp_capture - timestamp, timestamp_rectify - timestamp_capture)

    def close(self):
        ''' close the camera'''
        self.camera.close()
