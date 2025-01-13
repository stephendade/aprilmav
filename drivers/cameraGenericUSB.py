'''
Camera Interfacing for a generic USB Camera via OpenCV
'''

import time
import numpy
import cv2
from .cameraBase import cameraBase


class camera(cameraBase):
    '''A Camera setup and capture class for a USB Camera'''

    def __init__(self, camParams):
        '''Initialise the camera, based on a dict of settings'''
        super().__init__(camParams)

        self.camera = cv2.VideoCapture(0)
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH,
                        self.camParams['resolution'][0])
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT,
                        self.camParams['resolution'][1])

        time.sleep(2)

    def getNumberImages(self):
        '''Get number of loaded images'''
        return None

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
