'''
Camera Interfacing for a generic USB Camera via OpenCV
'''

import time
import cv2
from .cameraBase import cameraBase


class camera(cameraBase):
    '''A Camera setup and capture class for a USB Camera'''

    def __init__(self, camParams, use_jetson=False, camName=""):
        '''Initialise the camera, based on a dict of settings'''
        super().__init__(camParams, use_jetson, camName)

        self.camera = cv2.VideoCapture()
        self.camera.open(camParams['cameraPath'])

        # read one frame to start camera.
        self.camera.read()

        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH,
                        self.camParams['resolution'][0])
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT,
                        self.camParams['resolution'][1])
        self.camera.set(cv2.CAP_PROP_FPS, self.camParams['fps'])
        self.camera.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
        self.camera.set(cv2.CAP_PROP_GAIN, self.camParams['gain'])
        self.camera.set(cv2.CAP_PROP_EXPOSURE, self.camParams['exposure'])
        self.camera.set(cv2.CAP_PROP_BACKLIGHT, self.camParams['backlight'])

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
        return_value, image = self.camera.read()

        imageBW = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        if not get_raw:
            imageBW = self.doImageEnhancement(imageBW)
            imageBW = self.maybeDoFishEyeConversion(imageBW)

        return (imageBW, timestamp)

    def close(self):
        ''' close the camera'''
        super().close()
        self.camera.release()
        del self.camera
