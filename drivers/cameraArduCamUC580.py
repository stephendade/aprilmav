'''
Camera Interfacing for the ArduCam UC-580 (OV9281 Global Shutter)
'''

import time
import cv2
from . import arducam_mipicamera as arducam
from .cameraBase import cameraBase


class camera(cameraBase):
    '''A Camera setup and capture class for the ArduCam UC580'''

    def __init__(self, camParams, tagSize, tagFamily, decimation, tagEngine, use_cuda=False, camName=""):
        '''Initialise the camera, based on a dict of settings'''
        super().__init__(camParams, tagSize, tagFamily, decimation, tagEngine, use_cuda, camName)

        self.camera = arducam.mipi_camera()
        self.camera.halfres = camParams['halfres']
        self.frame = None

        self.V4L2_CID_EXPOSURE = 9963793

        # Set camera settings
        self.camera.init_camera()
        self.camera.set_resolution(
            self.camParams['resolution'][0], self.camParams['resolution'][1])
        self.camera.set_control(self.V4L2_CID_EXPOSURE, 600)

        # fmt = self.camera.get_format()
        # print("Camera format is {}".format(fmt))

        time.sleep(1)

    def getNumberImages(self):
        '''Get number of loaded images'''
        return None

    def getFileName(self):
        '''Get current file in camera'''
        return None

    def getImage(self, get_raw=False):
        ''' Capture a single image from the Camera '''

        self.image_timestamp = time.time()
        self.frame = self.camera.capture(encoding="i420")
        image = self.frame.as_array.reshape(
            int(self.camParams['resolution'][1]*1.5), self.camParams['resolution'][0])
        timestamp_capture = time.time()

        # Convert to greyscale and crop
        image = cv2.cvtColor(image, cv2.COLOR_YUV2GRAY_I420)
        imageCrop = image[0:self.camParams['resolution']
                          [1], 0:self.camParams['resolution'][0]]

        # Halve the resolution
        if self.camera.halfres:
            self.imageBW = cv2.resize(
                imageCrop, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)

        if not get_raw:
            self.imageBW = self.maybedoImageEnhancement(self.imageBW)
            self.imageBW = self.maybeDoFishEyeConversion(self.imageBW)
        timestamp_rectify = time.time()

        self.time_capture = timestamp_capture - self.image_timestamp
        self.time_rectify = timestamp_rectify - timestamp_capture

    def close(self):
        ''' close the camera'''
        super().close()
        self.camera.close_camera()
        del self.frame
