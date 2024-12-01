'''
Camera Interfacing for the ArduCam UC-580 (OV9281 Global Shutter)
'''

import time
import cv2
from . import arducam_mipicamera as arducam


class camera:
    '''A Camera setup and capture class for the ArduCam UC580'''

    def __init__(self, camParams):
        '''Initialise the camera, based on a dict of settings'''

        if camParams['resolution'][0] % 16 != 0 or camParams['resolution'][1] % 16 != 0:
            print("Error: Camera resolution must be divisible by 16")
            return

        self.camParams = camParams
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

    def getImage(self):
        ''' Capture a single image from the Camera '''

        timestamp = time.time()
        self.frame = self.camera.capture(encoding="i420")
        image = self.frame.as_array.reshape(
            int(self.camParams['resolution'][1]*1.5), self.camParams['resolution'][0])

        # Convert to greyscale and crop
        image = cv2.cvtColor(image, cv2.COLOR_YUV2GRAY_I420)
        imageCrop = image[0:self.camParams['resolution']
                          [1], 0:self.camParams['resolution'][0]]

        # Halve the resolution
        if self.camera.halfres:
            imageCrop = cv2.resize(
                imageCrop, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)

        return (imageCrop, timestamp)

    def close(self):
        ''' close the camera'''
        self.camera.close_camera()
        del self.frame
