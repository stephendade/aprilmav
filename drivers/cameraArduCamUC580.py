'''
Camera Interfacing for the ArduCam UC-580 (OV9281 Global Shutter)
'''

import time
import numpy
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

        # Need to reconstruct K and D for each camera
        self.K = numpy.zeros((3, 3))
        self.D = numpy.zeros((4, 1))
        self.fisheye = camParams['fisheye']
        self.dim1 = None
        self.map1 = None
        self.map2 = None
        if camParams['fisheye']:
            self.K[0, 0] = camParams['cam_params'][0]
            self.K[1, 1] = camParams['cam_params'][1]
            self.K[0, 2] = camParams['cam_params'][2]
            self.K[1, 2] = camParams['cam_params'][3]
            self.K[2, 2] = 1
            self.D[0][0] = camParams['cam_paramsD'][0]
            self.D[1][0] = camParams['cam_paramsD'][1]
            self.D[2][0] = camParams['cam_paramsD'][2]
            self.D[3][0] = camParams['cam_paramsD'][3]

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

        # Generate the undistorted image mapping if fisheye
        if self.fisheye and self.dim1 is None:
            # Only need to get mapping at first frame
            # dim1 is the dimension of input image to un-distort
            self.dim1 = imageCrop.shape[:2][::-1]
            self.map1, self.map2 = cv2.fisheye.initUndistortRectifyMap(
                self.K, self.D, numpy.eye(3), self.K, self.dim1, cv2.CV_16SC2)

        return (imageCrop, timestamp)

    def close(self):
        ''' close the camera'''
        self.camera.close_camera()
        del self.frame
