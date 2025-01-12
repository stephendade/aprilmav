'''
Camera Interfacing for the Raspberry Pi Camera (V2)
'''

import time
import numpy
import cv2

from picamera import PiCamera


class camera:
    '''A Camera setup and capture class for the PiCamV2'''

    def __init__(self, camParams):
        '''Initialise the camera, based on a dict of settings'''

        if camParams['resolution'][0] % 16 != 0 or camParams['resolution'][1] % 16 != 0:
            print("Error: Camera resolution must be divisible by 16")
            return

        self.camParams = camParams
        self.camera = PiCamera(resolution=camParams['resolution'], framerate=camParams['framerate'],
                               sensor_mode=camParams['sensor_mode'])

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

    def getImage(self):
        ''' Capture a single image from the Camera '''

        timestamp = time.time()
        self.camera.capture(self.image, format="bgr",
                            use_video_port=self.camParams['use_video_port'])

        # and convert to OpenCV greyscale format
        self.image = self.image.reshape(
            (self.camera.resolution[1], self.camera.resolution[0], 3))
        self.image = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)
        
        # Generate the undistorted image mapping if fisheye
        if self.fisheye and self.dim1 is None:
            # Only need to get mapping at first frame
            # dim1 is the dimension of input image to un-distort
            self.dim1 = self.image.shape[:2][::-1]
            self.map1, self.map2 = cv2.fisheye.initUndistortRectifyMap(
                self.K, self.D, numpy.eye(3), self.K, self.dim1, cv2.CV_16SC2)

        return (self.image, timestamp)

    def close(self):
        ''' close the camera'''
        self.camera.close()
