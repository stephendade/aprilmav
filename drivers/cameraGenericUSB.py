'''
Camera Interfacing for a generic USB Camera via OpenCV
'''

import time
import numpy
import cv2


class camera:
    '''A Camera setup and capture class for a USB Camera'''

    def __init__(self, camParams):
        '''Initialise the camera, based on a dict of settings'''

        if camParams['resolution'][0] % 16 != 0 or camParams['resolution'][1] % 16 != 0:
            print("Error: Camera resolution must be divisible by 16")
            return

        self.camParams = camParams

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

        # Generate the undistorted image mapping if fisheye
        if self.fisheye and self.dim1 is None:
            # Only need to get mapping at first frame
            # dim1 is the dimension of input image to un-distort
            self.dim1 = imageBW.shape[:2][::-1]
            self.map1, self.map2 = cv2.fisheye.initUndistortRectifyMap(
                self.K, self.D, numpy.eye(3), self.K, self.dim1, cv2.CV_16SC2)

        return (imageBW, timestamp)

    def close(self):
        ''' close the camera'''
        del self.camera
