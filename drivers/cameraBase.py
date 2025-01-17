'''
A base class for the camera drivers
'''

import numpy
import cv2


class cameraBase:
    '''A Camera setup and capture base class'''

    def __init__(self, camParams):
        '''Initialise the camera, based on a dict of settings'''

        try:
            if camParams['resolution'][0] % 16 != 0 or camParams['resolution'][1] % 16 != 0:
                print("Error: Camera resolution must be divisible by 16")
                return
        except TypeError:
            # for camercal, camParams['resolution'] is not defined
            pass

        self.camParams = camParams

        try:
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
        except (KeyError, IndexError, TypeError):
            pass

    def getNumberImages(self):
        '''Get number of loaded images'''
        return None

    def getFileName(self):
        '''Get current file in camera'''
        return None

    def maybeDoFishEyeConversion(self, image):
        '''Convert the image to rectilinear'''

        # Generate the undistorted image mapping if fisheye
        if self.fisheye and self.dim1 is None:
            # Only need to get mapping at first frame
            # dim1 is the dimension of input image to un-distort
            self.dim1 = image.shape[:2][::-1]
            self.map1, self.map2 = cv2.fisheye.initUndistortRectifyMap(
                self.K, self.D, numpy.eye(3), self.K, self.dim1, cv2.CV_16SC2)

        if self.fisheye:
            imageUndistort = cv2.remap(image, self.map1, self.map2, interpolation=cv2.INTER_LINEAR,
                                       borderMode=cv2.BORDER_CONSTANT)
            return imageUndistort

        return image

    def getImage(self, get_raw=False):
        ''' Capture a single image from the Camera '''
        return None

    def close(self):
        ''' close the camera'''
        pass
