'''
A base class for the camera drivers
'''

import numpy
import cv2
from pyapriltags import Detector


class cameraBase:
    '''A Camera setup and capture base class'''

    def __init__(self, camParams, aprildecimation=1, aprilthreads=1, tagSize=0.1):
        '''Initialise the camera, based on a dict of settings'''

        self.camParams = camParams

        if camParams:
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
        else:
            self.fisheye = False
            self.dim1 = None
            self.map1 = None
            self.map2 = None

        # Apriltag detector
        self.at_detector = Detector(searchpath=['apriltags3py/apriltags/lib', 'apriltags3py/apriltags/lib'],
                                    families='tagStandard41h12',
                                    nthreads=aprilthreads,
                                    quad_decimate=aprildecimation,
                                    quad_sigma=0.0,
                                    refine_edges=1,
                                    decode_sharpening=0.25,
                                    debug=0)
        self.tagSize = tagSize

    def getNumberImages(self):
        '''Get number of loaded images'''
        return None

    def getFileName(self):
        '''Get current file in camera'''
        return None

    def getApriltagsandImage(self):
        '''Do image capture, undistort and apriltag detection'''
        (img, timestamp) = self.getImage()

        if img is None:
            return []
        return (self.at_detector.detect(
                img, True, self.camParams['cam_params'], self.tagSize), timestamp)

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
        else:
            return image

    def getImage(self):
        ''' Capture a single image from the Camera '''
        return None, None

    def close(self):
        ''' close the camera'''
        pass
