'''
A base class for the camera drivers
'''

import time
import numpy
import cv2

from transforms3d.euler import euler2mat

from modules.aprilDetect import aprilDetect, tagEngines


class cameraBase:
    '''A Camera setup and capture base class'''

    def __init__(self, camParams, tagSize, tagFamily, decimation, tagEngine, use_cuda=False, camName=""):
        '''Initialise the camera, based on a dict of settings'''

        self.use_cuda = use_cuda
        self.camName = camName
        self.doEnhancement = camParams['doEnhancement'] if 'doEnhancement' in camParams else False
        self.time_capture = 0
        self.time_rectify = 0

        self.imageBW = None
        self.image_timestamp = 0

        try:
            if camParams['resolution'][0] % 16 != 0 or camParams['resolution'][1] % 16 != 0:
                print("Error: Camera resolution must be divisible by 16")
                return
        except TypeError:
            # for camercal, camParams['resolution'] is not defined
            pass

        self.camParams = camParams

        if 'tagEngine' in camParams and camParams['tagEngine'] is not None:
            self.at_detector = aprilDetect(tagSize, tagFamily, decimation, camParams['tagEngine'])
        elif tagEngine is not None:
            self.at_detector = aprilDetect(tagSize, tagFamily, decimation, tagEngine)
        else:
            self.at_detector = None
            self.tags = None
            self.time_detect = 0

        try:
            # Need to reconstruct K and D for each camera
            self.K = numpy.zeros((3, 3))
            self.D = numpy.zeros((4, 1))
            self.fisheye = camParams['fisheye']
            self.dim1 = None
            self.map1 = None
            self.map2 = None
            self.K[0, 0] = camParams['cam_params'][0]
            self.K[1, 1] = camParams['cam_params'][1]
            self.K[0, 2] = camParams['cam_params'][2]
            self.K[1, 2] = camParams['cam_params'][3]
            self.K[2, 2] = 1
            self.KFlat = camParams['cam_params']
            if camParams['fisheye']:
                self.D[0][0] = camParams['cam_paramsD'][0]
                self.D[1][0] = camParams['cam_paramsD'][1]
                self.D[2][0] = camParams['cam_paramsD'][2]
                self.D[3][0] = camParams['cam_paramsD'][3]
        except (KeyError, IndexError, TypeError):
            pass

        # create camera to vehicle transformation matrix
        campos = camParams['positionRelVehicle']
        camrot = camParams['rotationRelVehicle']
        # Convert rotation tuple (Euler angles) to rotation matrix
        rotation_matrix = euler2mat(numpy.deg2rad(camrot[0]), numpy.deg2rad(camrot[1]),
                                    numpy.deg2rad(camrot[2]), axes='sxyz')

        # Construct the transformation matrix
        T_CamtoVeh = numpy.eye(4)
        T_CamtoVeh[0:3, 0:3] = rotation_matrix
        T_CamtoVeh[0:3, 3] = campos

        self.T_CamtoVeh = T_CamtoVeh

    def getNumberImages(self):
        '''Get number of loaded images'''
        return None

    def getFileName(self):
        '''Get current file in camera'''
        return None

    def maybedoImageEnhancement(self, image):
        '''Enhance the image to optimize AprilTag detection'''

        if not self.doEnhancement:
            return image

        # denoise the image via a 5x5 kernal gaussian blur
        # then sharpen the image via subtracting a blurred version
        # From https://www.iaarc.org/publications/fulltext/166_ISARC_2024_Paper_207.pdf
        if self.use_cuda:
            blurred = cv2.cuda.createGaussianFilter(image.type(), -1, (5, 5), 0).apply(image)
            image = cv2.cuda.addWeighted(image, 1.5, blurred, -0.5, 0)
        else:
            blurred = cv2.GaussianBlur(image, (5, 5), 0)
            image = cv2.addWeighted(image, 1.5, blurred, -0.5, 0)
        return image

    def maybeDoFishEyeConversion(self, image):
        '''Convert the image to rectilinear'''

        # Generate the undistorted image mapping if fisheye
        if self.fisheye and self.dim1 is None:
            if self.use_cuda:
                self.dim1 = (image.size()[0], image.size()[1])  # width, height
            else:
                self.dim1 = image.shape[:2][::-1]
            if self.use_cuda:
                gpu_map1, gpu_map2 = cv2.fisheye.initUndistortRectifyMap(
                    self.K, self.D, numpy.eye(3), self.K, self.dim1, cv2.CV_32FC1)
                self.map1 = cv2.cuda_GpuMat()
                self.map1.upload(gpu_map1)
                self.map2 = cv2.cuda_GpuMat()
                self.map2.upload(gpu_map2)
            else:
                self.map1, self.map2 = cv2.fisheye.initUndistortRectifyMap(
                    self.K, self.D, numpy.eye(3), self.K, self.dim1, cv2.CV_16SC2)
        if self.fisheye:
            if self.use_cuda:
                imageUndistort = cv2.cuda.remap(image, self.map1, self.map2, interpolation=cv2.INTER_LINEAR,
                                                borderMode=cv2.BORDER_CONSTANT)
            else:
                imageUndistort = cv2.remap(image, self.map1, self.map2, interpolation=cv2.INTER_LINEAR,
                                           borderMode=cv2.BORDER_CONSTANT)
            return imageUndistort
        return image

    def doDetect(self):
        '''Detect tags in the imageBW using the at_detector'''
        if self.at_detector is None:
            print("Error: No tag detector available")
            return
        if self.imageBW is None:
            print("Error: No image to detect tags in")
            return
        detectStart = time.time()
        if self.at_detector.tagEngine == tagEngines.OpenCV:
            self.tags = self.at_detector.detect(self.imageBW, self.K)
        else:
            self.tags = self.at_detector.detect(self.imageBW, self.KFlat)
        self.time_detect = time.time() - detectStart

    def getImage(self, get_raw=False):
        ''' Capture a single image from the Camera '''
        return None

    def close(self):
        ''' close the camera'''
        pass
