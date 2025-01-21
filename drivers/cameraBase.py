'''
A base class for the camera drivers
'''

import numpy
import cv2

'''
Need to install jetson-utils (https://github.com/dusty-nv/jetson-utils) and
sudo apt install nvidia-cuda-dev
'''
try:
    import vpi
    import jetson_utils
except ImportError:
    pass


class cameraBase:
    '''A Camera setup and capture base class'''

    def __init__(self, camParams, use_jetson=False):
        '''Initialise the camera, based on a dict of settings'''

        self.use_jetson = use_jetson

        if self.use_jetson:
            self.cudaFrame = jetson_utils.cudaImage(camParams['resolution'][0],
                                                    camParams['resolution'][1],
                                                    'gray8')

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
                if self.use_jetson:
                    self.D = camParams['cam_paramsD']
                else:
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
            self.dim1 = image.shape[:2][::-1]
            if self.use_jetson:
                # Create an uniform grid
                grid = vpi.WarpGrid(self.dim1)

                # Create undistort warp map from the calibration parameters and the grid
                self.map1 = vpi.WarpMap.fisheye_correction(grid, K=self.K[0:2, :], X=numpy.eye(3, 4),
                                                           coeffs=self.D, mapping=vpi.FisheyeMapping.EQUIDISTANT)
            else:
                # Only need to get mapping at first frame
                # dim1 is the dimension of input image to un-distort
                self.dim1 = image.shape[:2][::-1]
                self.map1, self.map2 = cv2.fisheye.initUndistortRectifyMap(
                    self.K, self.D, numpy.eye(3), self.K, self.dim1, cv2.CV_16SC2)

        if self.fisheye:
            if self.use_jetson:
                self.cudaFrame = vpi.asimage(numpy.uint8(jetson_utils.cudaFromNumpy(image)))
                # do undistortion in CUDA
                with vpi.Backend.CUDA:
                    imgCorrected = self.cudaFrame.remap(self.map1, interp=vpi.Interp.CATMULL_ROM).convert(vpi.Format.U8)
                    imageUndistort = numpy.asarray(imgCorrected.cpu())
                return imageUndistort
            else:
                imageUndistort = cv2.remap(image, self.map1, self.map2, interpolation=cv2.INTER_LINEAR,
                                           borderMode=cv2.BORDER_CONSTANT)
            return imageUndistort
        else:
            return image

    def getImage(self, get_raw=False):
        ''' Capture a single image from the Camera '''
        return None

    def close(self):
        ''' close the camera'''
        if self.use_jetson:
            # close the camera
            vpi.clear_cache()
