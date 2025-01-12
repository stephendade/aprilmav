'''
Camera Interfacing for a directory of images
'''

import os
import time
import numpy
import cv2


class FileCamera:
    '''A Camera setup and capture class'''

    def __init__(self, camParams, folder="."):
        '''Initialise the camera, based on a dict of settings'''

        self.images = [
            os.path.join(folder, file)
            for file in os.listdir(folder)
            if file.endswith(('.png', '.jpg'))
        ]
        self.images.sort()

        # Need to reconstruct K and D for each camera
        if camParams:
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

        print("FileCamera: Found {0} images in folder".format(
            len(self.images)))

    def getNumberImages(self):
        '''Get number of loaded images'''
        return len(self.images)

    def getImage(self):
        ''' Capture a single image from the Camera and time of capture (sec since epoch)'''

        if len(self.images) == 0:
            print("Warning: FileCamera out of images")
            return None

        try:
            basename = os.path.splitext(os.path.basename(self.images[0]))[0]
            timestamp = int(basename)/1000
        except ValueError:
            timestamp = time.time()
        img = cv2.imread(self.images.pop(0), cv2.IMREAD_GRAYSCALE)
        # img = cv2.fastNlMeansDenoising(img,None, 3, 5, 17)

        # Generate the undistorted image mapping if fisheye
        if self.fisheye and self.dim1 is None:
            # Only need to get mapping at first frame
            # dim1 is the dimension of input image to un-distort
            self.dim1 = img.shape[:2][::-1]
            self.map1, self.map2 = cv2.fisheye.initUndistortRectifyMap(
                self.K, self.D, numpy.eye(3), self.K, self.dim1, cv2.CV_16SC2)

        return (img, timestamp)

    def getFileName(self):
        '''Get current file in camera'''
        return self.images[0]

    def close(self):
        ''' close the camera'''
        return
