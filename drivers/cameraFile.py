'''
Camera Interfacing for a directory of images
'''

import os
import time
import cv2
from .cameraBase import cameraBase


class FileCamera(cameraBase):
    '''A Camera setup and capture class'''

    def __init__(self, camParams, folder=".", use_cuda=False, camName=""):
        '''Initialise the camera, based on a dict of settings'''
        super().__init__(camParams, use_cuda, camName)

        self.images = [
            os.path.join(folder, file)
            for file in os.listdir(folder)
            if file.endswith(('.png', '.jpg'))
        ]
        self.images.sort()

        print("FileCamera: Found {0} images in folder".format(
            len(self.images)))

    def getNumberImages(self):
        '''Get number of loaded images'''
        return len(self.images)

    def getImage(self, get_raw=False):
        ''' Capture a single image from the Camera and time of capture (sec since epoch)'''

        if len(self.images) == 0:
            print("Warning: FileCamera out of images")
            return None
        startTime = time.time()
        try:
            basename = os.path.splitext(os.path.basename(self.images[0]))[0]
            timestamp = int(basename)/1000
        except ValueError:
            timestamp = time.time()
        img = cv2.imread(self.images.pop(0), cv2.IMREAD_GRAYSCALE)
        timestamp_capture = time.time()
        if self.use_cuda:
            # Upload the image to the GPU
            pro_image = cv2.cuda_GpuMat()
            pro_image.upload(img)
        else:
            pro_image = img

        if not get_raw:
            pro_image = self.maybedoImageEnhancement(pro_image)
            pro_image = self.maybeDoFishEyeConversion(pro_image)
        timestamp_rectify = time.time()

        # Download the result back to the CPU
        if self.use_cuda:
            imageBW = pro_image.download()
        else:
            imageBW = pro_image

        return (imageBW, timestamp, timestamp_capture - startTime, timestamp_rectify - timestamp_capture)

    def getFileName(self):
        '''Get current file in camera'''
        return self.images[0]

    def close(self):
        ''' close the camera'''
        super().close()
        return
