'''
Camera Interfacing for a directory of images
'''

import os
import time
import cv2
from .cameraBase import cameraBase


class FileCamera(cameraBase):
    '''A Camera setup and capture class'''

    def __init__(self, camParams, folder=".", use_jetson=False):
        '''Initialise the camera, based on a dict of settings'''
        super().__init__(camParams, use_jetson)

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

        try:
            basename = os.path.splitext(os.path.basename(self.images[0]))[0]
            timestamp = int(basename)/1000
        except ValueError:
            timestamp = time.time()
        img = cv2.imread(self.images.pop(0), cv2.IMREAD_GRAYSCALE)
        # img = cv2.fastNlMeansDenoising(img,None, 3, 5, 17)

        if not get_raw:
            img = self.maybeDoFishEyeConversion(img)

        return (img, timestamp)

    def getFileName(self):
        '''Get current file in camera'''
        return self.images[0]

    def close(self):
        ''' close the camera'''
        super().close()
        return
