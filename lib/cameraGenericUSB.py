'''
Camera Interfacing for a generic USB Camera via OpenCV
'''

import cv2
import time


class camera:
    '''A Camera setup and capture class for a USB Camera'''

    def __init__(self, camParams):
        '''Initialise the camera, based on a dict of settings'''

        if camParams['resolution'][0] % 16 != 0 or camParams['resolution'][1] % 16 != 0:
            print("Error: Camera resolution must be divisible by 16")
            return

        self.camParams = camParams

        self.camera = cv2.VideoCapture(0)
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, self.camParams['resolution'][0])
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self.camParams['resolution'][1])

        time.sleep(2)

    def getNumberImages(self):
        '''Get number of loaded images'''
        return None

    def getFileName(self):
        '''Get current file in camera'''
        return None

    def getImage(self):
        ''' Capture a single image from the Camera '''

        return_value, image = self.camera.read()

        # Rotate if required
        if self.camParams['rotation'] == 180:
            image = cv2.rotate(image, cv2.ROTATE_180)
        if self.camParams['rotation'] == 90:
            image = cv2.rotate(image, cv2.ROTATE_90_CLOCKWISE)
        if self.camParams['rotation'] == 270:
            image = cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE)            

        return cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) 

    def close(self):
        ''' close the camera'''
        del self.camera
