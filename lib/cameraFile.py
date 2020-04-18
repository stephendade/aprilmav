'''
Camera Interfacing for a directory of images
'''

import glob
import cv2
import os

class FileCamera:
    '''A Camera setup and capture class for the PiCamV2'''
    
    def __init__(self, dir="."):
        '''Initialise the camera, based on a dict of settings'''
        
        self.images = glob.glob(os.path.join(dir, "*.jpg"))
        self.images.sort()
        
        print("FileCamera: Found {0} images in folder".format(len(self.images)))
        
    def getNumberImages(self):
        '''Get number of loaded images'''
        return len(self.images)
        
    def getImage(self):
        ''' Capture a single image from the Camera '''
        
        if len(self.images) == 0:
            print("Warning: FileCamera out of images")
            return None
        
        img = cv2.imread(self.images.pop(0), cv2.IMREAD_GRAYSCALE)
        #img = cv2.fastNlMeansDenoising(img,None, 3, 5, 17)
        
        return img
    
    def getFileName(self):
        '''Get current file in camera'''
        return self.images[0]
        
    def close(self):
        ''' close the camera'''
        pass 