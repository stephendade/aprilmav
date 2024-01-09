'''
Camera Interfacing for Libcamera
'''

from picamera2 import Picamera2
import cv2
import time


class camera:
    '''A Camera setup and capture class for Libcamera'''
    
    def __init__(self, camParams):
        '''Initialise the camera, based on a dict of settings'''
        
        # find the camera by name
        self.camera = None
        for cam in Picamera2.global_camera_info():
            if cam['Model'] == camParams['model']:
                self.camera = Picamera2(cam['Num'])
                break

        # Couldn't find the camera
        if not self.camera:
            print("Error: Could not find {0}".format(camParams['cam_name']))
            return

        self.camParams = camParams
        
        self.camera.rotation = camParams['rotation']
        self.frame = None
                
        # Set camera settings
        config = self.camera.create_still_configuration({"format": 'BGR888', "size": (self.camParams['resolution'][0], 
                                                                                      self.camParams['resolution'][1])})
        self.camera.configure(config)
        self.camera.start()
        
        time.sleep(1)
        
    def getNumberImages(self):
        '''Get number of loaded images'''
        return None

    def getFileName(self):
        '''Get current file in camera'''
        return None
        
    def getImage(self):
        ''' Capture a single image from the Camera '''
        
        self.frame = self.camera.capture_array()
        #image = self.frame.as_array.reshape(int(self.camParams['resolution'][1]*1.5), self.camParams['resolution'][0])
        
        # Convert to greyscale and crop
        image = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
        #imageCrop = image[0:self.camParams['resolution'][1], 0:self.camParams['resolution'][0]]
        
        # Rotate if required
        if self.camParams['rotation'] == 180:
            image = cv2.rotate(image, cv2.ROTATE_180)
        if self.camParams['rotation'] == 90:
            image = cv2.rotate(image, cv2.ROTATE_90_CLOCKWISE)
        if self.camParams['rotation'] == 270:
            image = cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE)            
        return image
        
    def close(self):
        ''' close the camera'''
        #self.camera.close_camera()
        del self.frame
