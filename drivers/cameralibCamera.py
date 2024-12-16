'''
Camera Interfacing for Libcamera
4-7ms capture time for RPi GS Camera
'''

import time
import cv2
from picamera2 import Picamera2


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
        self.frame = None

        # Set camera settings
        config = self.camera.create_still_configuration({"size": (self.camParams['resolution'][0],
                                                        self.camParams['resolution'][1])},
                                                        controls={
                                                            'FrameRate': 60},
                                                        buffer_count=2)
        self.camera.configure(config)
        self.camera.start()

        # Run for a second to get a reasonable "middle" exposure level.
        time.sleep(1)
        metadata = self.camera.capture_metadata()
        self.camera.stop()
        controls = {"ExposureTime": 5000,    # microseconds. So 5ms
                    "AnalogueGain": int(metadata["AnalogueGain"]),
                    "ColourGains": metadata["ColourGains"],
                    'FrameRate': 60}

        # Set camera settings with new gains
        config = self.camera.create_still_configuration({"size": (self.camParams['resolution'][0],
                                                        self.camParams['resolution'][1])},
                                                        controls=controls,
                                                        buffer_count=2)
        self.camera.configure(config)
        self.camera.start()

    def getNumberImages(self):
        '''Get number of loaded images'''
        return None

    def getFileName(self):
        '''Get current file in camera'''
        return None

    def getImage(self):
        ''' Capture a single image from the Camera '''

        timestamp = time.time()
        self.frame = self.camera.capture_array()

        # Convert to greyscale
        image = cv2.cvtColor(self.frame, cv2.COLOR_RGB2GRAY)

        return (image, timestamp)

    def close(self):
        ''' close the camera'''
        # self.camera.close_camera()
        del self.frame