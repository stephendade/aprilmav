'''
Camera Interfacing for Libcamera
4-7ms capture time for RPi GS Camera
'''

import time
import cv2
from picamera2 import Picamera2
from .cameraBase import cameraBase


class camera(cameraBase):
    '''A Camera setup and capture class for Libcamera'''

    def __init__(self, camParams, use_cuda=False, camName=""):
        '''Initialise the camera, based on a dict of settings'''
        super().__init__(camParams, use_cuda, camName)

        # find the camera by name
        self.camera = None
        for cam in Picamera2.global_camera_info():
            if cam['Model'] == camParams['model']:
                self.camera = Picamera2(cam['Num'])
                break

        # Couldn't find the camera
        if not self.camera:
            print("Error: Could not find {0}".format(camParams['cam_driver']))
            return

        self.frame = None

        # Set camera settings
        config = self.camera.create_still_configuration({"size": (self.camParams['resolution'][0],
                                                        self.camParams['resolution'][1])},
                                                        controls={
                                                            'FrameRate': self.camParams['fps']},
                                                        buffer_count=2)
        self.camera.configure(config)
        self.camera.start()

        # Run for a second to get a reasonable "middle" exposure level.
        time.sleep(1)
        metadata = self.camera.capture_metadata()
        self.camera.stop()
        controls = {"ExposureTime": self.camParams['exposure'],    # microseconds
                    "AnalogueGain": int(metadata["AnalogueGain"] * self.camParams['analogue_gain']),
                    "ColourGains": metadata["ColourGains"],
                    'FrameRate': self.camParams['fps']}

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

    def getImage(self, get_raw=False):
        ''' Capture a single image from the Camera '''

        timestamp = time.time()
        self.frame = self.camera.capture_array()
        timestamp_capture = time.time()

        # Convert to greyscale
        image = cv2.cvtColor(self.frame, cv2.COLOR_RGB2GRAY)

        if not get_raw:
            image = self.maybedoImageEnhancement(image)
            image = self.maybeDoFishEyeConversion(image)
        timestamp_rectify = time.time()

        return (image, timestamp, timestamp_capture - timestamp, timestamp_rectify - timestamp_capture)

    def close(self):
        ''' close the camera'''
        # self.camera.close_camera()
        super().close()
        del self.frame
