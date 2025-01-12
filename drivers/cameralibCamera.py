'''
Camera Interfacing for Libcamera
4-7ms capture time for RPi GS Camera
'''

import time
import numpy
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
            print("Error: Could not find {0}".format(camParams['cam_driver']))
            return

        self.camParams = camParams
        self.frame = None

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
            self.D[0][0] = camParams['cam_paramsD'][0]
            self.D[1][0] = camParams['cam_paramsD'][1]
            self.D[2][0] = camParams['cam_paramsD'][2]
            self.D[3][0] = camParams['cam_paramsD'][3]

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

        # Generate the undistorted image mapping if fisheye
        if self.fisheye and self.dim1 is None:
            # Only need to get mapping at first frame
            # dim1 is the dimension of input image to un-distort
            self.dim1 = image.shape[:2][::-1]
            self.map1, self.map2 = cv2.fisheye.initUndistortRectifyMap(
                self.K, self.D, numpy.eye(3), self.K, self.dim1, cv2.CV_16SC2)

        return (image, timestamp)

    def close(self):
        ''' close the camera'''
        # self.camera.close_camera()
        del self.frame
