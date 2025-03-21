'''
Camera Interfacing for a GStreamer pipeline via OpenCV
'''

import time
import cv2
from .cameraBase import cameraBase


class camera(cameraBase):
    '''A Camera setup and capture class for a GStreamer source via OpenCV'''

    def __init__(self, camParams, use_jetson=False, camName=""):
        '''Initialise the camera, based on a dict of settings'''
        super().__init__(camParams, use_jetson, camName)

        # Check if OpenCV has GStreamer enabled
        if 'GStreamer:                   YES' not in cv2.getBuildInformation():
            print("Error: OpenCV is not built with GStreamer support")
            return

        # Construct the GStreamer pipeline string
        gst_pipeline = (
            f"{self.camParams['model']} ! "
            f"video/x-raw, width=(int){self.camParams['resolution'][0]}, height=(int){self.camParams['resolution'][1]}, format=(string)BGRx ! "
            f"videoconvert ! video/x-raw, format=(string)BGR ! "
            f"appsink"
        )

        print(gst_pipeline)

        self.camera = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)

        if not self.camera.isOpened():
            print("Error: Unable to open camera with GStreamer pipeline")
            return

        self.frame = None
        self.pro_image = None

        time.sleep(2)

    def getNumberImages(self):
        '''Placeholder for a method to get the number of images captured'''
        pass

    def getFileName(self):
        '''Get current file in camera'''
        return None

    def getImage(self, get_raw=False):
        ''' Capture a single image from the Camera '''

        timestamp = time.time()
        return_value, self.frame = self.camera.read()
        timestamp_capture = time.time()

        if self.use_jetson:
            # Pre-allocation of GPU memory
            if self.pro_image is None:
                self.pro_image = cv2.cuda_GpuMat()

            # it's actually quicker to convert to grayscale on the CPU and upload smaller
            # image to GPU
            self.pro_image.upload(cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY))
        else:
            self.pro_image = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)

        if not get_raw:
            self.pro_image = self.maybedoImageEnhancement(self.pro_image)
            self.pro_image = self.maybeDoFishEyeConversion(self.pro_image)
        timestamp_rectify = time.time()

        # Download the result back to the CPU
        if self.use_jetson:
            imageBW = self.pro_image.download()
        else:
            imageBW = self.pro_image

        return (imageBW, timestamp, timestamp_capture - timestamp, timestamp_rectify - timestamp_capture)

    def close(self):
        ''' close the camera'''
        super().close()
        del self.camera
