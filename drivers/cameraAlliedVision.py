'''
Camera Interfacing for an Allied Vision Camera via Vimba X SDK
Requires the python package to be installed:
pip3 install https://github.com/alliedvision/VmbPy/releases/download/1.1.0/vmbpy-1.1.0-py3-none-linux_aarch64.whl
'''

import time
from queue import Queue
import cv2
from vmbpy import *

from .cameraBase import cameraBase

# Handler for the camera stream
class Handler:
    def __init__(self):
        self.display_queue = Queue(1)

    def get_image(self):
        return self.display_queue.get(True)

    def __call__(self, cam: Camera, stream: Stream, frame: Frame):
        if frame.get_status() == FrameStatus.Complete:
            # print('{} acquired {}'.format(cam, frame), flush=True)

            # Convert frame if it is not already the correct format
            if frame.get_pixel_format() == PixelFormat.Mono8:
                display = frame
            else:
                # This creates a copy of the frame. The original `frame` object can be requeued
                # safely while `display` is used
                display = frame.convert_pixel_format(PixelFormat.Mono8)

            self.display_queue.put(display.as_opencv_image(), True)

        cam.queue_frame(frame)


class camera(cameraBase):
    '''A Camera setup and capture class for a Allied Vision Camera'''
    def __init__(self, camParams, use_cuda=False, camName=""):
        '''Initialise the camera, based on a dict of settings'''
        super().__init__(camParams, use_cuda, camName)

        with VmbSystem.get_instance() as vmb:
            try:
                cam = vmb.get_camera_by_id(self.camParams['cameraName'])
                with cam:
                    # apply camera settings
                    try:
                        cam.ExposureAuto.set('Off')
                    except (AttributeError, VmbFeatureError):
                        print("Can't set ExposureAuto")
                    try:
                        cam.ExposureMode.set('Timed')
                    except (AttributeError, VmbFeatureError):
                        print("Can't set ExposureMode")
                    try:
                        cam.ExposureTime.set(self.camParams['exposure'])  #usec
                    except (AttributeError, VmbFeatureError):
                        print("Can't set exposure")
                    try:
                        cam.GainAuto.set("Off")  #usec
                    except (AttributeError, VmbFeatureError):
                        print("Can't set GainAuto")
                    try:
                        cam.Gain.set(self.camParams['gain'])  #dB
                    except (AttributeError, VmbFeatureError):
                        print("Can't set Gain")

                    cam.set_pixel_format(PixelFormat.Mono8)
            except Exception as e:
                print("Unable to setup camera: {0}".format(e))
                vmb._shutdown()

        # streaming handler
        self.handler = Handler()

        self.frame = None
        self.pro_image = None

    def getNumberImages(self):
        '''Get number of loaded images'''
        return None

    def getFileName(self):
        '''Get current file in camera'''
        return None

    def getImage(self, get_raw=False):
        ''' Capture a single image from the Camera '''
        with VmbSystem.get_instance() as vmb:
            try:
                cam = vmb.get_camera_by_id(self.camParams['cameraName'])
                with cam:
                    try:
                        cam.start_streaming(handler=self.handler, buffer_count=1)
                        timestamp = time.time()
                        self.frame = self.handler.get_image()
                    except Exception as e:
                        print("Unable to capture camera: {0}".format(e))
                        timestamp = time.time()
                        self.frame = None
                    finally:
                        cam.stop_streaming()
            except Exception as e:
                timestamp = time.time()
                self.frame = None
                print("Unable to read camera: {0}".format(e))
                vmb._shutdown()

        timestamp_capture = time.time()

        if self.use_cuda:
            # Pre-allocation of GPU memory
            if self.pro_image is None:
                self.pro_image = cv2.cuda_GpuMat()

            # it's actually quicker to convert to grayscale on the CPU and upload smaller
            # image to GPU
            self.pro_image.upload(cv2.addWeighted(self.frame, 0.5, self.frame, 0.5, 0))
        else:
            # For some unknown reason, VmbPy returns the mono image in a wierd format and
            # the Apriltag detector fails. So need to recreate the image
            self.pro_image = cv2.addWeighted(self.frame, 0.5, self.frame, 0.5, 0)

        if not get_raw:
            self.pro_image = self.maybedoImageEnhancement(self.pro_image)
            self.pro_image = self.maybeDoFishEyeConversion(self.pro_image)
        timestamp_rectify = time.time()

        # Download the result back to the CPU
        if self.use_cuda:
            imageBW = self.pro_image.download()
        else:
            imageBW = self.pro_image

        return (imageBW, timestamp, timestamp_capture - timestamp, timestamp_rectify - timestamp_capture)

    def close(self):
        ''' close the camera'''
        super().close()
        print("Camera closed")
