'''
Camera Interfacing for an Allied Vision Camera via Vimba X SDK
Requires the python package to be installed:
pip3 install https://github.com/alliedvision/VmbPy/releases/download/1.1.0/vmbpy-1.1.0-py3-none-linux_aarch64.whl
'''

import time
from queue import Queue
import queue
import threading
import copy
import cv2
from vmbpy import VmbSystem, Camera, Stream, Frame, FrameStatus, PixelFormat, VmbFeatureError

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
            timestamp = time.time()

            # Convert frame if it is not already the correct format
            if frame.get_pixel_format() == PixelFormat.Mono8:
                display = frame
            else:
                # This creates a copy of the frame. The original `frame` object can be requeued
                # safely while `display` is used
                display = frame.convert_pixel_format(PixelFormat.Mono8)

            self.display_queue.put((timestamp, display.as_opencv_image()), True)

        cam.queue_frame(frame)


class FrameProducer(threading.Thread):
    def __init__(self, camParams, use_cuda, camName, frame_queue):
        threading.Thread.__init__(self)
        self.camParams = camParams
        self.use_cuda = use_cuda
        self.camName = camName
        self.frame_queue = frame_queue
        self._stop_event = threading.Event()
        self.cam = None
        self.handler = None
        self.frame = None

    def run(self):
        with VmbSystem.get_instance() as vmb:
            try:
                self.cam = vmb.get_camera_by_id(self.camParams['cameraName'])
                with self.cam:
                    # apply camera settings
                    try:
                        self.cam.ExposureAuto.set('Off')
                    except (AttributeError, VmbFeatureError):
                        print("Can't set ExposureAuto")
                    try:
                        self.cam.ExposureMode.set('Timed')
                    except (AttributeError, VmbFeatureError):
                        print("Can't set ExposureMode")
                    try:
                        self.cam.ExposureTime.set(self.camParams['exposure'])  # usec
                    except (AttributeError, VmbFeatureError):
                        print("Can't set exposure")
                    try:
                        self.cam.GainAuto.set("Off")  # usec
                    except (AttributeError, VmbFeatureError):
                        print("Can't set GainAuto")
                    try:
                        self.cam.Gain.set(self.camParams['gain'])  # dB
                    except (AttributeError, VmbFeatureError):
                        print("Can't set Gain")
                    try:
                        self.cam.Gamma.set(self.camParams['Gamma'])
                    except (AttributeError, VmbFeatureError):
                        print("Can't set Gamma")

                    self.cam.set_pixel_format(PixelFormat.Mono8)

                    # streaming handler
                    self.handler = Handler()
                    try:
                        self.cam.start_streaming(handler=self.handler, buffer_count=1)

                        while not self._stop_event.is_set():
                            timestamp, self.frame = self.handler.get_image()
                            if self.frame_queue.full():
                                self.frame_queue.get()
                            self.frame_queue.put((timestamp, copy.deepcopy(self.frame)))
                    except Exception as e:
                        print("Camera stream error: {0}".format(e))
                    finally:
                        self.cam.stop_streaming()
            except Exception as e:
                print("Camera error: {0}".format(e))
            finally:
                vmb._shutdown()

    def stop(self):
        self._stop_event.set()


class camera(cameraBase):
    '''A Camera setup and capture class for a Allied Vision Camera'''
    def __init__(self, camParams, tagSize, tagFamily, decimation, tagEngine, use_cuda=False, camName=""):
        '''Initialise the camera, based on a dict of settings'''
        super().__init__(camParams, tagSize, tagFamily, decimation, tagEngine, use_cuda, camName)
        self.camParams = camParams
        self.use_cuda = use_cuda
        self.camName = camName
        self.frame_queue = Queue(1)
        self.camThread = FrameProducer(camParams, use_cuda, camName, self.frame_queue)
        self.camThread.start()

        self.frame = None
        self.pro_image = None
        time.sleep(3)

    def getNumberImages(self):
        '''Get number of loaded images'''
        return None

    def getFileName(self):
        '''Get current file in camera'''
        return None

    def getImage(self, get_raw=False):
        ''' Capture a single image from the Camera stream'''

        # grab from the queue
        try:
            timestamp, self.frame = self.frame_queue.get()
        except queue.Empty:
            print("Camera queue empty")
            return (None, None, None, None)
        # if the frame is None, then we have a problem
        if self.frame is None:
            print("Camera frame is None")
            return (None, None, None, None)

        timestamp_capture = time.time()

        if self.use_cuda:
            # Pre-allocation of GPU memory
            if self.pro_image is None:
                self.pro_image = cv2.cuda_GpuMat()

            self.pro_image.upload(self.frame)
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
        self.camThread.stop()
        self.camThread.join()
        super().close()
        print("Camera closed")
