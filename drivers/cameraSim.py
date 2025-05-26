'''
Camera Interfacing for a simulated camera
'''

import time
import numpy
import cv2
from .cameraBase import cameraBase


class camera(cameraBase):
    '''A Camera setup and capture class'''

    def __init__(self, camParams, tagSize, tagFamily, decimation, tagEngine, use_cuda=False, camName=""):
        '''Initialise the camera, based on a dict of settings'''
        super().__init__(camParams, tagSize, tagFamily, decimation, tagEngine, use_cuda, camName)

        self.rotation_per_frame = 20  # degrees
        self.current_rotation = 0

        # Define the rotation and translation scripts for the image.
        # The scripts loop every 80 frames, forming a basic square pattern.
        self.location_in_script = 0
        self.rotation_script = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0,                      #
                                0, 10, 20, 30, 40, 50, 60, 70, 80, 90,             # +rot90
                                90, 90, 90, 90, 90, 90, 90, 90, 90, 90,            #
                                90, 100, 110, 120, 130, 140, 150, 160, 170, 180,   # +rot90
                                180, 180, 180, 180, 180, 180, 180, 180, 180, 180,  #
                                180, 190, 200, 210, 220, 230, 240, 250, 260, 270,  # +rot90
                                270, 270, 270, 270, 270, 270, 270, 270, 270, 270,  #
                                270, 280, 290, 300, 310, 320, 330, 340, 350, 360]  # +rot90

        self.translation_script_x = [0, 10, 20, 30, 40, 50, 60, 70, 80, 90,        # +x
                                     90, 90, 90, 90, 90, 90, 90, 90, 90, 90,       #
                                     90, 90, 90, 90, 90, 90, 90, 90, 90, 90,       #
                                     90, 90, 90, 90, 90, 90, 90, 90, 90, 90,       #
                                     80, 70, 60, 50, 40, 30, 20, 10, 0, 0,         # -x
                                     0, 0, 0, 0, 0, 0, 0, 0, 0, 0,                 #
                                     0, 0, 0, 0, 0, 0, 0, 0, 0, 0,                 #
                                     0, 0, 0, 0, 0, 0, 0, 0, 0, 0]                 #

        self.translation_script_y = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0,                 #
                                     0, 0, 0, 0, 0, 0, 0, 0, 0, 0,                 #
                                     0, 10, 20, 30, 40, 50, 60, 70, 80, 90,        # +y
                                     90, 90, 90, 90, 90, 90, 90, 90, 90, 90,       #
                                     90, 90, 90, 90, 90, 90, 90, 90, 90, 90,       #
                                     90, 90, 90, 90, 90, 90, 90, 90, 90, 90,       #
                                     80, 70, 60, 50, 40, 30, 20, 10, 0, 0,         # -y
                                     0, 0, 0, 0, 0, 0, 0, 0, 0, 0]                 #

        # Define the AprilTag detector
        self.tag_id = [0, 1, 2, 3, 4]
        self.tag_size = 50
        self.tag_img = []
        self.tag_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)

        # pre-generate tag images
        for tag in self.tag_id:
            # img = numpy.zeros((self.tag_size, self.tag_size), dtype=numpy.uint8)
            img = cv2.aruco.generateImageMarker(self.tag_dict, tag, self.tag_size)
            self.tag_img.append(img)

        # Constants for Gaussian noise
        self.NOISE_MEAN = 75
        self.NOISE_STDDEV = 15

    def getNumberImages(self):
        '''Get number of loaded images

        Returns:
            None
        '''
        return None

    def getImage(self, get_raw=False):
        ''' Capture a single image from the Camera and time of capture (sec since epoch)'''

        curFrame = self.NOISE_MEAN * numpy.ones((self.camParams['resolution'][1],
                                                 self.camParams['resolution'][0]),
                                                dtype=numpy.uint8)

        # Place the AprilTags around the center of the blank image
        center_x = self.camParams['resolution'][0] // 2
        center_y = self.camParams['resolution'][1] // 2
        grid_size = int(numpy.ceil(numpy.sqrt(len(self.tag_img))))
        tag_spacing = self.tag_size + 50  # 50 pixels spacing between tags

        start_x = center_x - (grid_size * tag_spacing) // 2
        start_y = center_y - (grid_size * tag_spacing) // 2

        for i, tag in enumerate(self.tag_img):
            row = i // grid_size
            col = i % grid_size
            x_offset = start_x + col * tag_spacing
            y_offset = start_y + row * tag_spacing
            curFrame[y_offset:y_offset + self.tag_size, x_offset:x_offset + self.tag_size] = tag

        (h, w) = curFrame.shape[:2]

        # Translate the image by the current script values
        tx = 3 * self.translation_script_x[self.location_in_script]
        ty = 3 * self.translation_script_y[self.location_in_script]
        M_translate = numpy.float32([[1, 0, tx], [0, 1, ty]])
        curFrame = cv2.warpAffine(curFrame, M_translate, (w, h),
                                  borderMode=cv2.BORDER_CONSTANT, borderValue=self.NOISE_MEAN)

        # Rotate the image by self.current_rotation degrees
        M_rot = cv2.getRotationMatrix2D((center_x, center_y), self.rotation_script[self.location_in_script], 1.0)
        curFrame = cv2.warpAffine(curFrame, M_rot, (w, h),
                                  borderMode=cv2.BORDER_CONSTANT, borderValue=self.NOISE_MEAN)

        # Add Gaussian noise to the image
        noise = numpy.random.normal(self.NOISE_MEAN, self.NOISE_STDDEV, curFrame.shape).astype(numpy.uint8)
        self.imageBW = cv2.add(curFrame, noise)

        self.location_in_script = (self.location_in_script + 1) % 80

        # Image enhancement
        self.imageBW = self.maybedoImageEnhancement(self.imageBW)

        # Return a fake time of 50ms for stable velocity calculations
        self.image_timestamp = time.time() - 0.05
        self.time_capture = 0.05
        self.time_rectify = 0.05

    def getFileName(self):
        '''Get current file in camera. Returns None.'''
        return None

    def close(self):
        ''' close the camera'''
