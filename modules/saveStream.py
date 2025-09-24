'''
Simple thread to output OpenCV stream file
'''

import os
import queue
import threading
import cv2

from modules.common import getFontSize, labelTags


class saveThread(threading.Thread):
    """
    Save images to a folder
    """
    def __init__(self, folder, exit_event, CAMERAS=None, compression=0, useJpg=False):
        threading.Thread.__init__(self)
        self.save_queue = queue.Queue()
        self.exit_event = exit_event

        self.text_height = None
        self.font_scale = 1
        self.thickness = 4

        self.compression = compression
        self.imagetype = cv2.IMWRITE_JPEG_QUALITY if useJpg else cv2.IMWRITE_PNG_COMPRESSION

        # create the capture folder if required
        try:
            os.makedirs(os.path.join(".", folder))
        except FileExistsError:
            pass

        # and camera folders if required
        if CAMERAS and len(CAMERAS) > 1:
            for CAMERA in CAMERAS:
                try:
                    os.makedirs(os.path.join(".", folder, CAMERA.camName))
                except FileExistsError:
                    pass

    def run(self):
        while True:
            if self.exit_event.wait(timeout=0.001) and self.save_queue.empty():
                return
            if self.save_queue.empty():
                continue
            (image, filename, posn, rot, tags) = self.save_queue.get()

            # add in data (colour)
            imageColour = cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)

            if not self.text_height:
                self.font_scale, self.thickness, self.text_height = getFontSize(imageColour)

            cv2.putText(imageColour, "Pos (m) = {0:.3f}, {1:.3f}, {2:.3f}".format(posn[0], posn[1], posn[2]),
                        (10, self.text_height + 10), cv2.FONT_HERSHEY_SIMPLEX,
                        self.font_scale, (0, 0, 255), self.thickness, cv2.LINE_AA)
            cv2.putText(imageColour, "Rot (deg) = {0:.1f}, {1:.1f}, {2:.1f}".format(rot[0], rot[1], rot[2]),
                        (10, 2*(self.text_height + 10)), cv2.FONT_HERSHEY_SIMPLEX,
                        self.font_scale, (0, 0, 255), self.thickness, cv2.LINE_AA)
            imageColour = labelTags(imageColour, tags, self.thickness, self.font_scale)
            cv2.imwrite(filename, imageColour, [self.imagetype, self.compression])
            print("Saved {0}".format(filename))
