'''
Simple thread to output OpenCV stream file
'''

import os
import queue
import threading
import cv2

from modules.common import labelTags


class saveThread(threading.Thread):
    """
    Save images to a folder
    """
    def __init__(self, folder, exit_event, CAMERAS=None):
        threading.Thread.__init__(self)
        self.save_queue = queue.Queue()
        self.exit_event = exit_event

        # create the capture folder if required
        try:
            os.makedirs(os.path.join(".", folder))
        except FileExistsError:
            pass

        # and camera folders if required
        if CAMERAS:
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
            img_width = imageColour.shape[1]
            scale = img_width / 3 / 500  # Assuming 50 is the base width for scaling

            cv2.putText(imageColour, "Pos (m) = {0:.3f}, {1:.3f}, {2:.3f}".format(posn[0], posn[1], posn[2]),
                        (10, int(30 * scale)), cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 255), int(2 * scale))
            cv2.putText(imageColour, "Rot (deg) = {0:.1f}, {1:.1f}, {2:.1f}".format(rot[0], rot[1], rot[2]),
                        (10, int(60 * scale)), cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 255), int(2 * scale))
            imageColour = labelTags(imageColour, tags)
            cv2.imwrite(filename, imageColour, [cv2.IMWRITE_PNG_COMPRESSION, 0])
            print("Saved {0}".format(filename))
