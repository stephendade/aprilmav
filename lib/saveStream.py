'''
Simple thread to output OpenCV stream file
'''

import os
import queue
import threading
import cv2


class saveThread(threading.Thread):
    """
    Save images to a folder
    """
    def __init__(self, folder, exit_event):
        threading.Thread.__init__(self)
        self.save_queue = queue.Queue()
        self.exit_event = exit_event

        # create the capture folder if required
        try:
            os.makedirs(os.path.join(".", folder))
        except FileExistsError:
            pass

    def run(self):
        while True:
            if self.save_queue.empty():
                continue
            (image, filename, posn, rot, tags) = self.save_queue.get()
            # add in data (colour)
            imageColour = cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)
            cv2.putText(imageColour, "Pos (m) = {0:.3f}, {1:.3f}, {2:.3f}".format(posn[0], posn[1], posn[2]), (10, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
            cv2.putText(imageColour, "Rot (deg) = {0:.1f}, {1:.1f}, {2:.1f}".format(rot[0], rot[1], rot[2]), (10, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
            imageColour = self.labelTags(imageColour, tags)
            cv2.imwrite(filename, imageColour, [cv2.IMWRITE_PNG_COMPRESSION, 0])

            # print("Saved {0}".format(filename))
            if self.exit_event.wait(timeout=0.01):
                return

    def labelTags(self, image, tags):
        """
        Label the tags in the image
        """
        # loop over the AprilTag detection results
        for r in tags:
            # extract the bounding box (x, y)-coordinates for the AprilTag
            # and convert each of the (x, y)-coordinate pairs to integers
            (ptA, ptB, ptC, ptD) = r.corners
            ptB = (int(ptB[0]), int(ptB[1]))
            ptC = (int(ptC[0]), int(ptC[1]))
            ptD = (int(ptD[0]), int(ptD[1]))
            ptA = (int(ptA[0]), int(ptA[1]))
            # draw the bounding box of the AprilTag detection
            cv2.line(image, ptA, ptB, (0, 255, 0), 2)
            cv2.line(image, ptB, ptC, (0, 255, 0), 2)
            cv2.line(image, ptC, ptD, (0, 255, 0), 2)
            cv2.line(image, ptD, ptA, (0, 255, 0), 2)
            # draw the center (x, y)-coordinates of the AprilTag
            (cX, cY) = (int(r.center[0]), int(r.center[1]))
            cv2.circle(image, (cX, cY), 5, (0, 0, 255), -1)
            # draw the tag ID
            cv2.putText(image, str(
                r.tag_id), (ptA[0] + 10, ptA[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 1)
        return image
