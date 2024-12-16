'''
Simple thread to output OpenCV stream to Gstreamer

Use gst-launch-1.0 -v udpsrc uri=udp://0.0.0.0:5000 ! application/x-rtp,payload=96,encoding-name=H264 ! \
    rtph264depay ! decodebin ! videoconvert ! autovideosink
 to view stream
'''

import queue
import threading
import cv2


class videoThread(threading.Thread):
    """
    A thread that takes in OpenCV images and streams then over RTP.
    Detected Apriltags and other system data is overlaid
    """
    def __init__(self, port, exit_event):
        threading.Thread.__init__(self)
        self.frame_queue = queue.Queue()
        self.port = str(port)
        self.exit_event = exit_event

    def run(self):
        vidOut = None
        while True:
            if self.exit_event.wait(timeout=0.001):
                if vidOut:
                    vidOut.release()
                return
            if self.frame_queue.empty():
                continue
            (image, posn, rot, tags) = self.frame_queue.get()
            # process B&W image to colour and add text
            imageColour = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
            imageColour = self.labelTags(imageColour, tags)
            cv2.putText(imageColour, "Pos (m) = {0:.3f}, {1:.3f}, {2:.3f}".format(posn[0], posn[1], posn[2]), (10, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
            cv2.putText(imageColour, "Rot (deg) = {0:.1f}, {1:.1f}, {2:.1f}".format(rot[0], rot[1], rot[2]), (10, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
            # Start video server if not already started
            if not vidOut:
                vidOut = cv2.VideoWriter('appsrc ! video/x-raw, format=BGR ! videoconvert ! x264enc \
                                         speed-preset=ultrafast tune=zerolatency ! rtph264pay name=pay0 pt=96 ! \
                                         udpsink host=0.0.0.0 port=' +
                                         self.port + '', cv2.CAP_GSTREAMER, 0, 20, imageColour.shape[:2][::-1], True)
                if not vidOut.isOpened():
                    print(
                        "Error opening video stream. Ensure OpenCV is built with GStreamer support")
                    return
            # Send processed image to video stream
            vidOut.write(imageColour)

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
