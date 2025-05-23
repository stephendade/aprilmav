'''
Simple thread to output OpenCV stream to Gstreamer

Use gst-launch-1.0 -v udpsrc uri=udp://0.0.0.0:5000 ! application/x-rtp,payload=96,encoding-name=H264 ! \
    rtph264depay ! decodebin ! videoconvert ! autovideosink
 to view stream
'''

import queue
import threading
import cv2

from modules.common import getFontSize, labelTags


class videoThread(threading.Thread):
    """
    A thread that takes in OpenCV images and streams then over RTP.
    Detected Apriltags and other system data is overlaid
    """
    def __init__(self, IPport, exit_event):
        threading.Thread.__init__(self)
        self.frame_queue = queue.Queue()
        self.ip = IPport.split(":")[0]
        self.port = IPport.split(":")[1]
        self.exit_event = exit_event

        self.text_height = None
        self.font_scale = 1
        self.thickness = 4

    def run(self):
        vidOut = None
        while True:
            if self.exit_event.wait(timeout=0.001):
                if vidOut:
                    vidOut.release()
                return
            if self.frame_queue.empty():
                continue
            (img_tags_by_cam, posn, rot) = self.frame_queue.get()
            imageColour = None
            for camName in sorted(img_tags_by_cam.keys()):
                imageCam = cv2.cvtColor(img_tags_by_cam[camName][0], cv2.COLOR_GRAY2BGR)
                if not self.text_height:
                    self.font_scale, self.thickness, self.text_height = getFontSize(imageColour)

                if img_tags_by_cam[camName][3]:
                    imageCam = labelTags(imageCam, img_tags_by_cam[camName][3], self.thickness, self.font_scale)
                # overlay camera name on the image
                cv2.putText(imageCam, camName, (10, self.text_height + 10),
                            cv2.FONT_HERSHEY_SIMPLEX, self.font_scale, (255, 0, 0), self.thickness, cv2.LINE_AA)
                # put a border around the image
                imageCam = cv2.copyMakeBorder(
                    imageCam, 5, 5, 5, 5, cv2.BORDER_CONSTANT, value=(255, 255, 255))
                # append to final image
                if imageColour is None:
                    imageColour = imageCam
                else:
                    imageColour = cv2.hconcat([imageColour, imageCam])
            if posn:
                cv2.putText(imageColour, "Pos (m) = {0:.3f}, {1:.3f}, {2:.3f}".format(posn[0], posn[1], posn[2]),
                            (10, self.text_height + 10), cv2.FONT_HERSHEY_SIMPLEX,
                            self.font_scale, (0, 0, 255), self.thickness, cv2.LINE_AA)
            if rot:
                cv2.putText(imageColour, "Rot (deg) = {0:.1f}, {1:.1f}, {2:.1f}".format(rot[0], rot[1], rot[2]),
                            (10, 2*(self.text_height + 10)), cv2.FONT_HERSHEY_SIMPLEX,
                            self.font_scale, (0, 0, 255), self.thickness, cv2.LINE_AA)
            # resize to 1080p, while keeping ratio
            height, width = imageColour.shape[:2]
            if height > 1080 or width > 1920:
                scaling_factor = min(1920 / width, 1080 / height)
                new_size = (int(width * scaling_factor), int(height * scaling_factor))
                imageColour = cv2.resize(imageColour, new_size, interpolation=cv2.INTER_AREA)
            # Start video server if not already started
            if not vidOut:
                vidOut = cv2.VideoWriter('appsrc ! video/x-raw, format=BGR ! videoconvert ! x264enc \
                                         speed-preset=faster tune=zerolatency ! rtph264pay config-interval=1 name=pay0 pt=96 ! \
                                         udpsink host=' + self.ip + ' port=' +
                                         self.port + '', cv2.CAP_GSTREAMER, 0, 20, imageColour.shape[:2][::-1], True)
                if not vidOut.isOpened():
                    print(
                        "Error opening video stream. Ensure OpenCV is built with GStreamer support")
                    return
            # Send processed image to video stream
            vidOut.write(imageColour)
