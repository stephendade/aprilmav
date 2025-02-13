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
    def __init__(self, IPport, exit_event):
        threading.Thread.__init__(self)
        self.frame_queue = queue.Queue()
        self.ip = IPport.split(":")[0]
        self.port = IPport.split(":")[1]
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
            (img_by_cam, posn, rot, tags_by_cam) = self.frame_queue.get()
            imageColour = None
            for camName in sorted(img_by_cam.keys()):
                imageCam = cv2.cvtColor(img_by_cam[camName][0], cv2.COLOR_GRAY2BGR)
                if tags_by_cam:
                    img_by_cam[camName] = self.labelTags(imageCam, tags_by_cam[camName])
                # overlay camera name on the image
                cv2.putText(imageCam, camName, (10, 45),
                            cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 0, 0), 2)
                # put a border around the image
                imageCam = cv2.copyMakeBorder(
                    imageCam, 5, 5, 5, 5, cv2.BORDER_CONSTANT, value=(255, 255, 255))
                # append to final image
                if imageColour is None:
                    imageColour = imageCam
                else:
                    imageColour = cv2.hconcat([imageColour, imageCam])
            if posn:
                cv2.putText(imageColour, "Pos (m) = {0:.3f}, {1:.3f}, {2:.3f}".format(posn[0], posn[1], posn[2]), (10, 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
            if rot:
                cv2.putText(imageColour, "Rot (deg) = {0:.1f}, {1:.1f}, {2:.1f}".format(rot[0], rot[1], rot[2]), (10, 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
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
