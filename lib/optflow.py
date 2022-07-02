'''
Optical flow functions
'''

import math
import numpy
import cv2
import time

class OptFlow:
    '''Using optical flow'''

    def __init__(self, debug=True, doGUI=False):
        self.debug = debug
        # params for ShiTomasi corner detection
        self.feature_params = dict( maxCorners = 100,
                                    qualityLevel = 0.3,
                                    minDistance = 7,
                                    blockSize = 7 )

        # Parameters for lucas kanade optical flow
        self.lk_params = dict( winSize  = (15, 15),
                               maxLevel = 2,
                               criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))
                               
        self.p0 = numpy.array([])
        self.prevFrame = numpy.array([])
        
        self.flowspeed = (0,0)
        self.flowstd = (0,0)
        self.flowqual = 0
        
        self.doGUI = doGUI
        if self.doGUI:
            # Create some random colors
            self.color = numpy.random.randint(0, 255, (100, 3))
            self.mask = None

    def doInitFrame(self, frame):
        # Initial frame
        self.flowspeed = [-1, -1]
        self.flowstd = [-1, -1]
        p0 = cv2.goodFeaturesToTrack(frame, mask = None, **self.feature_params)
        if p0 is not None:
            self.p0 = p0
        else:
            self.p0 = numpy.array([])
        self.prevFrame = frame.copy()

    def newFrame(self, frame, timeCapture):
        # Process opencv frame
        
        #if this is the first frame, just get points
        if len(self.prevFrame) == 0:
            self.doInitFrame(frame)
        
        # No good points from prev frame
        if len(self.p0) == 0:
            self.doInitFrame(frame)
            
        # Calculate optical flow
        p1, st, err = cv2.calcOpticalFlowPyrLK(self.prevFrame, frame, self.p0, None, **self.lk_params)

        if self.doGUI:
            self.mask = numpy.zeros_like(frame)
            
        # Select good points and get deltas relative to prev frame
        if p1 is not None and self.p0 is not None:
            deltas = []
            good_new = p1[st==1]
            good_old = self.p0[st==1]
            for i, (new, old) in enumerate(zip(good_new, good_old)):
                a, b = new.ravel()
                c, d = old.ravel()
                deltas.append((int(a)-int(c), int(b)-int(d)))
                if self.doGUI:
                    self.mask = cv2.line(self.mask, (int(a), int(b)), (int(c), int(d)), self.color[i].tolist(), 2)
                    frame = cv2.circle(frame, (int(a), int(b)), 5, self.color[i].tolist(), -1)
                    
            if len(deltas) > 0:
                self.flowspeed = numpy.around(numpy.mean(deltas, axis=0))
                self.flowstd = numpy.around(numpy.std(deltas, axis=0), 1)
                #print(deltas)
                #print(self.flowstd)
                print("Optical flow, {0:.1f},{1:.1f},{2:.1f},{3:.1f}".format(self.flowspeed[0], self.flowspeed[1], self.flowstd[0], self.flowstd[1]))
            else:
                self.flowspeed = [-1, -1]
                self.flowstd = [-1, -1]
                
            if self.doGUI:
                img = cv2.add(frame, self.mask)
                cv2.imshow('frame', img)
                k = cv2.waitKey(1) & 0xff
                time.sleep(0.2)
                    
        # Now update the previous frame and previous points
        p0 = cv2.goodFeaturesToTrack(frame, mask = None, **self.feature_params)
        if p0 is not None:
            self.p0 = p0
        else:
            self.p0 = numpy.array([])
        self.prevFrame = frame.copy()

