'''
Geometrical functions
'''
from collections import deque
import math
import numpy

from transforms3d.euler import mat2euler, euler2mat

from modules.common import getPos, getRotation, getTransform

numpy.set_printoptions(precision=3, suppress=True)


def isclose(x, y, rtol=1.e-5, atol=1.e-8):
    '''Return true if two floating point numbers are close enough'''
    return abs(x-y) <= atol + rtol * abs(y)


def mag(x):
    '''Return magnitude of vector'''
    return math.sqrt(sum(i**2 for i in x))


class tagDB:
    '''Database of all detected tags'''

    def __init__(self, debug=False, slidingWindow=5, extraOpt=False):
        self.T_VehToWorld = deque(maxlen=slidingWindow+1)
        self.timestamps = deque(maxlen=slidingWindow+1)
        self.T_VehToWorldFiltered = deque(maxlen=slidingWindow+1)
        self.timestampsFiltered = deque(maxlen=slidingWindow+1)
        self.deltaVelocityFiltered = deque(maxlen=slidingWindow+1)
        self.tagPlacement = {}
        self.tagnewT = {}
        self.tagDuplicatesT = {}
        self.debug = debug
        self.reportedPos = numpy.array((0, 0, 0))
        self.reportedPosPrev = numpy.array((0, 0, 0))
        self.reportedRot = numpy.array((0, 0, 0))
        self.reportedVelocity = numpy.array((0, 0, 0))
        self.reportedTimestampPrev = 0
        self.T_VehToWorld.append(numpy.array(numpy.eye((4))))
        self.T_VehToWorld.append(numpy.array(numpy.eye((4))))
        self.T_VehToWorldFiltered.append(numpy.array(numpy.eye((4))))
        self.T_VehToWorldFiltered.append(numpy.array(numpy.eye((4))))
        self.slidingWindow = slidingWindow
        self.extraOpt = extraOpt
        # Z-score threshold for outlier detection
        self.threshold = 2
        # Tag candidates. If they are present for 5 frames, add them to the database
        self.tagCandidates = deque(maxlen=5)

    def printTags(self):
        '''Print all tags in the database'''
        for tagid, tag in self.tagPlacement.items():
            print("Tag ID {0} at pos {1}, rot {2}".format(tagid, getPos(tag).round(3),
                                                          getRotation(tag).round(1)))

    def newFrame(self):
        '''Reset the duplicates for a new frame of tags'''
        self.tagDuplicatesT = {}
        self.tagnewT = {}

    def addTag(self, tag, T_CamtoVeh):
        '''Add tag to database in vehicle reference frame'''
        if tag.tag_id not in self.tagPlacement:
            # tag is in vehicle frame
            T_TagToCam = getTransform(tag)
            # T_TagToCam[0:3, 3] = T_TagToCam[0:3, 3]
            # T(Veh <- tag) = T(Veh <- Cam_t) * T(Cam_t <- tag)
            self.tagnewT[tag.tag_id] = T_CamtoVeh @ T_TagToCam
            if self.debug:
                print("New Tag ID {0} at pos {1}, rot {2}".format(tag.tag_id, getPos(self.tagnewT[tag.tag_id]).round(3),
                                                                  getRotation(self.tagnewT[tag.tag_id]).round(1)))
        else:
            # get tag's last pos, in camera frame
            T_TagToCam = getTransform(tag)
            # T_TagToCam[0:3, 3] = T_TagToCam[0:3, 3]

            # save tag positions in Vehicle frame at time t for the duplicate
            # T(Veh <- tag) = T(Veh <- Cam_t) * T(Cam_t <- tag)
            self.tagDuplicatesT[tag.tag_id] = T_CamtoVeh @ T_TagToCam
            if self.debug:
                print("Dupl Tag ID {0} pos {1}, rot {2}".format(tag.tag_id,
                                                                getPos(
                                                                    self.tagDuplicatesT[tag.tag_id]).round(3),
                                                                getRotation(self.tagDuplicatesT[tag.tag_id]).round(1)))

    def generateReportedLoc(self, timestamp):
        '''Generate the vehicle's current position
         and velocity and rotation in ArduPilot NED format,
         noting that Apriltag coords are (image right, image down, image out) relative to camera.
         Timestamp is seconds since epoch'''
        self.T_VehToWorldFiltered.append(self.T_VehToWorld[-1])
        self.timestampsFiltered.append(self.timestamps[-1])

        # only start filtering when we have enough data
        if len(self.T_VehToWorldFiltered) > self.slidingWindow and self.slidingWindow > 2:
            # Generate position, rotation in ArduPilot format, zscore to remove outliers
            timeSeriesPos = []
            timeSeriesRot = []
            for i in range(1, self.slidingWindow+1):
                if len(self.T_VehToWorldFiltered) >= i:
                    timeSeriesPos.append(getPos(self.T_VehToWorldFiltered[-i]))
                    timeSeriesRot.append(getRotation(self.T_VehToWorldFiltered[-i], True))
            self.reportedPos = self.zscoreFilter(timeSeriesPos)
            self.reportedRot = self.zscoreFilter(timeSeriesRot)
        else:
            self.reportedPos = getPos(self.T_VehToWorldFiltered[-1])
            self.reportedRot = getRotation(self.T_VehToWorldFiltered[-1], True)

        # store the delta velocity and z-score filter it
        delta = numpy.array(self.reportedPos) - numpy.array(self.reportedPosPrev)
        self.deltaVelocityFiltered.append(delta / (timestamp - self.reportedTimestampPrev))
        if len(self.deltaVelocityFiltered) > self.slidingWindow and self.slidingWindow > 2:
            self.reportedVelocity = self.zscoreFilter(self.deltaVelocityFiltered)
        else:  # not enough data yet
            self.reportedVelocity = self.deltaVelocityFiltered[-1]

        # SMA average the velocity
        if len(self.deltaVelocityFiltered) > self.slidingWindow and self.slidingWindow > 2:
            self.reportedVelocity = numpy.mean(self.deltaVelocityFiltered, axis=0)

        self.reportedPosPrev = self.reportedPos
        self.reportedTimestampPrev = timestamp

    def zscoreFilter(self, timeseries):
        """
        Filters out outliers from a time series using Z-scores.

        Parameters:
        timeseries (list or numpy.ndarray): A list or array of positions where each position is a list or array of
        coordinates.

        Returns:
        tuple: The the last position if it's not an outlier, else the 2nd last position.
        """
        timeseries = numpy.array(timeseries)

        # Calculate mean and standard deviation for each coordinate
        mean_pos = numpy.mean(timeseries, axis=0)
        std_pos = numpy.std(timeseries, axis=0)

        # return the last position if it's not an outlier, else return the 2nd last position
        if all(numpy.abs((timeseries[-1] - mean_pos) / std_pos) < self.threshold):
            return timeseries[-1]
        return timeseries[-2]

    def getTagdb(self):
        '''get coords of all tags by axis'''
        # xcoord = []
        # for tagid, tag in self.tagPlacement.items():
        #    xcoord.append(tag[0, axis])
        return self.tagPlacement

    def getTagPoints(self, axis):
        '''Get all the tags positions in a particular axis'''
        xcoord = []
        for _, tag in self.tagPlacement.items():
            xcoord.append(tag[axis, 3])
        return xcoord

    def getBestTransform(self, timestamp):
        '''Given the current self.tagDuplicatesT, what is the best fitting transform
        back to self.tagPlacement. timestamp is the time that the tags were captured'''
        # get the least cost transform from the common points at time t-1 to time t
        # cost is the sum of position error between the common points, with the t-1 points
        # projected forward to t
        if len(self.tagDuplicatesT) == 0 and len(self.tagPlacement) != 0:
            print("WARNING: No tags in view")
        elif len(self.tagDuplicatesT) == 1:
            print("WARNING: Only 1 duplicate tag in view")

        if len(self.tagDuplicatesT) > 0:
            bestTransform = numpy.array(numpy.eye((4)))
            lowestCost = 999
            # Use each tag pair as a guess for the correct transform - lowest cost wins
            # Where "cost" is the least 3d distance between *all* tag pairs
            for tagid, tagT in self.tagDuplicatesT.items():
                if self.debug:
                    print("Trying tag {0}".format(tagid))
                # t is the time now, t-1 is the previous frame - where T_VehToWorld is at this point
                # tag duplicate (tagDB) is the world frame
                # new tag is in the vehicle frame
                # World frame is time-independent
                # TTagDB(World<-Tag) = TVeh(World <- veh_t) * TNewTag(veh_t <-Tag)
                # and:
                # TVeh(World <- veh_t-1) = TVeh(World <- veh_t) * TVeh(veh_t <- veh_t-1)
                #
                # Thus
                # TVeh(World <- veh_t) = TVeh(World <- veh_t-1) * TVeh(veh_t <- veh_t-1)^-1
                # then
                # TTagDB(World<-Tag) = TVeh(World <- veh_t-1) * TVeh(veh_t <- veh_t-1)^-1 * TNewTag(veh_t<-Tag)
                #
                # Thus
                # TVeh(veh_t <- veh_t-1) = TNewTag(veh_t<-Tag) * TTagDB(World<-Tag)^-1 * TVeh(World <- veh_t-1)
                Ttprevtocur = tagT @ numpy.linalg.inv(
                    self.tagPlacement[tagid]) @ self.T_VehToWorld[-1]
                # print("tagT =\n{0}".format(tagT))
                # print("self.tagPlacement[tagid]^-1 =\n{0}".format(numpy.linalg.inv(self.tagPlacement[tagid])))
                # print("self.T_VehToWorld(old) =\n{0}".format(self.T_VehToWorld))
                # and figure out summed distances between transformed new point to old (in world frame)
                summeddist = 0
                for tagidj, tagTj in self.tagDuplicatesT.items():
                    PosDelta = self.tagPlacement[tagidj] @ [[0], [0], [0], [
                        1]] - self.T_VehToWorld[-1] @ numpy.linalg.inv(Ttprevtocur) @ tagTj @ [[0], [0], [0], [1]]
                    summeddist += mag(PosDelta)

                if lowestCost > summeddist:
                    if self.debug:
                        print("Using tag {0} with per-tag average error {1:.3f}m".format(
                            tagid, summeddist / len(self.tagDuplicatesT)))
                    lowestCost = summeddist
                    bestTransform = Ttprevtocur
                else:
                    if self.debug:
                        print("Ignoring tag {0} with error {1:.3f}m".format(
                            tagid, summeddist / len(self.tagDuplicatesT)))

            # now iterate the bestTransform a little to see if we can get a better fit
            if self.extraOpt:
                step_size = 0.01
                for _ in range(100):
                    # Generate small changes to the rotation part
                    rotation_matrix = Ttprevtocur[:3, :3]
                    euler_angles = mat2euler(rotation_matrix)
                    delta_euler = numpy.random.randn(3) * step_size
                    new_euler_angles = euler_angles + delta_euler
                    new_rotation_matrix = euler2mat(*new_euler_angles)

                    # Generate small changes to the translation part
                    translation_vector = Ttprevtocur[:3, 3]
                    delta_translation = numpy.random.randn(3) * step_size
                    new_translation_vector = translation_vector + delta_translation

                    # Construct the new transformation matrix
                    new_transform = numpy.eye(4)
                    new_transform[:3, :3] = new_rotation_matrix
                    new_transform[:3, 3] = new_translation_vector

                    # Calculate the cost for the new transform
                    summeddist = 0
                    for tagidj, tagTj in self.tagDuplicatesT.items():
                        PosDelta = self.tagPlacement[tagidj] @ [[0], [0], [0], [
                            1]] - self.T_VehToWorld[-1] @ numpy.linalg.inv(new_transform) @ tagTj @ [[0], [0], [0], [1]]
                        summeddist += mag(PosDelta)

                    # Update the best transform if the new cost is lower
                    if lowestCost > summeddist:
                        if self.debug:
                            print("Optimising with error {0:.3f}m".format(summeddist))
                        lowestCost = summeddist
                        bestTransform = new_transform
            if lowestCost > 0.5 * len(self.tagDuplicatesT):
                print("WARNING: bad position estimate. Ignoring this frame.")
            else:
                # We have our least-cost transform
                # TVeh(World <- veh_t) = TVeh(World <- veh_t-1) * TVeh(veh_t <- veh_t-1)^-1
                self.T_VehToWorld.append(self.T_VehToWorld[-1] @ numpy.linalg.inv(
                    bestTransform))
                self.timestamps.append(timestamp)
                if self.debug:
                    print("Delta {0}, Rot = {1}".format(getPos(bestTransform).round(3),
                                                        getRotation(bestTransform).round(1)))

                # Update position, rotation, velocity
                self.generateReportedLoc(timestamp)
            if self.debug:
                print("New Pos {0}, Rot = {2} with {1} tags".format(self.reportedPos.round(3),
                                                                    len(self.tagDuplicatesT),
                                                                    self.reportedRot.round(1)))
        # finally add any new tags (veh frame) to database (in world reference frame)
        thisFrameCandidates = {}
        for tagid, tagT in self.tagnewT.items():
            # TTagDB(World<-Tag) = TVeh(World <- veh_t) * TNewTag(veh_t <-Tag)
            tagInWorld = self.T_VehToWorld[-1] @ tagT
            thisFrameCandidates[tagid] = [getPos(tagInWorld), getRotation(tagInWorld, True)]
        self.tagCandidates.append(thisFrameCandidates)

        # if a tag was present in every frame, add it to the database
        if len(self.tagCandidates) == 5:
            for tagid in self.tagCandidates[0]:
                if all(tagid in list(frame.keys()) for frame in self.tagCandidates):
                    median_pos = numpy.median([frame[tagid][0] for frame in self.tagCandidates], axis=0)
                    median_rot = numpy.median([frame[tagid][1] for frame in self.tagCandidates], axis=0)
                    T_tag = numpy.eye(4)
                    T_tag[:3, :3] = euler2mat(*median_rot, 'sxyz')
                    T_tag[:3, 3] = median_pos
                    self.tagPlacement[tagid] = T_tag
                    if self.debug:
                        print("Added Tag ID {0} at pos {1}, rot {2}".format(tagid,
                                                                            getPos(self.tagPlacement[tagid]).round(3),
                                                                            getRotation(self.tagPlacement[tagid]).round(1)))
