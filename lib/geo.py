'''
Geometrical functions
'''

import math
from collections import deque
import numpy

from transforms3d.euler import mat2euler, euler2mat


def getTransform(tag):
    '''tag pose to transformation matrix'''
    T_Tag = numpy.array(numpy.eye((4)))
    T_Tag[0:3, 0:3] = numpy.array(tag.pose_R)
    pose_t = numpy.array(tag.pose_t)
    T_Tag[0:3, 3] = pose_t.reshape(3)

    # flip x axis
    # T_Tag = [[-1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]] @ T_Tag
    #
    # now convert to NED coord frame
    # so x-90, y+90, z in order
    # https://www.andre-gaschler.com/rotationconverter/
    # [[0,0,1,0],[-1,0,0,0],[0,-1,0,0],[0,0,0,1]]
    # T_Tag = [[1,0,0,0],[0,0,1,0],[0,-1,0,0],[0,0,0,1]] @ T_Tag
    return T_Tag


def getPos(T_tag):
    '''output the transformation matrix position as xyz tuple'''
    return numpy.array([T_tag[0, 3], T_tag[1, 3], T_tag[2, 3]])


def getRotation(T_Tag, useRadians=False):
    '''Get the vehicle's current rotation in Euler ZYX degrees'''
    if useRadians:
        return numpy.array(mat2euler(T_Tag[0:3][0:3], 'sxyz'))
    return numpy.array(numpy.rad2deg(mat2euler(T_Tag[0:3][0:3], 'sxyz')))


def isclose(x, y, rtol=1.e-5, atol=1.e-8):
    '''Return true if two floating point numbers are close enough'''
    return abs(x-y) <= atol + rtol * abs(y)


def mag(x):
    '''Return magnitude of vector'''
    return math.sqrt(sum(i**2 for i in x))


class tagDB:
    '''Database of all detected tags'''

    def __init__(self, debug=False, maxjump=0.1, slidingWindow=5, campos=(0, 0, 0), camrot=(0, 0, 0)):
        self.T_CamToWorld = deque(maxlen=slidingWindow+1)
        self.timestamps = deque(maxlen=slidingWindow+1)
        self.T_CamToWorldFiltered = deque(maxlen=slidingWindow+1)
        self.timestampsFiltered = deque(maxlen=slidingWindow+1)
        self.deltaVelocityFiltered = deque(maxlen=slidingWindow+1)
        self.tagPlacement = {}
        self.tagnewT = {}
        self.tagDuplicatesT = {}
        self.T_CamtoVeh = numpy.array(numpy.eye((4)))
        self.debug = debug
        self.maxjump = maxjump
        self.reportedPos = (0, 0, 0)
        self.reportedPosPrev = (0.0, 0.0, 0.0)
        self.reportedRot = (0, 0, 0)
        self.reportedVelocity = (0, 0, 0)
        self.reportedTimestampPrev = 0
        self.T_CamToWorld.append(numpy.array(numpy.eye((4))))
        self.T_CamToWorld.append(numpy.array(numpy.eye((4))))
        self.T_CamToWorldFiltered.append(numpy.array(numpy.eye((4))))
        self.T_CamToWorldFiltered.append(numpy.array(numpy.eye((4))))
        self.slidingWindow = slidingWindow
        # Define a threshold for Z-scores to identify outliers
        self.threshold = 2

        '''Generate the transformation matrix from camera to vehicle frame'''
        # Convert rotation tuple (Euler angles) to rotation matrix
        rotation_matrix = euler2mat(numpy.deg2rad(camrot[0]), numpy.deg2rad(camrot[1]),
                                    numpy.deg2rad(camrot[2]), axes='sxyz')

        # Construct the transformation matrix
        T_CamtoVeh = numpy.eye(4)
        T_CamtoVeh[0:3, 0:3] = rotation_matrix
        T_CamtoVeh[0:3, 3] = campos

        self.T_CamtoVeh = T_CamtoVeh

    def newFrame(self):
        '''Reset the duplicates for a new frame of tags'''
        self.tagDuplicatesT = {}
        self.tagnewT = {}

    def addTag(self, tag):
        '''Add tag to database'''
        if tag.tag_id not in self.tagPlacement:
            # tag is in cur camera frame
            T_TagToCam = getTransform(tag)
            # T_TagToCam[0:3, 3] = T_TagToCam[0:3, 3]
            self.tagnewT[tag.tag_id] = T_TagToCam
            if self.debug:
                print("New Tag ID {0} at pos {1}, rot {2}".format(tag.tag_id, getPos(self.tagnewT[tag.tag_id]).round(3),
                                                                  getRotation(self.tagnewT[tag.tag_id]).round(1)))
        else:
            # get tag's last pos, in camera frame
            T_TagToCam = getTransform(tag)
            # T_TagToCam[0:3, 3] = T_TagToCam[0:3, 3]

            # save tag positions in Camera frame at time t for the duplicate
            self.tagDuplicatesT[tag.tag_id] = T_TagToCam
            if self.debug:
                print("Dupl Tag ID {0} pos {1}, rot {2}".format(tag.tag_id,
                                                                getPos(
                                                                    self.tagDuplicatesT[tag.tag_id]).round(3),
                                                                getRotation(self.tagDuplicatesT[tag.tag_id]).round(1)))

    def is_large_jump(self, previous_transform, current_transform):
        """
        Check if the difference between two transformation matrices is larger than the threshold.

        Parameters:
        - previous_transform: np.array, previous 4x4 transformation matrix
        - current_transform: np.array, current 4x4 transformation matrix
        - threshold: float, threshold for detecting a large jump (e.g., 0.1)

        Returns:
        - bool: True if the difference is considered a large jump, False otherwise
        """
        # Compute the Frobenius norm of the difference
        diff = numpy.linalg.norm(current_transform - previous_transform, ord='fro')

        # Check if the difference exceeds the threshold
        return diff > self.maxjump

    def generateReportedLoc(self, timestamp):
        '''Generate the vehicle's current position
         and velocity and rotation in ArduPilot NED format,
         noting that Apriltag is coords are right, fwd, up (ENU).
         This assumes camera is on top of vehicle, bottom of camera facing fwd
         Timestamp is seconds since epoch'''
        # Check for any large jumps in position between
        # t and t-1 and t-2 and ignore accordingly
        if (self.is_large_jump(self.T_CamToWorld[-1], self.T_CamToWorld[-2]) and
           self.is_large_jump(self.T_CamToWorld[-2], self.T_CamToWorld[-3]) and
           len(self.timestampsFiltered) > 2):
            if self.debug:
                print("Ignoring jump1")
            self.T_CamToWorldFiltered.append(self.T_CamToWorld[-3])
            self.timestampsFiltered.append(self.timestamps[-3])
        elif self.is_large_jump(self.T_CamToWorld[-1], self.T_CamToWorld[-2]) and len(self.timestampsFiltered) > 2:
            if self.debug:
                print("Ignoring jump2")
            self.T_CamToWorldFiltered.append(self.T_CamToWorld[-2])
            self.timestampsFiltered.append(self.timestamps[-2])
        else:
            self.T_CamToWorldFiltered.append(self.T_CamToWorld[-1])
            self.timestampsFiltered.append(self.timestamps[-1])

        # Generate position, rotation in ArduPilot format, average 5 times (SMA)
        averagedpos = []
        averagedrot = []
        for i in range(1, self.slidingWindow+1):
            if len(self.T_CamToWorldFiltered) >= i:
                T_VehToWorld = self.T_CamtoVeh @ (self.T_CamToWorldFiltered[-i])
                averagedpos.append(getPos(T_VehToWorld))
                averagedrot.append(getRotation(T_VehToWorld, True))
        # only start filtering when we have enough data
        if len(self.T_CamToWorldFiltered) > self.slidingWindow:
            self.reportedPos = self.zscoreFilter(averagedpos)
            self.reportedRot = self.zscoreFilter(averagedrot)
        else:
            self.reportedPos = getPos(T_VehToWorld)
            self.reportedRot = getRotation(T_VehToWorld, True)

        # store the delta velocity
        delta = numpy.array(self.reportedPos) - numpy.array(self.reportedPosPrev)
        self.deltaVelocityFiltered.append(delta / (timestamp - self.reportedTimestampPrev))
        if len(self.deltaVelocityFiltered) > self.slidingWindow:
            self.reportedVelocity = self.zscoreFilter(self.deltaVelocityFiltered)
        else:  # not enough data yet
            self.reportedVelocity = self.deltaVelocityFiltered[-1]

        self.reportedPosPrev = self.reportedPos
        self.reportedTimestampPrev = timestamp

    def zscoreFilter(self, averagedpos):
        """
        Filters out outlier positions based on Z-scores and returns the mean of the remaining positions.

        Parameters:
        averagedpos (list or numpy.ndarray): A list or array of positions where each position is a list or array of
        coordinates.

        Returns:
        tuple: The mean position of the filtered positions as a tuple of coordinates.

        The method performs the following steps:
        1. Converts the input positions to a numpy array.
        2. Calculates the mean and standard deviation for each coordinate.
        3. Computes the Z-scores for each coordinate.
        4. Filters out positions where any coordinate has a Z-score greater than the threshold.
        5. Calculates the mean of the remaining positions.
        6. Returns the mean position as a tuple.
        """
        averagedpos = numpy.array(averagedpos)

        # Calculate mean and standard deviation for each coordinate
        mean_pos = numpy.mean(averagedpos, axis=0)
        std_pos = numpy.std(averagedpos, axis=0)

        # Calculate Z-scores for each coordinate
        z_scores = numpy.abs((averagedpos - mean_pos) / std_pos)

        # Filter out positions with any coordinate having a Z-score greater than the threshold
        filtered_positions = averagedpos[(z_scores < self.threshold).all(axis=1)]

        # Calculate the mean of the remaining positions
        mean_filtered_pos = numpy.mean(filtered_positions, axis=0)

        newPos = tuple(mean_filtered_pos)
        return newPos

    def getReportedVelocity(self):
        '''get the current reported velocity'''
        return self.reportedVelocity

    def getCurrentPosition(self):
        '''get the vehicle's current position in xyz'''
        T_VehToWorld = self.T_CamtoVeh @ self.T_CamToWorld[-1]
        # [0:3, 3]
        return getPos(T_VehToWorld)

    def getCurrentRotation(self, radians=False):
        '''get the vehicle's current position in xyz'''
        T_VehToWorld = self.T_CamtoVeh @ self.T_CamToWorld[-1]
        return getRotation(T_VehToWorld, radians)

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
                # t is the time now, t-1 is the previous frame - where T_CamToWorld is at this point
                # tag is the same world position at both orig and duplicate
                # World frame is time-independent
                # TOrig(World<-Tag) = T(World <- Cam_t) * TDup(Cam_t<-Tag)
                # and:
                # T(World <- Cam_t-1) = T(World <- Cam_t) * T(Cam_t <- Cam_t-1)
                #
                # Thus
                # T(World <- Cam_t) = T(World <- Cam_t-1) * T(Cam_t <- Cam_t-1)^-1
                # then
                # TOrig(World<-Tag) = T(World <- Cam_t-1) * T(Cam_t <- Cam_t-1)^-1 * TDup(Cam_t<-Tag)
                # Thus
                # T(Cam_t <- Cam_t-1) = TDup(Cam_t<-Tag) * TOrig(World<-Tag)^-1 * T(World <- Cam_t-1)
                Ttprevtocur = tagT @ numpy.linalg.inv(
                    self.tagPlacement[tagid]) @ self.T_CamToWorld[-1]
                # print("tagT =\n{0}".format(tagT))
                # print("self.tagPlacement[tagid]^-1 =\n{0}".format(numpy.linalg.inv(self.tagPlacement[tagid])))
                # print("self.T_CamToWorld(old) =\n{0}".format(self.T_CamToWorld))
                # and figure out summed distances between transformed new point to old (in world frame)
                summeddist = 0
                for tagidj, tagTj in self.tagDuplicatesT.items():
                    PosDelta = self.tagPlacement[tagidj] @ [[0], [0], [0], [
                        1]] - self.T_CamToWorld[-1] @ numpy.linalg.inv(Ttprevtocur) @ tagTj @ [[0], [0], [0], [1]]
                    summeddist += mag(PosDelta)

                if lowestCost > summeddist:
                    if self.debug:
                        print("Using tag {0} with error {1:.3f}m".format(
                            tagid, summeddist))
                    lowestCost = summeddist
                    bestTransform = Ttprevtocur

            if lowestCost > 0.5:
                print("WARNING: bad position estimate. Ignoring this frame.")
            else:
                # We have our least-cost transform
                self.T_CamToWorld.append(self.T_CamToWorld[-1] @ numpy.linalg.inv(
                    bestTransform))
                self.timestamps.append(timestamp)
                if self.debug:
                    print("Delta {0}, Rot = {1}".format(getPos(bestTransform).round(3),
                                                        getRotation(bestTransform).round(1)))

                # Update position, rotation, velocity
                self.generateReportedLoc(timestamp)
            if self.debug:
                print("New Pos {0}, Rot = {2} with {1} tags".format(self.getCurrentPosition().round(3),
                                                                    len(self.tagDuplicatesT),
                                                                    self.getCurrentRotation().round(1)))
        # finally add any new tags
        for tagid, tagT in self.tagnewT.items():
            # T(World <- tag) = T(World <- Cam_t) * T(Cam_t <- tag)
            self.tagPlacement[tagid] = self.T_CamToWorld[-1] @ tagT
            if self.debug:
                print("Added Tag ID {0} at pos {1}, rot {2}".format(tagid, getPos(self.tagPlacement[tagid]).round(3),
                                                                    getRotation(self.tagPlacement[tagid]).round(1)))
