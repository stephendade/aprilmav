'''
AprilTag detection module and shortcuts
'''
import os
import numpy
import cv2
from pyapriltags import Detector


class ApriltagDectection:
    '''
    Placeholder for detected tag information when using OpenCV
    '''
    tag_id = None
    corners = None
    center = None
    hamming = 0
    decision_margin = 0
    homography = None
    pose_R = None
    pose_t = None
    pose_err = None


class aprilDetect:
    '''
    Apriltag detector, using either pyapriltags or OpenCV
    '''
    def __init__(self, tagSize, tagFamily="tag36h11", decimate=1.0, OpenCV=False):
        self.tagSize = tagSize/1000
        self.decimate = decimate
        self.at_detector = None
        self.OpenCV = OpenCV
        self.tagFamily = tagFamily
        self.PrevFrameTags = []

        if OpenCV:
            # Setup Aruco detector
            aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36H11)
            aruco_parameters = cv2.aruco.DetectorParameters()
            aruco_parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
            aruco_parameters.aprilTagQuadDecimate = self.decimate
            aruco_parameters.markerBorderBits = 1
            aruco_parameters.adaptiveThreshWinSizeMin = 10
            aruco_parameters.adaptiveThreshWinSizeMax = 20
            aruco_parameters.adaptiveThreshWinSizeStep = 10
            aruco_parameters.cornerRefinementMaxIterations = 20
            aruco_parameters.cornerRefinementMinAccuracy = 0.20
            aruco_parameters.minCornerDistanceRate = 0.05
            self.at_detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_parameters)

            # Define 3D points of tag corners (assuming tag is on XY plane)
            self.objPoints = numpy.array([
                [-self.tagSize/2, self.tagSize/2, 0],
                [self.tagSize/2, self.tagSize/2, 0],
                [self.tagSize/2, -self.tagSize/2, 0],
                [-self.tagSize/2, -self.tagSize/2, 0]
            ])
        else:
            self.at_detector = Detector(searchpath=['apriltags3py/apriltags/lib', 'apriltags3py/apriltags/lib'],
                                        families=self.tagFamily,
                                        nthreads=max(1, os.cpu_count() - 1),
                                        quad_decimate=self.decimate,
                                        quad_sigma=0.0,
                                        refine_edges=1,
                                        decode_sharpening=0.25,
                                        debug=0)

    def detect(self, image, K):
        '''
        Detect tags in an image
        :param image: The image to detect tags in
        :param K: The camera matrix
        :return: A list of detected tags
        '''
        if self.OpenCV:
            # Detect tags
            corners, ids, _ = self.at_detector.detectMarkers(image)

            # Determine each tag position
            tags = []
            if ids is not None:
                for idx, tagid in enumerate(ids):
                    # Calculate pose for each tag
                    success, rvec, tvec = cv2.solvePnP(
                        self.objPoints,
                        corners[idx][0],
                        K,
                        None,
                        flags=cv2.SOLVEPNP_IPPE_SQUARE
                    )
                    if success:
                        # Calculate pose error
                        pose_err = numpy.linalg.norm(tvec)
                        rotation_matrix, _ = cv2.Rodrigues(rvec)
                        tag = ApriltagDectection()
                        tag.tag_id = tagid[0]
                        tag.corners = corners[idx][0]
                        tag.center = corners[idx][0].mean(axis=0)
                        tag.hamming = 0
                        tag.decision_margin = 0
                        tag.homography = None
                        tag.pose_R = rotation_matrix
                        tag.pose_t = tvec
                        tag.pose_err = pose_err/1E8

                        tags.append(tag)
        else:
            # Detect tags
            tags = self.at_detector.detect(image, True, K, self.tagSize)

        # Determine if tag rotation is similar to previous frame and filter out if not
        filteredTags = self.filterTags(tags)

        return filteredTags

    def filterTags(self, tags):
        '''
        Filter out tags that have a rotation that is not similar to the previous frame
        :param tags: The tags to filter
        :return: The filtered tags
        '''
        # Determine if tag rotation is similar (<30 degrees) to same tag in previous frame and filter out if not
        filteredTags = []
        for tag in tags:
            found = False
            for prevTag in self.PrevFrameTags:
                if tag.tag_id == prevTag.tag_id:
                    diff = self.getAngle(prevTag.pose_R, tag.pose_R)
                    if abs(diff) < 30:   # degrees
                        filteredTags.append(tag)
                    # else:
                    #     print(f"Ignoring flipped tag {tag.tag_id}, delta rotation = {diff}")
                    found = True
                    break
            if not found:
                # new tag
                filteredTags.append(tag)

        self.PrevFrameTags = tags

        return filteredTags

    @staticmethod
    def getAngle(P, Q):
        '''
        Calculate the angle between two rotation matrices
        :param P: The first rotation matrix
        :param Q: The second rotation matrix
        :return: The angle between the two rotation matrices in degrees
        '''
        R = numpy.dot(P, numpy.transpose(Q))
        cos_theta = (numpy.trace(R)-1)/2
        return numpy.arccos(cos_theta) * (180/numpy.pi)
