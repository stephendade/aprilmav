'''
Geometrical functions
'''

import math
import numpy

#import transformations as tf 

def getTransform(tag):
    '''tag pose to transformation matrix'''
    T_Tag = numpy.array( numpy.eye((4)) )
    T_Tag[0:3, 0:3] = numpy.array(tag.pose_R)
    tag.pose_t = numpy.array(tag.pose_t)
    T_Tag[0:3, 3] = tag.pose_t.reshape(3)
    
    # flip x axis
    T_Tag = [[-1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]] @ T_Tag
    
    #now convert to NED coord frame
    # so x-90, y+90, z in order
    # https://www.andre-gaschler.com/rotationconverter/
    #[[0,0,1,0],[-1,0,0,0],[0,-1,0,0],[0,0,0,1]]
    #T_Tag = [[1,0,0,0],[0,0,1,0],[0,-1,0,0],[0,0,0,1]] @ T_Tag
    return T_Tag

def getPos(tag):
    '''output the transformation matrix position as xyz tuple'''
    return [tag[0,3], tag[1,3], tag[2,3]]

def getRotation(T_Tag):
    '''Get the vehicle's current rotation in RPY degrees'''
    sy = math.sqrt(T_Tag[0,0] * T_Tag[0,0] +  T_Tag[1,0] * T_Tag[1,0])
    singular = sy < 1e-6
    if  not singular :
        x = math.atan2(T_Tag[2,1] , T_Tag[2,2])
        y = math.atan2(-T_Tag[2,0], sy)
        z = math.atan2(T_Tag[1,0], T_Tag[0,0])
    else :
        x = math.atan2(-T_Tag[1,2], T_Tag[1,1])
        y = math.atan2(-T_Tag[2,0], sy)
        z = 0
        
    # z-y-x euler angles
    #yaw=math.atan2(T_Tag[1,0],T_Tag[0,0]);
    #pitch=math.atan2(-T_Tag[2,0],math.sqrt(math.pow(T_Tag[2,1], 2)+math.pow(T_Tag[2,2], 2)));
    #roll=math.atan2(T_Tag[2,1],T_Tag[2,2]);
    
    return numpy.array([math.degrees(x), math.degrees(y), math.degrees(z)]).round(3)

def isclose(x, y, rtol=1.e-5, atol=1.e-8):
    return abs(x-y) <= atol + rtol * abs(y)

def euler_angles_from_rotation_matrix(R):
    '''
    From a paper by Gregory G. Slabaugh (undated),
    "Computing Euler angles from a rotation matrix
    '''
    phi = 0.0
    if isclose(R[2,0],-1.0):
        theta = math.pi/2.0
        psi = math.atan2(R[0,1],R[0,2])
    elif isclose(R[2,0],1.0):
        theta = -math.pi/2.0
        psi = math.atan2(-R[0,1],-R[0,2])
    else:
        theta = -math.asin(R[2,0])
        cos_theta = math.cos(theta)
        psi = math.atan2(R[2,1]/cos_theta, R[2,2]/cos_theta)
        phi = math.atan2(R[1,0]/cos_theta, R[0,0]/cos_theta)
    return [math.degrees(psi), math.degrees(theta), math.degrees(phi)]
    
def mag(x): 
    return math.sqrt(sum(i**2 for i in x))
    
class tagDB:
    '''Database of all detected tags'''
    
    def __init__(self, deltax=0, deltay=0, deltaz=0):
        self.T_CamToWorld = numpy.array( numpy.eye((4)) )
        self.tagPlacement = {}
        self.tagnewT = {}
        self.tagDuplicatesT = {}
        self.T_CamtoVeh = numpy.array( numpy.eye((4)) )
        #self.P_CamtoVeh = numpy.array([deltax, deltay, deltaz])
        
    def newFrame(self):
        '''Reset the duplicates for a new frame of tags'''
        self.tagDuplicatesT = {}
        self.tagnewT = {}
        
    def addTag(self, tag):
        '''Add tag to database'''
        if tag.tag_id not in self.tagPlacement:
            # tag is in cur camera frame
            T_TagToCam = getTransform(tag)
            #T_TagToCam[0:3, 3] = T_TagToCam[0:3, 3]
            self.tagnewT[tag.tag_id] = T_TagToCam
            
            #print("Added Tag ID {0}, Qual {2}, T =\n {1}".format(tag.tag_id, tagPlacement[tag.tag_id].round(3), tag.pose_err))
            print("Added Tag ID {0} at pos {1}".format(tag.tag_id, getPos(self.tagnewT[tag.tag_id])))
        else:
            # get tag's last pos, in camera frame
            T_TagToCam = getTransform(tag)
            #T_TagToCam[0:3, 3] = T_TagToCam[0:3, 3]
            
            # save tag positions in Camera frame at time t for the duplicate
            self.tagDuplicatesT[tag.tag_id] = T_TagToCam
            print("Duplicate Tag ID {0} at pos {1}".format(tag.tag_id, getPos(self.tagDuplicatesT[tag.tag_id])))
            
        #print("R =\n{0}".format(tag.pose_R))
        #print("T =\n{0}".format(tag.pose_t))
        #print("Tag =\n{0}".format(T_TagToCam@[[0],[0],[0],[1]]))
            
        #print("Got Tag ID {0}, Qual {2}, T =\n {1}".format(tag.tag_id, numpy.format_float_positional(T_TagToCam.round(3)), tag.pose_err))
            
    def getCurrentPosition(self):
        '''get the vehicle's current position in xyz'''
        T_VehToWorld = self.T_CamtoVeh @ self.T_CamToWorld
        #[0:3, 3]
        return [T_VehToWorld[0,3], T_VehToWorld[1,3], T_VehToWorld[2,3]]

    def getCurrentRotation(self):
        R = self.T_CamToWorld
        '''Get the vehicle's current rotation in RPY degrees'''
        sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
        singular = sy < 1e-6
        if  not singular :
            x = math.atan2(R[2,1] , R[2,2])
            y = math.atan2(-R[2,0], sy)
            z = math.atan2(R[1,0], R[0,0])
        else :
            x = math.atan2(-R[1,2], R[1,1])
            y = math.atan2(-R[2,0], sy)
            z = 0
            
        # z-y-x euler angles
        yaw=math.atan2(R[1,0],R[0,0]);
        pitch=math.atan2(-R[2,0],math.sqrt(math.pow(R[2,1], 2)+math.pow(R[2,2], 2)));
        roll=math.atan2(R[2,1],R[2,2]);
        
        return numpy.array([math.degrees(roll), math.degrees(pitch), math.degrees(yaw)])
        
    def getTagdb(self):
        '''get coords of all tags by axis'''
        #xcoord = []
        #for tagid, tag in self.tagPlacement.items():
        #    xcoord.append(tag[0, axis])
        return self.tagPlacement

    def getTagPoints(self, axis):
        xcoord = []
        for tagid, tag in self.tagPlacement.items():
            xcoord.append(tag[axis,3])
        return xcoord
        
    def getBestTransform(self):
        '''Given the current self.tagDuplicatesT, what is the best fitting transform
        back to self.tagPlacement'''
        # get the least cost transform from the common points at time t-1 to time t
        # cost is the sum of position error between the common points, with the t-1 points
        # projected forward to t
        if len(self.tagDuplicatesT) > 0:
            bestTransform = -1
            lowestCost = 999
            # Use each tag pair as a guess for the correct transform - lowest cost wins
            for tagid, tagT in self.tagDuplicatesT.items():
                print("Trying {0}".format(tagid))
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
                Ttprevtocur =  tagT @ numpy.linalg.inv(self.tagPlacement[tagid]) @ self.T_CamToWorld
                #print("tagT =\n{0}".format(tagT))
                #print("self.tagPlacement[tagid]^-1 =\n{0}".format(numpy.linalg.inv(self.tagPlacement[tagid])))
                #print("self.T_CamToWorld(old) =\n{0}".format(self.T_CamToWorld))
                # and figure out summed distances between transformed new point to old (in world frame)
                summeddist = 0
                for tagidj, tagTj in self.tagDuplicatesT.items():
                    # PosDelta = TOrig(World<-Tag) * [0,0,0] - T(World <- Cam_t-1) * T(Cam_t <- Cam_t-1)^-1 * TDup(Cam_t<-Tag) * [0,0,0]
                    PosDelta = self.tagPlacement[tagidj]@[[0],[0],[0],[1]] - self.T_CamToWorld @ numpy.linalg.inv(Ttprevtocur) @ tagTj@[[0],[0],[0],[1]]
                    #print(self.tagPlacement[tagidj] - (self.T_CamToWorld @ numpy.linalg.inv(Ttprevtocur) @ tagTj))
                    summeddist += math.sqrt(math.pow(PosDelta[0], 2) + math.pow(PosDelta[1], 2) + math.pow(PosDelta[2], 2))
                    
                #print("Tag rot (Tag {1})= {0} deg".format(getRotation(Ttprevtocur), tagid))
                #print("Tag T (Tag {1})= {0} deg".format([Ttprevtocur[0,3],Ttprevtocur[1,3],Ttprevtocur[2,3]], tagid))
                if lowestCost > summeddist:
                    print("Using {0} {1}".format(tagid, summeddist))
                    lowestCost = summeddist
                    bestTransform = Ttprevtocur
                    
            #is it a based transform? If more than 5cm delta or summed 15deg rot
            #print((euler_angles_from_rotation_matrix(self.T_CamToWorld)))
            #print((euler_angles_from_rotation_matrix(numpy.linalg.inv(bestTransform))))
            #print(mag(getPos(bestTransform)))
            #print("self.bestTransform^-1 =\n{0}".format(numpy.linalg.inv(bestTransform)))
            #print("Pos {0}, Rot = {1}".format(getPos(bestTransform), getRotation(bestTransform)))
            #print("Pos {0}, Rot = {1}".format(getPos(numpy.linalg.inv(bestTransform)), getRotation(numpy.linalg.inv(bestTransform))))
            #print("self.bestTransform^-1 =\n{0}".format(numpy.linalg.inv(bestTransform)))
            #f mag(getPos(bestTransform)) > 0.1:
            #    print("Bad Translation")
            #    bestTransform = numpy.array( numpy.eye((4)) )
            #else:
            #    print("Tag =\n{0}".format(bestTransform))
            #    print("Tag =\n{0}".format(numpy.linalg.inv(bestTransform)))                    
            #we have the lowest cost transform (need inverse)
            # T(World <- Cam_t) = T(World <- Cam_t-1) * T(Cam_t <- Cam_t-1)^-1
            #print(numpy.linalg.inv(bestTransform))

            self.T_CamToWorld = self.T_CamToWorld @ numpy.linalg.inv(bestTransform)
            #print("self.T_CamToWorld(new) =\n{0}".format(self.T_CamToWorld))
            #print((euler_angles_from_rotation_matrix(self.T_CamToWorld)))

        print("Pos {0}, Rot = {2} with {1} tags".format(self.getCurrentPosition(), len(self.tagDuplicatesT), self.getCurrentRotation()))
        
        # finally add any new tags
        for tagid, tagT in self.tagnewT.items(): 
            # T(World <- tag) = T(World <- Cam_t) * T(Cam_t <- tag)
            self.tagPlacement[tagid] = self.T_CamToWorld @ tagT
            dist = math.sqrt(math.pow(tagT[0,3], 2) + math.pow(tagT[1,3], 2) + math.pow(tagT[2,3], 2))
            #print("Added Tag ID {0} at {2:.3}, T(world) =\n {1}".format(tagid, self.tagPlacement[tagid].round(3), dist))
            print("Added Tag ID {0} at pos {1}, rot {2}".format(tagid, getPos(self.tagPlacement[tagid]), getRotation(self.tagPlacement[tagid])))
            #         print("Pos {0}, Rot = {3} in {1} with {4}/{2} tags".format(tagPlacement.getCurrentPosition().round(3), file, len(tags), tagPlacement.getCurrentRotation().round(0), tagsused))