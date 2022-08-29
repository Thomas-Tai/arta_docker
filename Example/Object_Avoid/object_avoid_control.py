#!/usr/bin/env python3

import rospy

from geometry_msgs.msg import Twist,Point
from std_msgs.msg import String

from gazebo_msgs.srv import GetModelState
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from swc_control.srv import lidarReading, lidarReadingResponse
import copy
import math
import time

msg = """
This is a script intend to avoid the obstacle (dynamic & static)

"""
class decided_goal:
    def __init__(self):
        print("init")
        # decided goal - final goal processed after all the calculation
        self.dgoalX = 0.0
        self.dgoalY = 0.0
        self.dspeed = 0.0

        # temp main goal - processed without any obj. avoid
        self.tmgoalX = 0.0
        self.tmgoalY = 0.0
        self.tmspeed = 0.0

        # robot states publish from auto_cmd_control.py - zswcc/curr_robot_state_pub
        self.currPoseX = 0.0
        self.currPoseY = 0.0
        self.currPoseYaw = 0.0
        self.currGoalX = 0.0
        self.currGoalY = 0.0
        self.currSpeed = 0.0

        self.direction = True

        goal_pub_freq = rospy.get_param("~goal_pub_freq", 10.0)
        self.decided_goal_pub = rospy.Publisher("zswcc/decided_goal_pub", Point, queue_size=1)
        rospy.Timer(rospy.Duration(1.0/goal_pub_freq), self.decided_goal_msg)

        rospy.Subscriber("zswcc/curr_robot_state_pub", Twist, self.curr_robot_state)

    def decided_goal_msg(self,req = None):
        dgoal = Point()
        dgoal.x = self.dgoalX
        dgoal.y = self.dgoalY
        dgoal.z = self.dspeed

        self.decided_goal_pub.publish(dgoal)

    def curr_robot_state(self,req = None):
        if req is not None:
            self.currPoseX = req.linear.x
            self.currPoseY = req.linear.y
            self.currPoseYaw = req.linear.z
            self.currGoalX = req.angular.x
            self.currGoalY = req.angular.y
            self.currSpeed = req.angular.z

    def temp_main_goal(self,section=None,reqGoal=None):
        oriX,oriY = 0.1,0.1
        GoriX,GoriY = 5,0.1
        #direction = True
        self.tmspeed = 0.7

        # calculate the difference
        xDiff = self.currGoalX - self.currPoseX
        yDiff = self.currGoalY - self.currPoseY
        #angle2goal = math.atan2(yDiff+0.01, xDiff+0.01)
        normalDiff = math.sqrt(xDiff**2+yDiff**2+0.01)

        if abs(normalDiff) < 0.3:
            #time.sleep(1)
            if self.direction is True:
                if self.currGoalX == GoriX and self.currGoalY == GoriY:
                    self.tmgoalX, self.tmgoalY = oriX,oriY
                    self.direction = False
                else:
                    self.tmgoalX, self.tmgoalY = GoriX,GoriY
            else:
                if self.currGoalX == oriX and self.currGoalY == oriY:
                    self.tmgoalX, self.tmgoalY = GoriX,GoriY
                    self.direction = True
                else:
                    self.tmgoalX, self.tmgoalY = oriX,oriY
        print(">>>>>>>>>>>   Current direction is {}".format(self.direction))

    def stop(self,req=None):
        self.dgoalX = self.currPoseX
        self.dgoalY = self.currPoseY

        self.decided_goal_msg()

class objAdvoid:

    def __init__(self,decided_goal_obj):
        self.SOA = False
        self.DOA = False

        self.tmpGoalX = 0.0
        self.tmpGoalY = 0.0
        self.tmpGoalSpeed = 0.0

        self.SOADirAvailable = False
        self.SOADirX = 0.0
        self.SOADirY = 0.0
        self.SOADirAngle = 0.0
        #self.SOAGoalSpeed = 0.0

        self.DGO = decided_goal_obj

        self.respCLB = [False]*12
        # angle rangle for all the section
        self.respCL_A = list(range(-6,7,1))
        self.respCL_A = list(map(lambda x: x*math.pi/6, self.respCL_A))
        #print("self.respCL_A is {}".format(self.respCL_A))

        self.mustStopMargin = 0.8
        self.safeMargin = 1.3      # 0.9m

    def get_tmgoal(self):
        #print("self.DGO.tmgoalX is {}".format(self.DGO.tmgoalX))
        self.tmpGoalX = self.DGO.tmgoalX
        self.tmpGoalY = self.DGO.tmgoalY
        self.tmpGoalSpeed = self.DGO.tmspeed

    def find_angle_falled_section(self,currAngle):
        # i.e. If currAngle > -3.14 and < -2.62, return 1
        NumList = self.respCL_A
        left = 0
        right = len(NumList)-1
        while left <= right:
            mid = left + ((right-left)>>1)
            if NumList[mid]>=currAngle and NumList[mid-1] < currAngle:
                return mid
            elif NumList[0]>=currAngle:
                print("There is an error for currAngle, it is smaller than -3.14 !!!")
                return 0
            elif NumList[mid]>=currAngle:
                right = mid-1
            else:
                left = mid+1
        print("There is an error for currAngle, it is bigger than 3.14 !!!")
        return len(NumList)

    def the12SectionsClip(self,section):
        if section < 0:
            section = section + 12
        elif section > 11:
            section = section - 12
        return section

    def pi_clip(self,angle):
            # avoid the angle bounding between -3.14 and 3.14
            if angle > 0:
                if angle > math.pi:
                    return angle - 2*math.pi
            else:
                if angle < -math.pi:
                    return angle + 2*math.pi
            return angle

    def crashPredictedFun(self,respCL,tmpGoalX=None,tmpGoalY=None):
        self.respCLB = list(map(lambda x: True if x < self.safeMargin else False, respCL))     # set to True if a segment is in danger distance (dist from center of SWC < 1m)
                
        # For testing - testing data
        #respCL = [1.9447627067565918, 0.7034728527069092, 0.8252302408218384, 1.2040328979492188, 0.7781926989555359, 2.4923880100250244, 1.237562894821167, 0.8328360319137573, 0.8328360319137573, 1.285671591758728, 2.008408546447754, 2.009399652481079]
        
        if tmpGoalX is None:
            tmpGoalX = self.tmpGoalX
            tmpGoalY = self.tmpGoalY

        # calculate the angle of current robot position to the goal position
        xDiff = tmpGoalX - self.DGO.currPoseX
        yDiff = tmpGoalY - self.DGO.currPoseY

        tmpGoalAngle = math.atan2(yDiff, xDiff)
        tmpGoalAngle = tmpGoalAngle - self.DGO.currPoseYaw              # transfer to degree according to the robot
        tmpGoalAngle = self.pi_clip(tmpGoalAngle)

        FirstBiggerInd = self.find_angle_falled_section(tmpGoalAngle)
        
        tmpGoalAngleSection = FirstBiggerInd - 1                        # the tmpGoalAngle fall between tmpGoalAngleSection and FirstBiggerInd
        return self.respCLB[tmpGoalAngleSection],tmpGoalAngleSection,tmpGoalAngle,FirstBiggerInd

    def static_obj_avoid(self):
        print("judging from lidar")
        self.get_tmgoal()   # get tmgoal val from decided_goal
        #rospy.wait_for_service('zswcc/curr_lidarReading_pub')
        try: 
            Cur_lidar_state = rospy.ServiceProxy('zswcc/curr_lidarReading_pub', lidarReading)
            respCL = (Cur_lidar_state(True)).floatArray

            self.respCLBMS = list(map(lambda x: True if x < self.mustStopMargin else False, respCL))
            if any(self.respCLBMS) is True:
                print("##### Not enough room!!! emergency stop activated #####")
                self.tmpGoalX = self.DGO.currPoseX
                self.tmpGoalY = self.DGO.currPoseY
            else:

                """self.respCLB = list(map(lambda x: True if x < self.safeMargin else False, respCL))     # set to True if a segment is in danger distance (dist from center of SWC < 1m)
                
                # For testing - testing data
                #respCL = [1.9447627067565918, 0.7034728527069092, 0.8252302408218384, 1.2040328979492188, 0.7781926989555359, 2.4923880100250244, 1.237562894821167, 0.8328360319137573, 0.8328360319137573, 1.285671591758728, 2.008408546447754, 2.009399652481079]
                
                # calculate the angle of current robot position to the goal position
                xDiff = self.tmpGoalX - self.DGO.currPoseX
                yDiff = self.tmpGoalY - self.DGO.currPoseY

                tmpGoalAngle = math.atan2(yDiff, xDiff)
                tmpGoalAngle = tmpGoalAngle - self.DGO.currPoseYaw              # transfer to degree according to the robot
                tmpGoalAngle = self.pi_clip(tmpGoalAngle)

                FirstBiggerInd = self.find_angle_falled_section(tmpGoalAngle)
                
                tmpGoalAngleSection = FirstBiggerInd - 1                        # the tmpGoalAngle fall between tmpGoalAngleSection and FirstBiggerInd
                crashPredicted = self.respCLB[tmpGoalAngleSection]"""
                #print("tmpGoalAngle is {}".format(tmpGoalAngle))
                #print("self.respCL_A is {}".format(self.respCL_A))
                #print("respCLB is {}".format(self.respCLB))
                #print("tmpGoalAngleSection is {}".format(self.respCLB[tmpGoalAngleSection]))
                #print("self.DGO.currPoseX is {}, self.DGO.currPoseY is {}".format(self.DGO.currPoseX,self.DGO.currPoseY))

                #tmpGoalFirstBiggerInd = self.find_angle_falled_section(tmpGoalAngle)

                crashPredicted,tmpGoalAngleSection,tmpGoalAngle,FirstBiggerInd = self.crashPredictedFun(respCL)

                CTGAFSM = copy.deepcopy(tmpGoalAngleSection) #CTGAFSM
                safeSectionIdxIncrement = 0
                safeSectionIdxIncrementInitDir = 0

                
                if crashPredicted is True and self.SOADirAvailable is True:
                    # If current dir to tmpgoal is false but previous safe direction is available 

                    PreSafeAngleDir2X = math.cos(self.SOADirAngle)
                    PreSafeAngleDir2Y = math.cos(self.SOADirAngle)

                    tmpGoalX = self.DGO.currPoseX + PreSafeAngleDir2X * 1.2
                    tmpGoalY = self.DGO.currPoseY + PreSafeAngleDir2Y * 1.2

                    PreCrashPredicted,_,_,_ = self.crashPredictedFun(respCL,tmpGoalX,tmpGoalY)

                    if PreCrashPredicted == False:
                        # If safe to use previous direction
                        print("--***-- Using previous safe goal --***--")
                        self.tmpGoalX = self.DGO.currPoseX + PreSafeAngleDir2X * 1.1
                        self.tmpGoalY = self.DGO.currPoseX + PreSafeAngleDir2Y * 1.1
                    else:
                        self.SOADirAvailable is False
                    
                elif crashPredicted is True and self.SOADirAvailable is False:
                    print("edit goal")
                    distLowerBound = abs(tmpGoalAngle - self.respCL_A[tmpGoalAngleSection])
                    distUpperBound = abs(tmpGoalAngle - self.respCL_A[FirstBiggerInd])
                    if distLowerBound <= distUpperBound:
                        # This determine the direction of searching safe section
                        safeSectionIdxIncrementInitDir = -1
                        safeSectionIdxIncrement = -1

                    else:
                        safeSectionIdxIncrementInitDir = 1
                        safeSectionIdxIncrement = 1

                         
                    CTGAFSM = CTGAFSM + safeSectionIdxIncrementInitDir
                    #CTGAFSL = CTGAFSM - 1
                    #CTGAFSR = CTGAFSM + 1

                    CTGAFSM = self.the12SectionsClip(CTGAFSM)
                    CTGAFSL = self.the12SectionsClip(CTGAFSM - 1)
                    CTGAFSR = self.the12SectionsClip(CTGAFSM + 1)

                    print("***CTGAFSM before while loop is {}".format(CTGAFSM))
                    print("self.respCLB is {}".format(self.respCLB))
                    #print("CTGAFSL is {}".format(CTGAFSL))
                    #print("CTGAFSR is {}".format(CTGAFSR))
                    #print("self.respCLB[int(CTGAFSL)] is {}".format(self.respCLB[int(CTGAFSL)]))
                    #print("self.respCLB[int(CTGAFSM)] is {}".format(self.respCLB[int(CTGAFSM)]))
                    #print("self.respCLB[int(CTGAFSR)] is {}".format(self.respCLB[int(CTGAFSR)]))

                    CTGAFSLB = self.respCLB[int(CTGAFSL)]
                    CTGAFSMB = self.respCLB[int(CTGAFSM)]
                    CTGAFSRB = self.respCLB[int(CTGAFSR)]
                    
                    i = 0

                    while (CTGAFSMB is True) or (CTGAFSLB is True) or (CTGAFSRB is True):
                        rospy.sleep(0.1)
                        #print("CTGAFSM is {}".format(CTGAFSM))
                        ## 
                        #if safeSectionIdxIncrementInitDir * safeSectionIdxIncrement > 0:
                        if i%2 == 0: #math.copysign(1, safeSectionIdxIncrementInitDir) == math.copysign(1, safeSectionIdxIncrement):
                            safeSectionIdxIncrement = safeSectionIdxIncrement * -1      # try the other direction
                            #print("1st section")
                        else:
                            safeSectionIdxIncrement = math.copysign(1, safeSectionIdxIncrement) * (abs(safeSectionIdxIncrement) + 1)    # 1 step further with current direction
                            #print("2nd section")
                        i = i + 1
                        ##
                        CTGAFSM = tmpGoalAngleSection + safeSectionIdxIncrement
                        if CTGAFSM > (11+11) or CTGAFSM < (-11-11):
                            rospy.loginfo("************* Static Object avoid calculation Failed with all direction section is False! *************")

                        else:
                            CTGAFSL = CTGAFSM - 1
                            CTGAFSR = CTGAFSM + 1

                        CTGAFSM = self.the12SectionsClip(CTGAFSM)
                        CTGAFSL = self.the12SectionsClip(CTGAFSL)
                        CTGAFSR = self.the12SectionsClip(CTGAFSR)
                        
                        #CTGAFSMC = copy.copy(CTGAFSM)
                        #print("CTGAFSMC is {}".format(CTGAFSMC))
                        CTGAFSLB = self.respCLB[int(CTGAFSL)]
                        #CTGAFSMB = self.respCLB[int(CTGAFSMC)]
                        CTGAFSRB = self.respCLB[int(CTGAFSR)]

                        CTGAFSMB = self.respCLB[int(CTGAFSM)]

                    #safeAngleIdx = copy.copy(CTGAFSMC)
                    safeAngleIdx = copy.copy(CTGAFSM)

                    # safe obj advoid goal
                    #safeValueMargin = min(respCL[int(CTGAFSL)],respCL[int(CTGAFSMC)],respCL[int(CTGAFSR)]) - self.safeMargin
                    #safeValueMargin = max(min(safeValueMargin,1),0)

                    if safeSectionIdxIncrementInitDir == 1:
                        safeAngleIdx = safeAngleIdx + 1     # to match the initial direction
                    safeAngleDir = self.respCL_A[int(safeAngleIdx)]

                    safeAngleDir2X = math.cos(safeAngleDir) * 1 #+ 1 * safeValueMargin
                    safeAngleDir2Y = math.sin(safeAngleDir) * 1 #+ 1 * safeValueMargin
                    SAnormalDiff = math.sqrt(safeAngleDir2X**2+safeAngleDir2Y**2)

                    self.SOADirAvailable = True
                    #self.SOADirX = copy.copy(safeAngleDir2X)
                    #self.SOADirY = copy.copy(safeAngleDir2Y)

                    self.tmpGoalX = self.DGO.currPoseX + safeAngleDir2X
                    self.tmpGoalY = self.DGO.currPoseY + safeAngleDir2Y

                    self.SOADirAngle = math.atan2(self.tmpGoalY, self.tmpGoalX)

                    #print("safeAngleDir is {}".format(safeAngleDir))
                    #print("SAnormalDiff is {}".format(SAnormalDiff))
                    #print("self.tmpGoalX is {}".format(self.tmpGoalX))
                    #print("self.tmpGoalY is {}".format(self.tmpGoalY))
                    self.SOA = True

                    print("***CTGAFSM after while loop is {}".format(int(CTGAFSM)))

                else:
                    self.SOA = True
                    # clear SOA safe when direction to safe is available
                    self.SOADirAvailable = False
                    self.SOADirX = 0
                    self.SOADirY = 0
                rospy.loginfo("Static Object avoid calculation successful")


        except rospy.ServiceException as e:
            rospy.loginfo("Get Lidar State service call failed : {0}".format(e))

    def dynamic_obj_avoid(self):
        print("avoiding human")

    def objAdvoidPass(self):
        ### For test only
        #self.SOA = True
        self.DOA = True
        ###
        if self.SOA is True and self.DOA is True:
            print("-objAdvoidPassed-")
            self.DGO.dgoalX = self.tmpGoalX
            self.DGO.dgoalY = self.tmpGoalY
            self.DGO.dspeed = self.tmpGoalSpeed
            #print("self.DGO.dgoalX is {}".format(self.DGO.dgoalX))
            #print("self.DGO.dgoalY is {}".format(self.DGO.dgoalY))
            #print("self.DGO.dspeed is {}".format(self.DGO.dspeed))
            self.DGO.decided_goal_msg()
        else:
            rospy.loginfo("Object avoid calculation failed")

if __name__ == "__main__":
    rospy.init_node("decide_goal_node")
    rospy.loginfo("decide_goal_node is now started, ready to get commands.") 

    dGoal = decided_goal()
    rospy.on_shutdown(dGoal.stop)
    objAdvoids = objAdvoid(dGoal)

    #rate = rospy.Rate(1)    # 1 Hz
    while True:
        #time.sleep(1)
        rospy.sleep(0.3)
        dGoal.temp_main_goal()
        objAdvoids.get_tmgoal()
        objAdvoids.static_obj_avoid()
        objAdvoids.objAdvoidPass()
        

    #rospy.Subscriber("current_human_intention", String, cur_movement.BP_I_response)

    #rospy.on_shutdown(cur_movement.stop)

    r = rospy.Rate(4)

    rospy.spin()