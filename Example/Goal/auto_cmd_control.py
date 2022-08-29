#!/usr/bin/env python3

from shutil import move

from pyrsistent import rex
import rospy

from geometry_msgs.msg import Twist,Point
from std_msgs.msg import String

from gazebo_msgs.srv import GetModelState
from swc_control.srv import positionTransfer, positionTransferResponse
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math
#from simple_pid import PID

import time

msg = """
This is a script intend to let the robot go straight in a loop

"""

class movement:

    def __init__(self):
        # control signal sent to TTK_cmdVel node
        self.simmode = False

        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.th = 1.0
        self.speed = 0.0
        self.turn = 0.0

        # current robot location
        self.poseX = 0.0
        self.poseY = 0.0
        self.poseYaw = 0.0

        # For receiving goal
        self.targetX = 0.0
        self.targetY = 0.0

        # For PID
        self.previous_angle = 0.0
        self.previous_dist = 0.0
        self.total_angle = 0.0
        self.total_dist = 0.0
        self.last_angle_sign = 0

        self.normalDiff = 0.0

        movement_pub_frequency = rospy.get_param("~movement_pub_frequency", 5.0)
        self.movement_pub = rospy.Publisher("zswcc/auto_movement_pub", Twist, queue_size=1)
        rospy.Timer(rospy.Duration(1.0/movement_pub_frequency), self.publish_movement_msg)
        self.curr_robot_state_pub = rospy.Publisher("zswcc/curr_robot_state_pub", Twist, queue_size=1)
        rospy.Timer(rospy.Duration(1.0/movement_pub_frequency), self.curr_robot_state_msg)

        #rospy.Subscriber("zswch/current_human_intention", String, self.BP_I_response)
        rospy.Subscriber("zswcc/decided_goal_pub", Point, self.decided_goal_react)


    def publish_movement_msg(self,event=None):
        twist_msg = Twist()
        twist_msg.linear.x = self.x
        twist_msg.linear.y = self.y
        twist_msg.linear.z = self.z
        twist_msg.angular.x = self.th
        twist_msg.angular.y = self.speed
        twist_msg.angular.z = self.turn
        self.movement_pub.publish(twist_msg)

    def curr_robot_state_msg(self,event=None):
        robot_state_msg = Twist()
        robot_state_msg.linear.x = self.poseX
        robot_state_msg.linear.y = self.poseY
        robot_state_msg.linear.z = self.poseYaw
        robot_state_msg.angular.x = self.targetX
        robot_state_msg.angular.y = self.targetY
        robot_state_msg.angular.z = self.speed
        self.curr_robot_state_pub.publish(robot_state_msg)

    def stop(self,req=None):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.th = 0.0
        self.speed = 0.0
        self.turn = 0.0
        # publish stop commend immediately
        self.publish_movement_msg()

    def go_forward(self,reverse=False):
        if not reverse:
            self.x = 1
        else:
            self.x = -1
        self.speed = 0.5

    def BP_I_response(self,req=None):
        print(req.data)
        if req.data == "left":
            self.turn = -1      # turn right
        elif req.data == "right":
            self.turn = 1       # turn left
        else:
            self.turn = 0
        self.speed = 0.5

    def pose_callback(self,req = None):
        return req.position.x

    def self_pose_update(self,sim=False):
        #sim = False # Testing
        sim = self.simmode
        if sim:
            rospy.wait_for_service('gazebo/get_model_state')
            try:
                G_get_model_state = rospy.ServiceProxy('gazebo/get_model_state', GetModelState)
                respG = G_get_model_state(model_name='arta')
                gPose = respG.pose.position
                gOrientation = respG.pose.orientation

                self.poseX = gPose.x
                self.poseY = gPose.y
                (roll,pitch,yaw) = euler_from_quaternion([gOrientation.x,gOrientation.y,gOrientation.z,gOrientation.w])
                self.poseYaw = yaw
            except rospy.ServiceException as e:
                rospy.loginfo("Get Model State service call failed : {0}".format(e))
        else:
            # add read rostopic from odom from robot
            rospy.wait_for_service('zswcc/curr_Location_pub')
            try:
                GR_get_model_state = rospy.ServiceProxy('zswcc/curr_Location_pub', positionTransfer)
                respGR = GR_get_model_state(True)
                gPose = respGR.position
                gOrientation = respGR.orientation

                self.poseX = gPose.x
                self.poseY = gPose.y
                (roll,pitch,yaw) = euler_from_quaternion([gOrientation.x,gOrientation.y,gOrientation.z,gOrientation.w])
                self.poseYaw = yaw
            except rospy.ServiceException as e:
                rospy.loginfo("Get Model State service call failed : {0}".format(e))




        print("current pose: X:{:.5f}, Y:{:.5f}, rad:{:.5f}, degree:{:.5f}".format(self.poseX,self.poseY,self.poseYaw,self.poseYaw*180/math.pi))

    def self_pose2target_update(self,goal=None):
        self.self_pose_update()

        if goal is not None:
            self.targetX = goal.x
            self.targetY = goal.y
            
        
        # calculate the difference
        xDiff = self.targetX - self.poseX
        yDiff = self.targetY - self.poseY
        angle2goal = math.atan2(yDiff+0.01, xDiff+0.01)
        self.normalDiff = math.sqrt(xDiff**2+yDiff**2+0.01)
        normalDiffLimit = max(min(self.normalDiff,1),-1) # limit to -1 till 1
        #print("angle2goal: {} self.poseYaw:{} self.normalDiff:{} maxminND:{}".format(angle2goal,self.poseYaw,self.normalDiff,normalDiffLimit))
        
        def pi_clip(angle):
            # avoid the angle bounding between -3.14 and 3.14
            if angle > 0:
                if angle > math.pi:
                    return angle - 2*math.pi
            else:
                if angle < -math.pi:
                    return angle + 2*math.pi
            return angle

        angleDiff = angle2goal - self.poseYaw
        angleDiff = pi_clip(angleDiff)
        print("difference output: {} {} {}".format(xDiff,yDiff,angleDiff))

        xKp = 1
        xKi = 0.0
        xKd = 0.1
        tKp = 1
        tKi = 0.0
        tKd = 0.1

        diff_angle = angleDiff - self.previous_angle
        diff_dist = self.normalDiff - self.previous_dist

        if abs(self.normalDiff) < 0.3:
            self.x = 0.0
            self.turn = 0.0
        elif abs(angleDiff) > 0.2:
            self.x = min(self.x, 0.5) * 0.9 # provide some remain velocity to avoid shaking!!!
            self.turn = math.copysign(1, angleDiff) * tKp * abs(angleDiff)/math.pi + tKi * self.total_angle + tKd * diff_angle  #math.copysign(1, (angleDiff)) * 
            if abs(self.turn) < 0.3:
                self.turn = math.copysign(1, angleDiff) * 0.32
        elif abs(self.normalDiff) > 0.3:
            self.x = xKp * normalDiffLimit + xKi * self.total_dist + xKd * diff_dist
            self.turn = min(self.turn,0.5) * 0.9 # provide some remain velocity to avoid shaking!!!
        else:
            self.x = 0.0
            self.turn = 0.0

        self.previous_angle = angleDiff
        self.previous_dist = self.normalDiff
        self.total_angle = self.total_angle + angleDiff
        self.total_dist = self.total_dist + self.normalDiff
        
        self.speed = 0.7
        print("output: {} {} {}".format(self.x,self.turn,self.speed))

    def loop_movement(self):
        goal = Point()
        oriX,oriY = 0,0
        goal.x = oriX
        goal.y = oriY
        self.self_pose2target_update(goal)
        #cur_movement.self_pose_update(sim=True)

        while True:
            self.self_pose2target_update()
            if abs(self.normalDiff) < 0.3:
                time.sleep(1)
                if goal.x == oriX and goal.y == oriY:
                    goal.x, goal.y = 1,0
                else:
                    goal.x, goal.y = oriX,oriY
                self.self_pose2target_update(goal)
                #time.sleep(1)
            time.sleep(0.1)
        
    def decided_goal_react(self,req=None):
        if req != None:
        #while True:
            if self.targetX != req.x or self.targetY != req.y or self.speed != req.z:
                decided_goal = Point()
                decided_goal.x = req.x
                decided_goal.y = req.y
                #self.speed = 0
                #self.self_pose2target_update(decided_goal)
                #rospy.sleep(0.5)        # stop a bit of time when goal change
                self.speed = req.z
                self.self_pose2target_update(decided_goal)
                print("Decided goal received X:{} Y:{} Speed:{}".format(decided_goal.x,decided_goal.y,self.speed))
            else:
                self.self_pose2target_update()
            

if __name__ == "__main__":
    rospy.init_node("automovement_driver")
    rospy.loginfo("automovement_driver is now started, ready to get commands.") 

    cur_movement = movement()

    #rospy.Subscriber("zswch/current_human_intention", String, cur_movement.BP_I_response)

    rospy.on_shutdown(cur_movement.stop)

    r = rospy.Rate(4)
    
    #cur_movement.loop_movement()


    #cur_movement.go_forward()
    #cur_movement.stop()

    rospy.spin()