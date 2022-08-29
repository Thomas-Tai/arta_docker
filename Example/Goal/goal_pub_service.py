#!/usr/bin/env python3

from __future__ import print_function

from goal_pub.srv import goalTransfer, goalTransferResponse

from geometry_msgs.msg import PoseStamped, Pose, Quaternion,PoseWithCovarianceStamped
from nav_msgs.msg import Odometry 

from tf.transformations import euler_from_quaternion, quaternion_from_euler

import rospy
import copy
import numpy as np
import math

class goalPub:
    def __init__(self):
        rospy.init_node('curr_goal_server')

        self.odomGoal = PoseStamped()
        self.odomGoal.header.seq = 0
        self.odomGoal.header.stamp = rospy.get_rostime()
        self.odomGoal.header.frame_id = "odom"
        #self.odomGoal.header.frame_id = "map"
        self.odomGoal.pose.position.x = 0.0
        self.odomGoal.pose.position.y = 0.0
        self.odomGoal.pose.orientation.w = 1.0      # This is in Quaternion mode

        self.incGoalX = 0.0
        self.incGoalY = 0.0
        self.incGoalYaw = 0.0

        #self.PSPQuaternionYaw = 0.0

        # Using fix frame as Odom / amcl_pose
        self.ao = rospy.Subscriber("arta/odom", Odometry, self.OdomCallback)
        #self.ao = rospy.Subscriber("arta_navigation/amcl_pose", PoseWithCovarianceStamped, self.OdomCallback)

        self.currOdomPose = Pose()

        
        self.s = rospy.Service('zswcc/curr_goal_pub', goalTransfer, self.handle_goal)

        #odomGoal_pub_frequency = rospy.get_param("~odomGoal_pub_frequency", 5.0)
        self.odomGoal_pub = rospy.Publisher("arta_navigation/move_base_simple/goal", PoseStamped, queue_size=1)

        print("ready to publish Goal to - /arta_navigation/move_base_simple/goal")
        rospy.spin()

    def pi_clip(self,angle):
            # avoid the angle bounding between -3.14 and 3.14
            if angle > 0:
                if angle > math.pi:
                    return angle - 2*math.pi
            else:
                if angle < -math.pi:
                    return angle + 2*math.pi
            return angle

    def OdomCallback(self,data=None):
        self.currOdomPose = data.pose.pose
        #print("self.currOdomPose is {}".format(self.currOdomPose))

    def handle_goal(self,req=None):
        if req.seq > self.odomGoal.header.seq:
            self.odomGoal.header.seq = copy.copy(int(req.seq))

            # Odom pose to Curr Pose
            (roll,pitch,currYaw) = euler_from_quaternion([self.currOdomPose.orientation.x,self.currOdomPose.orientation.y,self.currOdomPose.orientation.z,self.currOdomPose.orientation.w])

            currOdomPose = np.array([[self.currOdomPose.position.x],[self.currOdomPose.position.y]])
            
            c,s = np.cos(currYaw), np.sin(currYaw)
            R = np.array([[c, -s], [s, c]])
            IGoal = np.array([[req.IGoalX],[req.IGoalY]])
            odomIGoal = R.dot(IGoal)
            odomGoal = currOdomPose + odomIGoal
            self.odomGoal.pose.position.x = odomGoal[0][0]
            self.odomGoal.pose.position.y = odomGoal[1][0]
            
            #targetYaw = currYaw + req.IGoalYaw
            #print("currYaw is {}".format(currYaw))
            #print("req.IGoalYaw is {}".format(req.IGoalYaw))
            #print("targetYaw is {}".format(targetYaw))
            targetYaw = self.pi_clip(currYaw + req.IGoalYaw)
            #print("targetYaw is {}".format(targetYaw))
            (self.odomGoal.pose.orientation.x,self.odomGoal.pose.orientation.y,self.odomGoal.pose.orientation.z,self.odomGoal.pose.orientation.w) = quaternion_from_euler(roll,pitch,targetYaw)

            #self.odomGoal.pose.orientation.w = targetYaw

            #self.odomGoal.pose.position.x = self.odomGoal.pose.position.x + math.cos(targetYaw)
            #self.odomGoal.pose.position.y = self.odomGoal.pose.position.y + math.sin(targetYaw)

            self.odomGoal.header.stamp = rospy.get_rostime()

            self.odomGoal_pub.publish(self.odomGoal)

            return goalTransferResponse("-Goal_updated-")
        else:
            return goalTransferResponse("-Goal_not_updated-")
            
if __name__ == "__main__":
    
    #pub = rospy.Publisher('Here_is_the_location', Pose, queue_size=10)
    goalPublish = goalPub()
