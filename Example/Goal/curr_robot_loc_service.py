#!/usr/bin/env python3

from __future__ import print_function

from swc_control.srv import positionTransfer, positionTransferResponse
from nav_msgs.msg import Odometry 
from geometry_msgs.msg import Twist,Point
from std_msgs.msg import String

from tf.transformations import euler_from_quaternion

import numpy as np

import rospy

import copy

class publishLoc:
    def __init__(self):
        rospy.init_node('curr_Location_server')

        if not rospy.has_param("/SWC_StartTime"):
            #startTime = rospy.get_time()
            startTime = float("{:.3f}".format(rospy.get_time()))
            rospy.set_param("/SWC_StartTime",startTime)
        self.systemStartTime = rospy.get_param("/SWC_StartTime")
        #print(self.systemStartTime)
        
        self.odom = Odometry()
        self.odomR = Odometry()
        self.past01SecOdom = Odometry()
        self.past01SecOdom.pose.pose.position.x = 0
        self.past01SecOdom.pose.pose.position.y = 0
        self.pastOdomTime = 0
        self.OdomVelX = 0
        self.OdomVelY = 0
        self.LocalVelX = 0
        self.LocalVelY = 0
        self.velUpdateDuration = 0.2

        self.temp = rospy.Subscriber("arta/odom", Odometry, self.callback)
        
        self.s = rospy.Service('zswcc/curr_Location_pub', positionTransfer, self.handle_positionTransfer)
        print("Ready to provide current point")
        rospy.spin()

    def callback(self,data=None):
        #global my_data
        #print(data.pose.pose)
        self.odom = data
        #my_data = data.pose.pose
        #pub.publish(data.pose.pose)
        #return data.pose.pose
        
        curOdomTime = float('%d.%d' % (self.odom.header.stamp.secs, self.odom.header.stamp.nsecs))
        if (curOdomTime - self.pastOdomTime) > self.velUpdateDuration:     # update per velUpdateDuration
            # update velocity
            self.OdomVelX = (self.odom.pose.pose.position.x - self.past01SecOdom.pose.pose.position.x) / self.velUpdateDuration
            self.OdomVelY = (self.odom.pose.pose.position.y - self.past01SecOdom.pose.pose.position.y) / self.velUpdateDuration
            
            (roll,pitch,yaw) = euler_from_quaternion([self.odom.pose.pose.orientation.x,self.odom.pose.pose.orientation.y,self.odom.pose.pose.orientation.z,self.odom.pose.pose.orientation.w])
            self.odom.pose.pose.position.z = yaw
            #print("self.odom.pose.pose.position.z is {}".format(self.odom.pose.pose.position.z))
            #print("yaw is {}".format(yaw))
            currOdomVel = np.array([[self.OdomVelX],[self.OdomVelY]])
            c,s = np.cos(-yaw), np.sin(-yaw)
            R = np.array([[c, -s], [s, c]])
            currLocVel = R.dot(currOdomVel)
            self.LocalVelX = currLocVel[0][0]
            self.LocalVelY = currLocVel[1][0]
            #self.LocalVelX = -1
            #self.LocalVelY = -1
            self.past01SecOdom = self.odom
            self.pastOdomTime = float('%d.%d' % (self.past01SecOdom.header.stamp.secs, self.past01SecOdom.header.stamp.nsecs))
            self.odomR = self.odom

    def handle_positionTransfer(self,req=None):
        #rospy.sleep(1) # Sleeps for 1 sec
        if req.request == True:
            #rospy.loginfo("Returning Location")
            OdomtimeNow = float('%d.%d' % (self.odom.header.stamp.secs, self.odom.header.stamp.nsecs))
            #OdomtimeNow = float("{:.3f}".format(OdomtimeNow))
            timeSinceSystemUp = float("{:.3f}".format(OdomtimeNow - self.systemStartTime))
            #print("rospy.get_time() is {}".format(rospy.get_time()))
            return positionTransferResponse(timeSinceSystemUp,self.odomR.pose.pose.position,self.odomR.pose.pose.orientation,[self.OdomVelX,self.OdomVelY],[self.LocalVelX,self.LocalVelY])
        

if __name__ == "__main__":
    
    #pub = rospy.Publisher('Here_is_the_location', Pose, queue_size=10)
    pubLoc = publishLoc()
