#!/usr/bin/env python3
# need update !!!!!!!!!!!!!
from __future__ import print_function

from swc_control.srv import lidarReading, lidarReadingResponse
from nav_msgs.msg import Odometry 
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist,Point
from std_msgs.msg import String

import rospy

import numpy as np

class publishLoc:
    def __init__(self):
        self.lidarReading = LaserScan()
        self.allreading = [0.0]*360
        self.segments = [0.0]*12 # 360 to 12 segments
        self.temp = rospy.Subscriber("arta/merged_scan", LaserScan, self.callback)
        rospy.init_node('curr_handle_lidarReading_server')
        self.s = rospy.Service('zswcc/curr_lidarReading_pub', lidarReading, self.handle_lidarReading)
        print("Ready to provide current point")
        rospy.spin()

    def callback(self,data=None):
        #global my_data
        #print(data.pose.pose)
        self.lidarReading = data
        #my_data = data.pose.pose
        #pub.publish(data.pose.pose)
        #return data.pose.pose

    def reading2DirWarning(self,readingTuple):
        

        #readingList[np.isnan(readingList)] = 0
        #readingList[np.isinf(readingList)] = 15

        # neglected the lidar data from scanning of SWC's hardware instead of the surrounding
        
        #readingList = map(lambda x: x+1 if x < 0.35 else x, readingList)
        #[x+1 if x < 0.35 else x for x in readingList]

        #print(type(readingTuple))
        readingList = list(readingTuple)
        #print(type(readingList[0]))
        #print("readingList[90:280] is {}".format(readingList[90:280]))

        # when the footstep is down
        #readingList[90:135] = list(map(lambda x: x+1 if x < 0.35 else x, readingList[90:135])) # 0.35 is the error from the SWC hardware
        #readingList[250:280] = list(map(lambda x: x+1 if x < 0.35 else x, readingList[250:280]))
        # when the footstep is up
        readingList[60:280] = list(map(lambda x: x+1 if x < 0.39 else x, readingList[90:280]))
        self.allreading = readingList

        #print("readingList[90:280] is {}".format(readingList[90:280]))
        #[(x+1) if x < 0.35 else x for x in readingList] # this is not working
        #print(readingList)
        #print("***********************************")
        readingListIndex = list(range(0,13,1))
        readingListIndex = [x*30 for x in readingListIndex]
        # convert degrees to segments
        """self.segments[0] = min(readingList[readingListIndex[0]:readingListIndex[1]+5])
        self.segments[1] = min(readingList[(readingListIndex[1]-3):(readingListIndex[2]+3)])
        self.segments[2] = min(readingList[(readingListIndex[2]-3):(readingListIndex[3]+3)])
        self.segments[3] = min(readingList[(readingListIndex[3]-3):(readingListIndex[4]+3)])
        self.segments[4] = min(readingList[(readingListIndex[4]-3):(readingListIndex[5]+3)])
        self.segments[5] = min(readingList[(readingListIndex[5]-3):(readingListIndex[6]+3)])
        self.segments[6] = min(readingList[(readingListIndex[6]-3):(readingListIndex[7]+3)])
        self.segments[7] = min(readingList[(readingListIndex[7]-3):(readingListIndex[8]+3)])
        self.segments[8] = min(readingList[(readingListIndex[8]-3):(readingListIndex[9]+3)])
        self.segments[9] = min(readingList[(readingListIndex[9]-3):(readingListIndex[10]+3)])
        self.segments[10] = min(readingList[(readingListIndex[10]-3):(readingListIndex[11]+3)])
        self.segments[11] = min(readingList[(readingListIndex[11]-5):(readingListIndex[12])])"""

        self.segments[0] = sum(sorted(readingList[readingListIndex[0]:readingListIndex[1]+5])[0:3])/3
        self.segments[1] = sum(sorted(readingList[(readingListIndex[1]-3):(readingListIndex[2]+3)])[0:3])/3
        self.segments[2] = sum(sorted(readingList[(readingListIndex[2]-3):(readingListIndex[3]+3)])[0:3])/3
        self.segments[3] = sum(sorted(readingList[(readingListIndex[3]-3):(readingListIndex[4]+3)])[0:3])/3
        self.segments[4] = sum(sorted(readingList[(readingListIndex[4]-3):(readingListIndex[5]+3)])[0:3])/3
        self.segments[5] = sum(sorted(readingList[(readingListIndex[5]-3):(readingListIndex[6]+3)])[0:3])/3
        self.segments[6] = sum(sorted(readingList[(readingListIndex[6]-3):(readingListIndex[7]+3)])[0:3])/3
        self.segments[7] = sum(sorted(readingList[(readingListIndex[7]-3):(readingListIndex[8]+3)])[0:3])/3
        self.segments[8] = sum(sorted(readingList[(readingListIndex[8]-3):(readingListIndex[9]+3)])[0:3])/3
        self.segments[9] = sum(sorted(readingList[(readingListIndex[9]-3):(readingListIndex[10]+3)])[0:3])/3
        self.segments[10] = sum(sorted(readingList[(readingListIndex[10]-3):(readingListIndex[11]+3)])[0:3])/3
        self.segments[11] = sum(sorted(readingList[(readingListIndex[11]-5):(readingListIndex[12])])[0:3])/3

        #sum(sorted([4,6,2,3,9,8,12])[0:3])/3


        
    def handle_lidarReading(self,req=None):
        #rospy.sleep(1) # Sleeps for 1 sec
        if req.request == True:
            #print("Returning Lidar Reading")
            #print("self.lidarReading.ranges[0] is {}".format(self.lidarReading.ranges))
            #print("-------------------------------------")
            self.reading2DirWarning(self.lidarReading.ranges)
            #print("self.segments[0] is {}".format(self.segments[0]))
            return lidarReadingResponse(self.allreading,self.segments)
        

if __name__ == "__main__":
    
    #pub = rospy.Publisher('Here_is_the_location', Pose, queue_size=10)
    pubLoc = publishLoc()
