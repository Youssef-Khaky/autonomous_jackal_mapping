#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
import time
import numpy as np


class Lidar:

    def __init__(self):
        self.LOI = 0 #line of interest
        self.dist_readings = np.zeros(720)

        self.forward_distance_thresh = 1


        self.side_distance_thresh_upper = 1
        self.side_distance_thresh_lower = 0.5


        self.sideThreshUpper_Tilt = 140
        self.sideThreshLower_Tilt = 100


    def getLOI(self):
        return self.LOI

    def update(self,laserscan):
        self.dist_readings = laserscan.ranges
        arr = laserscan.ranges #gettings the readings
        npa = np.asarray(arr, dtype=np.float64) #npa

        #maximize the region we dont care about, we car about the right side, so readings from 80 to 300 are important, we ignore the rest
        npa[300:] = 999.9
        npa[0:80] = 999.9
        LOI = np.argmin(npa)
        self.LOI = LOI



    def aheadLessThan(self):
        return self.dist_readings[360] <self.forward_distance_thresh

    def lookAtWall(self):
        return self.LOI > self.sideThreshUpper_Tilt

    def lookingAway(self):
        return self.sideThreshLower_Tilt < self.sideThreshLower_Tilt

    def far_from_wall(self):
        return self.dist_readings[getLOI(self)] > side_distance_thresh_upper

    def close_to_wall(self):
        return self.dist_readings[getLOI(self)] < side_distance_thresh_lower



