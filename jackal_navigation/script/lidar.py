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

    def getMinDest(self):
        return self.DistOfPoint(self.getLOI())

    def DistOfPoint(self,point):
        return self.dist_readings[point]

    def update(self,laserscan):
        self.dist_readings = laserscan.ranges
        arr = laserscan.ranges #gettings the readings
        npa = np.asarray(arr, dtype=np.float64) #npa

        #maximize the region we dont care about, we car about the right side, so readings from 80 to 300 are important, we ignore the rest
        npa[300:] = 999.9
        npa[0:80] = 999.9
        self.LOI = np.argmin(npa)


    def check_region(self):
        min_dist = self.getMinDest()
        if min_dist > self.side_distance_thresh_upper:
            return "far"
        elif min_dist < self.side_distance_thresh_lower:
            return "close"
        else:
            return "in_bound"

    def check_tilt(self):
        LOI = self.getLOI()
        if LOI > self.sideThreshUpper_Tilt:
            return "con"
        elif LOI < self.sideThreshLower_Tilt:
            return "div"
        else:
            return "parallel"

    def lookAtWall(self):
        return self.LOI > self.sideThreshUpper_Tilt

    def lookingAway(self):
        return self.LOI < self.sideThreshLower_Tilt

    def far_from_wall(self):
        return self.dist_readings[self.getLOI()] > self.side_distance_thresh_upper

    def close_to_wall(self):
        return self.dist_readings[self.getLOI()] < self.side_distance_thresh_lower



