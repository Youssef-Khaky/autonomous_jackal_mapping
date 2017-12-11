#!/usr/bin/env python

import rospy
import matplotlib.pyplot as plt
from sensor_msgs.msg import LaserScan
import time
from mover_class import MoveJackal
import numpy as np
from geometry_msgs.msg import Twist
import copy
import matplotlib.patches as mpatches
from lidar import Lidar



def callback(data,args):
    la = data.ranges
    pub = args
    sensor.update(data)
    simplified_output = [0]*len(data.ranges)
    l = len(simplified_output)
    simplified_output[120] = data.ranges[120]
    LOI = sensor.getLOI()

    simplified_output[LOI] = data.ranges[sensor.getLOI()]

#    new_data = copy.copy(data)
#    new_data.ranges = simplified_output
    data.ranges = simplified_output

#    pub.publish(new_data)
    pub.publish(data)


#    correction(Jackal,data.ranges)
    correction()
#    _ = plt.hist(la,bins=[0.1,1,2,3,4,5,6,7,8,9,10])
    _ = plt.plot(la)
    plt.axhline(y=sensor.getMinDest())
    plt.axvline(x=LOI)
    _ = plt.xlabel('number of raser hits')
    _ = plt.ylabel(str(LOI))
    plt.ylim( 0, 5 ) 
    plt.xlim( 50, 350 ) 
    plt.draw()
    plt.pause(0.0001)
    plt.cla()

def listener():
    plt.ion()

    rospy.Subscriber("/front/scan", LaserScan, callback,(pub),queue_size =1)
    rospy.spin()

def getLOI(arr):
    npa = np.asarray(arr, dtype=np.float64)
    np.ones_like(npa)
    npa[300:] = 999.9 #we care about the right half a.k.a the first half
    npa[0:80] = 999.9
    #print(npa)
    LOI = np.argmin(npa)
    return LOI
def correction():
    region = sensor.check_region()
    tilt = sensor.check_tilt()

    if (region == "in_bound" and tilt == "parallel") or (region == "far" and tilt == "con") or (region == "close" and tilt == "div"):
        print(region, tilt,"move_forward")
    elif (region == "close") or ( region == "in_bound" and tilt == "con"):
        print(region, tilt,"rotate_by(0.5)")
    elif region == "far" or ( region == "in_bound" and tilt == "div"):
        print(region, tilt,"rotate_by(-0.5)")




rospy.init_node('move_Jackal_test', anonymous=True)
#pub = rospy.Publisher('left_obs', LaserScan, queue_size=1)
pub = rospy.Publisher('left_obs', LaserScan, queue_size=1)
Jackal = MoveJackal()
sensor = Lidar()
listener()
