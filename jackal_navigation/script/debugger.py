#!/usr/bin/env python

import rospy
import matplotlib.pyplot as plt
from sensor_msgs.msg import LaserScan
import time
from mover_class import MoveJackal
import numpy as np
from geometry_msgs.msg import Twist
import copy

from lidar import Lidar



def callback(data,args):
    
    pub = args
    sensor.update(data)
    simplified_output = [0]*len(data.ranges)
    l = len(simplified_output)
    simplified_output[120] = data.ranges[120]
    LOI = sensor.getLOI()

    simplified_output[LOI] = data.ranges[sensor.getLOI()]

    data.ranges = simplified_output

    pub.publish(data)

    correction()
    datapoints = data.ranges
    _ = plt.plot(datapoints)
    plt.axhline(y=sensor.getMinDest())
    _ = plt.xlabel('number of raser hits')
    _ = plt.ylabel('distance')
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
#    LOI = getLOI(arr)
    LOI = sensor.getLOI()
    print("LOI is", LOI, "dist is", sensor.DistOfPoint(120),"front clearance is",sensor.DistOfPoint(360))

    if sensor.aheadLessThanThresh():
        print("LOI is", LOI, "dist is", sensor.DistOfPoint(120),"front clearance is",sensor.DistOfPoint(360),"rotate ccw")
        Jackal.rotate_by(0.5)

    elif sensor.lookAtWall() and sensor.close_to_wall():
        print("LOI is", LOI, "dist is", sensor.DistOfPoint(120),"front clearance is",sensor.DistOfPoint(360),"rotate ccw")
        Jackal.rotate_by(0.5)

    elif LOI >= 140 and sensor.DistOfPoint(120)>1:
        print("LOI is", LOI, "dist is", sensor.DistOfPoint(120),"front clearance is",sensor.DistOfPoint(360),"forward")
        Jackal.move_forward(0.5)

    elif LOI <= 100 and sensor.DistOfPoint(120)>1:
        print("LOI is", LOI, "dist is", sensor.DistOfPoint(120),"front clearance is",sensor.DistOfPoint(360),"rotate cw")
        Jackal.rotate_by(-0.5)


    elif LOI <= 100 and sensor.DistOfPoint(120)<0.5:
        print("LOI is", LOI, "dist is", sensor.DistOfPoint(120),"front clearance is",sensor.DistOfPoint(360),"rotate ccw")
        Jackal.move_forward(0.5)

    elif sensor.DistOfPoint(120)<0.5:
        print("too close moving away")
        Jackal.rotate_by(0.5)

    elif sensor.DistOfPoint(120)>1:
        print("too far moving close")
        Jackal.rotate_by(-0.5)


    else:
        print("LOI is", LOI, "dist is", sensor.DistOfPoint(120),"front clearance is",sensor.DistOfPoint(360),"forward")
        Jackal.move_forward(0.5)



rospy.init_node('move_Jackal_test', anonymous=True)
#pub = rospy.Publisher('left_obs', LaserScan, queue_size=1)
pub = rospy.Publisher('left_obs', LaserScan, queue_size=1)
Jackal = MoveJackal()
sensor = Lidar()
listener()

