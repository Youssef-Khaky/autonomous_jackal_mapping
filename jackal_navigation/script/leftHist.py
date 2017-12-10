#!/usr/bin/env python

import rospy
import matplotlib.pyplot as plt
from sensor_msgs.msg import LaserScan
import time
import datetime
import os
import sys
from mover_class import MoveJackal
import numpy as np
from geometry_msgs.msg import Twist
import copy
import pyscreenshot as ImageGrab
from lidar import Lidar
import matplotlib.patches as mpatches
i =0

def callback(data,args):
    global i
    la = data.ranges
    pub = args
    sensor.update(data)
    simplified_output = [0]*len(data.ranges)
    l = len(simplified_output)
    simplified_output[120] = data.ranges[120]
    LOI = sensor.getLOI()

#    simplified_output[LOI] = data.ranges[sensor.getLOI()]

#    new_data = copy.copy(data)
#    new_data.ranges = simplified_output
    new = LaserScan()
    new.ranges = simplified_output
    new.header.frame_id = 'front_laser'
    new.angle_min = -2.3561899662
    new.angle_max = 2.3561899662
    new.angle_increment = 0.0065540750511
    new.header.seq = i
    

#    pub.publish(new_data)
    pub.publish(new)
    red_patch = mpatches.Patch(color='red', label=str(LOI))
    plt.legend(handles=[red_patch])
#    correction(Jackal,data.ranges)
    correction()
#    _ = plt.hist(la,bins=[0.1,1,2,3,4,5,6,7,8,9,10])
    _ = plt.plot(la)
    im = ImageGrab.grab(bbox=(300, 300, 1920, 1000))
    plt.axhline(y=sensor.getMinDest())
    _ = plt.xlabel('number of raser hits')
    _ = plt.ylabel(str(LOI))
    plt.ylim( 0, 5 )
    
    
    home = '/home/indigo/jackal_navigation/src/jackal/jackal_navigation/script/figs'
#    currentDT = datetime.datetime.now()
#    fileName = "figs"#/"+str(currentDT) + "fig.png"
#    print(fileName)
#    plt.savefig("~/bla.png")
    pathg = os.path.join(home,  str(i)+" graph")
    pathsc = os.path.join(home,  str(i)+" sc")
    im.save(pathsc+".png")
    plt.savefig(pathg,format='png')
    i = i+1
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
    sys.stdout.write(region)
    sys.stdout.write(" ")
    sys.stdout.write(tilt)
    sys.stdout.write(" ")
    sys.stdout.write(str(sensor.LOI))
    sys.stdout.write(" ")
    sys.stdout.write(str(sensor.getMinDest()))
    sys.stdout.write(" ")
    if (region == "in_bound" and tilt == "parallel") or (region == "far" and tilt == "con") or (region == "close" and tilt == "div"):
        Jackal.move_forward(0.5)
        print("inbound and straight, going forward")
    elif (region == "close") or ( region == "in_bound" and tilt == "con"):
        Jackal.rotate_by(0.5)
        print("too close, tilting away")
    elif region == "far" or ( region == "in_bound" and tilt == "div"):
        Jackal.rotate_by(-0.5)
        print("too far, tilting towards")
#    time.sleep(0.3)
rospy.init_node('move_Jackal_test', anonymous=True)
#pub = rospy.Publisher('left_obs', LaserScan, queue_size=1)
pub = rospy.Publisher('left_obs', LaserScan, queue_size=1,latch=True)
Jackal = MoveJackal()
sensor = Lidar()
listener()

