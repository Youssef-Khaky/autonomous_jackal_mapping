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



def callback(data):
    sensor.update(data)
    correction()

def listener():
    rospy.Subscriber("/front/scan", LaserScan, callback,queue_size =1)
    rospy.spin()

def correction():
    LOI = sensor.getLOI()
    print("LOI is", LOI, "dist is", sensor.DistOfPoint(120),"front clearance is",sensor.DistOfPoint(360))

    if sensor.aheadLessThanThresh():
        Jackal.rotate_by(0.5)

    elif sensor.lookAtWall() and sensor.close_to_wall():
        Jackal.rotate_by(0.5)

    elif LOI >= 140 and sensor.DistOfPoint(120)>1:
        Jackal.move_forward(0.5)

    elif LOI <= 100 and sensor.DistOfPoint(120)>1:
        Jackal.rotate_by(-0.5)

    elif LOI <= 100 and sensor.DistOfPoint(120)<0.5:
        Jackal.move_forward(0.5)

    elif sensor.DistOfPoint(120)<0.5:
        Jackal.rotate_by(0.5)

    elif sensor.DistOfPoint(120)>1:
        Jackal.rotate_by(-0.5)

    else:
        Jackal.move_forward(0.5)



rospy.init_node('move_Jackal_test', anonymous=True)
Jackal = MoveJackal()
sensor = Lidar()
listener()

