#!/usr/bin/env python

import mover_class
import rospy
from geometry_msgs.msg import Twist
import matplotlib.pyplot as plt
import time
from mover_class import MoveJackal
from sensor_msgs.msg import LaserScan


class Explorer:
    def __init__(self):
        rospy.init_node('explorer', anonymous=True)
        rospy.Subscriber("left_obs", LaserScan, callback,queue_size =1)
        self.moverObject = MoveJackal()
        self.node_terminated = False
        rospy.on_shutdown(self.shutdownhook)
        self.rate = rospy.Rate(10) # 10hz

    def 
    
