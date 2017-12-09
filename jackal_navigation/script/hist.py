#!/usr/bin/env python

import rospy
import matplotlib.pyplot as plt
from sensor_msgs.msg import LaserScan
import time

def callback(data):
    global pub
    
    log = [0]*len(data.ranges)
    l = len(log)
    log[l/4:l*3/4] = data.ranges[l/4:l*3/4]

    new_data = data
    new_data.ranges = log
    _ = plt.hist(log,bins=[0.1,1,2,3,4,5,6,7,8,9, 10])
    pub.publish(new_data)

    _ = plt.xlabel('number of raser hits')
    _ = plt.ylabel('distance')

    plt.draw()

    plt.pause(0.0001)
    plt.cla()

    
def listener():
    

    rospy.init_node('listener', anonymous=True)

    plt.ion()
    #_ = plt.hist(distances)

    #_ = plt.xlabel('number of raser hits')
    #_ = plt.ylabel('distance')
  
    #plt.show(block=False)
    
    #time.sleep(0.5) 

    
    rospy.Subscriber("/front/scan", LaserScan, callback,queue_size =1)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    pub = rospy.Publisher('front_obs', LaserScan, queue_size=1)
    listener()

