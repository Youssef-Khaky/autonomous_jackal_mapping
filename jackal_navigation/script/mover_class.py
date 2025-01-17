#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import matplotlib.pyplot as plt
import time

class MoveJackal():
    
    def __init__(self):
        self.node_terminated = False
        self.Jackal_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.on_shutdown(self.shutdownhook)
        self.rate = rospy.Rate(10) # 10hz
    
    def move_forward(self,speed):
        while not self.node_terminated:
            connections = self.Jackal_vel_publisher.get_num_connections()
            #print("number of connections are ",connections)
            if connections > 0:
                movement_msg = Twist()
                movement_msg.linear.x = speed
                self.Jackal_vel_publisher.publish(movement_msg)
                break
            else:
                self.rate.sleep()

    def stop(self):
        while not self.node_terminated:
            movement_msg = Twist()
            movement_msg.angular.z = 0
            movement_msg.linear.x = 0
            self.Jackal_vel_publisher.publish(movement_msg)
            break

    
    def rotate_by(self,ang_speed):
        while not self.node_terminated:
            connections = self.Jackal_vel_publisher.get_num_connections()
            #print("number of connections are ",connections)
            if connections > 0:
                movement_msg = Twist()
                movement_msg.angular.z = ang_speed
                self.Jackal_vel_publisher.publish(movement_msg)
                break
            else:
                self.rate.sleep()
        
    def move_fwd_duration(self, dur, speed):
        
        while not self.node_terminated:
            connections = self.Jackal_vel_publisher.get_num_connections()
            #print("number of connections are ",connections)
            if connections > 0:
                self.move_forward(speed)
                break
            else:
                self.rate.sleep()
        rospy.loginfo("Moving Forwards")
        time.sleep(dur)
        self.stop()
        rospy.loginfo("######## Finished Moving Forward")
        
    def mrot_ccw_duration(self, dur, speed):
        
        while not self.node_terminated:
            connections = self.Jackal_vel_publisher.get_num_connections()
            #print("number of connections are ",connections)
            if connections > 0:
                self.rotate_by(speed)
                break
            else:
                self.rate.sleep()
        rospy.loginfo("rotating")
        time.sleep(dur)
        self.stop()
        rospy.loginfo("######## Finished rotating ccw")
        

            
    def shutdownhook(Self):
        self.stop()
        self.node_terminated = True
        
    
if __name__ == '__main__':
    rospy.init_node('move_Jackal_test', anonymous=True)
    moveJackal_object = MoveJackal()
    

        
        
