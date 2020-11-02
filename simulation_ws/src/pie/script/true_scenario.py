#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
#from tf.transformations import euler_from_quaternion, quaternion_from_euler
import time
import threading
import pandas as pd
import numpy as np
import os
from datetime import datetime
from std_msgs.msg import String
import rosgraph


class true_scenario():
    def __init__(self,uavno):
        self.uavno = uavno
        self.current_state = None
        self.currentVelocity = None
        rospy.Subscriber('/uav'+str(self.uavno)+'/mavros/local_position/pose', PoseStamped, self.local_position_cb)
        rospy.Subscriber('/uav'+str(self.uavno)+'/mavros/state', State, self.state_cb)
        self.dataLabel_pub =  rospy.Publisher('/uav'+str(self.uavno)+'/data_label', String, queue_size=10)
        # rospy.Subscriber('/uav'+str(self.uavno)+'/mavros/local_position/velocity_local', TwistStamped, self.local_velocity_cb)
        # rospy.Subscriber('/uav'+str(self.uavno)+'/data_label', String, self.label_cb)
        # self.currentPosition = None
        # self.currentVelocity = None
        # self.currentLabel = "Hold"
        # self.stopThread = False

    def state_cb(self,msg):
        self.current_state = msg

    def local_position_cb(self,msg):
        if None is not self.current_state:
                if self.current_state.armed == False and self.current_state.mode == 'AUTO.LAND':
                    self.dataLabel_pub.publish('Hold')
                    rospy.sleep(10)
                    self.dataLabel_pub.publish('Finished')
                    return

        # if abs(msg.pose.position.x-17)<5 and abs(msg.pose.position.y-0)<5 and abs(msg.pose.position.z-5)<9:
        #     self.dataLabel_pub.publish('Obstacleavoid')
        # if abs(msg.pose.position.x-5)<3.5 and abs(msg.pose.position.y-12.5)<3.5 and abs(msg.pose.position.z-5)<9:
        #     self.dataLabel_pub.publish('Loiter')
        # elif abs(msg.pose.position.x-25)<3.5 and abs(msg.pose.position.y-12.5)<3.5 and abs(msg.pose.position.z-5)<9:
        #     self.dataLabel_pub.publish('Loiter')
        # elif abs(msg.pose.position.x-45)<3.5 and abs(msg.pose.position.y-12.5)<3.5 and abs(msg.pose.position.z-5)<9:
        #     self.dataLabel_pub.publish('Loiter')
        if abs(msg.pose.position.x-0)<1 and abs(msg.pose.position.y-0)<1 and abs(msg.pose.position.z-0)<.5:
            self.dataLabel_pub.publish('Hold')
        elif abs(msg.pose.position.x-50)<1 and abs(msg.pose.position.y-50)<1 and abs(msg.pose.position.z-0)<.5:
            self.dataLabel_pub.publish('Hold')
        elif abs(msg.pose.position.x-0)<1 and abs(msg.pose.position.y-0)<1 and abs(msg.pose.position.z-3)<.5:
            self.dataLabel_pub.publish('Hover')
        elif abs(msg.pose.position.x-0)<1 and abs(msg.pose.position.y-0)<1 and abs(msg.pose.position.z-1.5)<1:
            if self.currentVelocity is not None:
                # if self.currentVelocity.linear.z>0:
                self.dataLabel_pub.publish('Takeoff')
                # else:
                #     self.dataLabel_pub.publish('Land')
        elif abs(msg.pose.position.x-50)<1 and abs(msg.pose.position.y-50)<1 and abs(msg.pose.position.z-1.5)<1:
            self.dataLabel_pub.publish('Land')
        else:
            self.dataLabel_pub.publish('Search')


    def local_velocity_cb(self,msg):
        self.currentVelocity = msg.twist

    


if __name__ == '__main__':

    rospy.init_node('true_scenario_uav', anonymous=True)
    uav_num = rospy.get_param('~uav_num')
    true_sce = true_scenario(uav_num)
    rate = rospy.Rate(20.0)
    
    while rosgraph.is_master_online():
        rate.sleep()


    
