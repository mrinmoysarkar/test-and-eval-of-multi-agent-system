#!/usr/bin/env python

from __future__ import print_function

import rospy
from std_msgs.msg import String
import numpy as np
import cv2

import time
import os
import sys
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool, ParamGet, SetMode, CommandTOL
from math import radians, sin, cos
import time
import threading



class uavControl():
    def __init__(self,uavno):
        self.current_state = None
        self.setpointPub = None
        self.stopThread = False
        self.uavno = uavno
        self.currentPosition = None

    def state_cb(self,msg):
        self.current_state = msg

    def set_arm(self, arm, timeout):
        loop_freq = 1  # Hz
        rate = rospy.Rate(loop_freq)
        rospy.wait_for_service('/uav'+str(self.uavno)+'/mavros/cmd/arming')
        arming_srv = rospy.ServiceProxy('/uav'+str(self.uavno)+'/mavros/cmd/arming', CommandBool)
        for i in xrange(timeout * loop_freq):
            if self.current_state.armed == arm:
                return True
            else:
                try:
                    res = arming_srv(arm)
                    if not res.success:
                        rospy.logerr("failed to send arm command")
                    else:
                    	return True
                except rospy.ServiceException as e:
                    rospy.logerr(e)
            try:
                rate.sleep()
            except rospy.ROSException as e:
                rospy.logerr(e)
    	return False            

    def land(self,timeout):
    	loop_freq = 1  # Hz
    	rate = rospy.Rate(loop_freq)
    	rospy.wait_for_service('/uav'+str(self.uavno)+'/mavros/cmd/land')
    	land_set = rospy.ServiceProxy('/uav'+str(self.uavno)+'/mavros/cmd/land', CommandTOL)
    	
    	
    	for i in xrange(timeout * loop_freq):
    	    if self.current_state.armed == False:
    	        return True
    	    else:
    	        try:
    	            res = land_set(min_pitch=0.0,
                             yaw=0.0,
                             latitude=0.0,
                             longitude=0.0,
                             altitude=0.0) 
    	            if not res.success:
    	                rospy.logerr("failed to send land command")
    	            else:
    	            	return True
    	        except rospy.ServiceException as e:
    	            rospy.logerr(e)

    	    try:
    	        rate.sleep()
    	    except rospy.ROSException as e:
    	        print(e)

    	return False    

    def set_mode(self, mode, timeout):
        loop_freq = 1  # Hz
        rate = rospy.Rate(loop_freq)
        rospy.wait_for_service('/uav'+str(self.uavno)+'/mavros/set_mode')
        mode_set = rospy.ServiceProxy('/uav'+str(self.uavno)+'/mavros/set_mode', SetMode)
        for i in xrange(timeout * loop_freq):
            if self.current_state.mode == mode:
                return True
            else:
                try:
                    res = mode_set(0,mode)  # 0 is custom mode
                    if not res.mode_sent:
                        rospy.logerr("failed to send mode command")
                    else:
                    	return True
                except rospy.ServiceException as e:
                    rospy.logerr(e)

            try:
                rate.sleep()
            except rospy.ROSException as e:
                self.fail(e)

        return False

    def local_position_cb(self,msg):
    	self.currentPosition = msg.pose

    def getquaternion(self, roll, pitch, yaw):
        # return tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        roll = radians(roll)
        pitch = radians(pitch)
        yaw = radians(yaw)
        cy = cos(yaw * 0.5)
        sy = sin(yaw * 0.5)
        cp = cos(pitch * 0.5)
        sp = sin(pitch * 0.5)
        cr = cos(roll * 0.5)
        sr = sin(roll * 0.5)

        q=[0,0,0,0]
        q[0] = -1*(cy * cp * sr - sy * sp * cr) #x
        q[1] = -1*(sy * cp * sr + cy * sp * cr) #y
        q[2] = -1*(sy * cp * cr - cy * sp * sr) #z
        q[3] = -1*(cy * cp * cr + sy * sp * sr) #w
        
        return q

    def loop(self):
        rospy.Subscriber('/uav'+str(self.uavno)+'/mavros/state', State, self.state_cb)
        rospy.Subscriber('/uav'+str(self.uavno)+'/mavros/local_position/pose', PoseStamped, self.local_position_cb)

        self.setpointPub = rospy.Publisher('/uav'+str(self.uavno)+'/mavros/setpoint_position/local', PoseStamped, queue_size=100)

        rospy.sleep(5)
        rate = rospy.Rate(20.0)
        

        setpoint = PoseStamped()
        setpoint.pose.position.x = self.currentPosition.position.x
        setpoint.pose.position.y = self.currentPosition.position.y
        setpoint.pose.position.z = 1.5
        for i in range(100):
            self.setpointPub.publish(setpoint)
            rate.sleep()

        q = self.getquaternion(0,0,0)
        setpoint.pose.position.x = self.currentPosition.position.x
        setpoint.pose.position.y = self.currentPosition.position.y
        setpoint.pose.position.z = 1.5
        setpoint.pose.orientation.x = q[0]
        setpoint.pose.orientation.y = q[1]
        setpoint.pose.orientation.z = q[2]
        setpoint.pose.orientation.w = q[3]

        if self.set_mode('OFFBOARD', 5) and self.set_arm(True, 5):

            while not rospy.is_shutdown() and not self.stopThread:
                self.setpointPub.publish(setpoint)
                rate.sleep()

            self.land(5)
            rospy.sleep(10)
            self.set_arm(False, 5)
            print("landed safely :).")


