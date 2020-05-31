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


current_state = None
current_alt = 0
setpointPub = None
stopThread = False

def state_cb(msg):
	global current_state
	current_state = msg
	#print(current_state)

def set_arm(arm, timeout):
    loop_freq = 1  # Hz
    rate = rospy.Rate(loop_freq)
    rospy.wait_for_service('/uav1/mavros/cmd/arming')
    arming_srv = rospy.ServiceProxy('/uav1/mavros/cmd/arming', CommandBool)
    for i in xrange(timeout * loop_freq):
        if current_state.armed == arm:
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

def land(timeout):
	loop_freq = 1  # Hz
	rate = rospy.Rate(loop_freq)
	rospy.wait_for_service('/uav1/mavros/cmd/land')
	land_set = rospy.ServiceProxy('/uav1/mavros/cmd/land', CommandTOL)
	
	
	for i in xrange(timeout * loop_freq):
	    if current_state.armed == False:
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

def set_mode(mode, timeout):
    loop_freq = 1  # Hz
    rate = rospy.Rate(loop_freq)
    rospy.wait_for_service('/uav1/mavros/set_mode')
    mode_set = rospy.ServiceProxy('/uav1/mavros/set_mode', SetMode)
    for i in xrange(timeout * loop_freq):
        if current_state.mode == mode:
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

def local_position_cb(msg):
	global current_alt
	current_alt = msg.pose.position.z

def getquaternion(roll, pitch, yaw):
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

def loop():
	global setpointPub, stopThread
	rate = rospy.Rate(20.0)
	q = getquaternion(0,0,90)
	print(q)
	setpoint = PoseStamped()
	setpoint.pose.position.x = 0.9
	setpoint.pose.position.y = 1.7
	setpoint.pose.position.z = 1.5
	setpoint.pose.orientation.x = q[0]
	setpoint.pose.orientation.y = q[1]
	setpoint.pose.orientation.z = q[2]
	setpoint.pose.orientation.w = q[3]


	while not rospy.is_shutdown() and not stopThread:
		setpointPub.publish(setpoint)
		rate.sleep()

	land(5)
	rospy.sleep(10)
	set_arm(False, 5)
	print("landed safely :).")

if __name__ == '__main__':

	rospy.init_node('hover_and_land_scenario', anonymous=True)
	rospy.Subscriber('/uav1/mavros/state', State, state_cb)
	rospy.Subscriber('/uav1/mavros/local_position/pose',PoseStamped,local_position_cb)

	setpointPub = rospy.Publisher('/uav1/mavros/setpoint_position/local',PoseStamped,queue_size=100)
	setpoint = PoseStamped()
	setpoint.pose.position.x = 0.9
	setpoint.pose.position.y = 1.7
	setpoint.pose.position.z = 1.5

	rate = rospy.Rate(20.0)
	for i in range(100):
	    setpointPub.publish(setpoint);
	    rate.sleep()

	if set_mode('OFFBOARD', 5) and set_arm(True, 5):
		threading.Thread(target=loop).start()
        rate = rospy.Rate(5.0)
        while not rospy.is_shutdown():
            rate.sleep()
            cmd = raw_input("command: \"l\" for land and then \"q\" for exit the main loop: ")
            if cmd=='l':
                stopThread = True
            elif cmd=='q':
                break