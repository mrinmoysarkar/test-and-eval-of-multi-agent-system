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

import time


current_state = None

def state_cb(msg):
	global current_state
	current_state = msg
	#print(current_state)


def set_arm(arm, timeout):
    loop_freq = 1  # Hz
    rate = rospy.Rate(loop_freq)
    rospy.wait_for_service('mavros/cmd/arming')
    arming_srv = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
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

# def land(timeout):
# 	loop_freq = 1  # Hz
#     rate = rospy.Rate(loop_freq)
#     rospy.wait_for_service('mavros/set_mode')
#     land_set = rospy.ServiceProxy('mavros/cmd/land', SetMode)
#     for i in xrange(timeout * loop_freq):
#         if current_state.mode == mode:
#             return True
#         else:
#             try:
#                 res = land_set(mode)  # 0 is custom mode
#                 if not res.success:
#                     rospy.logerr("failed to send land command")
#                 else:
#                 	return True
#             except rospy.ServiceException as e:
#                 rospy.logerr(e)

#         try:
#             rate.sleep()
#         except rospy.ROSException as e:
#             self.fail(e)

#     return False
    

def set_mode(mode, timeout):
    loop_freq = 1  # Hz
    rate = rospy.Rate(loop_freq)
    rospy.wait_for_service('mavros/set_mode')
    mode_set = rospy.ServiceProxy('mavros/set_mode', SetMode)
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


if __name__ == '__main__':

	rospy.init_node('vertical_transition_scenario', anonymous=True)
	rospy.Subscriber('/mavros/state', State, state_cb)

	setpointPub = rospy.Publisher('mavros/setpoint_position/local',PoseStamped,queue_size=100)
	setpoint = PoseStamped()
	setpoint.pose.position.x = 0
	setpoint.pose.position.y = 0
	setpoint.pose.position.z = 1.5

	rate = rospy.Rate(20.0)
	for i in range(100):
	    setpointPub.publish(setpoint);
	    rate.sleep()
	if set_mode('OFFBOARD', 5) and set_arm(True, 5):
		init_time = time.time()
		while not rospy.is_shutdown():
		    setpointPub.publish(setpoint);
		    rate.sleep()
		    if (time.time()-init_time) > 5:
		    	break
# taking off vertically done
##########################
		x_values = np.linspace(0,1.5,50)
		z_values = np.linspace(1.5,2,50)
		for (x,z) in zip(x_values,z_values):
			setpoint.pose.position.x = x
			setpoint.pose.position.y = 0
			setpoint.pose.position.z = z
			setpointPub.publish(setpoint);
			rate.sleep()

		init_time = time.time()
		while not rospy.is_shutdown():
		    setpointPub.publish(setpoint);
		    rate.sleep()
		    if (time.time()-init_time) > 5:
		    	break
#############################
# going to right with altitude gain done
#############################
		y_values = np.linspace(0,1.5,50)
		for y in y_values:
			setpoint.pose.position.x = 1.5
			setpoint.pose.position.y = y
			setpoint.pose.position.z = 2.0
			setpointPub.publish(setpoint);
			rate.sleep()

		init_time = time.time()
		while not rospy.is_shutdown():
		    setpointPub.publish(setpoint);
		    rate.sleep()
		    if (time.time()-init_time) > 5:
		    	break
###############################
# going forward with constant altitude done
#############################
		x_values = np.linspace(1.5,0,50)
		for x in x_values:
			setpoint.pose.position.x = x
			setpoint.pose.position.y = 1.5
			setpoint.pose.position.z = 2.0
			setpointPub.publish(setpoint);
			rate.sleep()

		init_time = time.time()
		while not rospy.is_shutdown():
		    setpointPub.publish(setpoint);
		    rate.sleep()
		    if (time.time()-init_time) > 5:
		    	break
###############################
# going left with constant altitude done
##########################
		y_values = np.linspace(1.5,3,50)
		z_values = np.linspace(2,1.5,50)
		for (y,z) in zip(y_values,z_values):
			setpoint.pose.position.x = 0
			setpoint.pose.position.y = y
			setpoint.pose.position.z = z
			setpointPub.publish(setpoint);
			rate.sleep()

		init_time = time.time()
		while not rospy.is_shutdown():
		    setpointPub.publish(setpoint);
		    rate.sleep()
		    if (time.time()-init_time) > 5:
		    	break
#############################
# going forward with altitude lose done
#############################
		x_values = np.linspace(0,1.5,50)
		for x in x_values:
			setpoint.pose.position.x = x
			setpoint.pose.position.y = 3.0
			setpoint.pose.position.z = 1.5
			setpointPub.publish(setpoint);
			rate.sleep()

		init_time = time.time()
		while not rospy.is_shutdown():
		    setpointPub.publish(setpoint);
		    rate.sleep()
		    if (time.time()-init_time) > 5:
		    	break
###############################
# going right with constant altitude done
##########################
		x_values = np.linspace(1.5,0.5,50)
		y_values = np.linspace(3,0.5,50)
		for (x,y) in zip(x_values,y_values):
			setpoint.pose.position.x = x
			setpoint.pose.position.y = y
			setpoint.pose.position.z = 1.5
			setpointPub.publish(setpoint);
			rate.sleep()

		init_time = time.time()
		while not rospy.is_shutdown():
		    setpointPub.publish(setpoint);
		    rate.sleep()
		    if (time.time()-init_time) > 5:
		    	break
#############################
# coming back diagonally with constant altitude done
##########################
		x_values = np.linspace(0.5,0,50)
		y_values = np.linspace(0.5,0,50)
		z_values = np.linspace(1.5,0.1,50)
		for (x,y,z) in zip(x_values,y_values,z_values):
			setpoint.pose.position.x = x
			setpoint.pose.position.y = y
			setpoint.pose.position.z = z
			setpointPub.publish(setpoint)
			rate.sleep()

		init_time = time.time()
		while not rospy.is_shutdown():
		    setpointPub.publish(setpoint);
		    rate.sleep()
		    if (time.time()-init_time) > 5:
		    	break
#############################
# landing with deviation

		setpoint.pose.position.x = 0
		setpoint.pose.position.y = 0
		setpoint.pose.position.z = 0.05
    	init_time = time.time()
    	while not rospy.is_shutdown():
		    setpointPub.publish(setpoint);
		    rate.sleep()
		    if (time.time()-init_time) > 5:
		    	set_arm(False, 5)
		    	break

	