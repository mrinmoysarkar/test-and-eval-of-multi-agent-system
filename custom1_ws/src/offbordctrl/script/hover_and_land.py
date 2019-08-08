#!/usr/bin/env python

from __future__ import print_function

import rospy
from std_msgs.msg import String
import numpy as np
#import cv2

import time
import os
import sys
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool, ParamGet, SetMode, CommandTOL

import time


current_state = None
current_alt = 0

def state_cb(msg):
	global current_state
	current_state = msg
	#print(current_state)


def set_arm(arm, timeout):
    loop_freq = 1  # Hz
    rate = rospy.Rate(loop_freq)
    rospy.wait_for_service('/custom1/mavros/cmd/arming')
    arming_srv = rospy.ServiceProxy('/custom1/mavros/cmd/arming', CommandBool)
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
	global current_alt
	print("current alt: ", current_alt)
	loop_freq = 1  # Hz
	rate = rospy.Rate(loop_freq)
	rospy.wait_for_service('/custom1/mavros/cmd/land')
	land_set = rospy.ServiceProxy('/custom1/mavros/cmd/land', CommandTOL)
	land_cmd = [0,0,0,0,0]#CommandTOL()
	# land_cmd.min_pitch = 0.0
	# land_cmd.yaw = 0.0
	# land_cmd.latitude = 0.0#float('nan')
	# land_cmd.longitude = 0.0#float('nan')
	# land_cmd.altitude = 0
	
	for i in xrange(timeout * loop_freq):
	    if current_state.armed == False:
	        return True
	    else:
	        try:
	            res = land_set(min_pitch=0.0,
                         yaw=0.0,
                         latitude=float('nan'),
                         longitude=float('nan'),
                         altitude=current_alt)  # 0 is custom mode
	            # res = land_set()
	            print(res)
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
    rospy.wait_for_service('/custom1/mavros/set_mode')
    mode_set = rospy.ServiceProxy('/custom1/mavros/set_mode', SetMode)
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

if __name__ == '__main__':

	rospy.init_node('custom1_hover_and_land_scenario', anonymous=True)
	rospy.Subscriber('/custom1/mavros/state', State, state_cb)
	rospy.Subscriber('/custom1/mavros/local_position/pose',PoseStamped,local_position_cb)

	setpointPub = rospy.Publisher('/custom1/mavros/setpoint_position/local',PoseStamped,queue_size=100)
	setpoint = PoseStamped()
	setpoint.pose.position.x = 0.9
	setpoint.pose.position.y = 1.7
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
#		setpoint.pose.position.x = 0.9
#		setpoint.pose.position.y = 1.7
#		setpoint.pose.position.z = 0.5
#	
#		init_time = time.time()
#		while not rospy.is_shutdown():
#		    setpointPub.publish(setpoint);
#		    rate.sleep()
#		    if (time.time()-init_time) > 5:
#		    	break
#		# set_mode('AUTO.LAND', 5)
		land(5)
		rospy.sleep(5)
		set_arm(False, 5)