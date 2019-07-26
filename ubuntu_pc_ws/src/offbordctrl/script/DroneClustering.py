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
from std_msgs.msg import Float32MultiArray
import time
import threading


current_state = None
current_alt = 0 
x = 0 
y = 0
z = 1.5
cur_x = 0
cur_y = 0
update_pos = False
setpointPub = None
stopThread = False


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

def land(timeout):
	global current_alt
	print("current alt: ", current_alt)
	loop_freq = 1  # Hz
	rate = rospy.Rate(loop_freq)
	rospy.wait_for_service('mavros/cmd/land')
	land_set = rospy.ServiceProxy('mavros/cmd/land', CommandTOL)
	land_cmd = [0,0,0,0,0]
	
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

def local_position_cb(msg):
	global current_alt
	global cur_x
	global cur_y
	current_alt = msg.pose.position.z
	cur_x = msg.pose.position.x
	cur_y = msg.pose.position.y

def setPoint_cb(msg):
	global x
	global y
	global update_pos
	global cur_x
	global cur_y
	x=msg.data[0]
	y=msg.data[1]
	x = min(max(0,x),1.8)
	y = min(max(-2.3,y),6.4)

	if abs(x-cur_x) > 0.35 or abs(y-cur_y) > 0.35:
		update_pos = True

def sendTargetPos():
	global x
	global y
	global z
	global update_pos
	global cur_x
	global cur_y
	global setpointPub
    global stopThread
	setpoint = PoseStamped()
	setpoint.pose.position.x = 0
	setpoint.pose.position.y = 0
	setpoint.pose.position.z = z

	init_time = time.time()
	while not rospy.is_shutdown():
		if update_pos:
			x_arr = np.linspace(cur_x,x,20)
			y_arr = np.linspace(cur_y,y,20)
			for i in range(len(x_arr)):
				setpoint.pose.position.x = x_arr[i]
				setpoint.pose.position.y = y_arr[i]
				setpoint.pose.position.z = z
			    setpointPub.publish(setpoint)
		    	rate.sleep()
		    update_pos = False
		else:
			setpoint.pose.position.x = x
			setpoint.pose.position.y = y
			setpoint.pose.position.z = z
		    setpointPub.publish(setpoint)
	    	rate.sleep()
	    if stopThread:
            try:
                land(5)
            except Exception as e:
                print(e)
            rospy.sleep(5)
            try:
                set_arm(False, 5)
            except Exception as e:
                print(e)
            break

    print("closing sendTargetPos thread")



if __name__ == '__main__':

	rospy.init_node('drone_cluster_center1', anonymous=True)

	rospy.Subscriber('/mavros/state', State, state_cb)
	rospy.Subscriber('/mavros/local_position/pose',PoseStamped,local_position_cb)

	rospy.Subscriber("/drone1SetPoint",Float32MultiArray,setPoint_cb)

	setpointPub = rospy.Publisher('mavros/setpoint_position/local',PoseStamped,queue_size=100)
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

		threading.Thread(target=sendTargetPos).start()
        rate = rospy.Rate(5.0)
        while not rospy.is_shutdown():
            rate.sleep()
            cmd = raw_input("command: ")
            # print(cmd)
            if cmd=='land':
                stopThread = True
            elif cmd=='exit':
                break




		
