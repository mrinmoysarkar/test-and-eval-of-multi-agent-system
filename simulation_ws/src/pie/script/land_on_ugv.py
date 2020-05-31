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
from math import radians, degrees, pi, cos, sin
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist
import time


current_state = None
current_alt = 0
hx=0.947
hy=1.687
hz=2.0
curx = 0
cury = 0

def state_cb(msg):
    global current_state
    current_state = msg
    #print(current_state)

def set_arm(arm, timeout):
    loop_freq = 1  # Hz
    rate = rospy.Rate(loop_freq)
    rospy.wait_for_service('/mavros/cmd/arming')
    arming_srv = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
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
    rospy.wait_for_service('/mavros/cmd/land')
    land_set = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)
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
    rospy.wait_for_service('/mavros/set_mode')
    mode_set = rospy.ServiceProxy('/mavros/set_mode', SetMode)
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
    global curx
    global cury
    current_alt = msg.pose.position.z
    curx = msg.pose.position.x
    cury = msg.pose.position.y

def getquaternion(roll, pitch, yaw):
    # return tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    roll = radians(roll)
    pitch = radians(pitch)
    yaw = radians(yaw)
    cy = cos(yaw * 0.5);
    sy = sin(yaw * 0.5);
    cp = cos(pitch * 0.5);
    sp = sin(pitch * 0.5);
    cr = cos(roll * 0.5);
    sr = sin(roll * 0.5);

    q=[0,0,0,0]
    q[0] = -1*(cy * cp * sr - sy * sp * cr)
    q[1] = -1*(sy * cp * sr + cy * sp * cr)
    q[2] = -1*(sy * cp * cr - cy * sp * sr)
    q[3] = -1*(cy * cp * cr + sy * sp * sr)
    
    return q;

def home_cb(msg):
    global hx
    global hy
    global hz
    hx = msg.transform.translation.x
    hy = msg.transform.translation.y
    hz = msg.transform.translation.z
    hx = min(max(0.2,hx),1.8)
    hy = min(max(-2.0,hy),6.0)
    print(hx,hy)

if __name__ == '__main__':
    rospy.init_node('land_on_ugv_scenario', anonymous=True)
    rospy.Subscriber('/mavros/state', State, state_cb)
    rospy.Subscriber('/mavros/local_position/pose',PoseStamped,local_position_cb)
    rospy.Subscriber('/vicon/home/home',TransformStamped,home_cb)

    setpointPub = rospy.Publisher('/mavros/setpoint_position/local',PoseStamped,queue_size=100)
    setpoint = PoseStamped()
    setpoint.pose.position.x = 0.947
    setpoint.pose.position.y = 1.687
    setpoint.pose.position.z = 2.5

    setvelPub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped',Twist,queue_size=100)
    setvel = Twist()
    setvel.linear.x=0
    setvel.linear.y=0
    setvel.linear.z=-0.2

    rate = rospy.Rate(20.0)
    for i in range(100):
        setpointPub.publish(setpoint);
        rate.sleep()
    if set_mode('OFFBOARD', 5) and set_arm(True, 5):
        init_time = time.time()
        while not rospy.is_shutdown():
            setpointPub.publish(setpoint)
            rate.sleep()
            if (time.time()-init_time) > 10:
                break
        # setpoint.pose.position.z = 1.0
        # init_time = time.time()
        # while not rospy.is_shutdown():
        #     setpointPub.publish(setpoint)
        #     rate.sleep()
        #     if (time.time()-init_time) > 5:
        #         break

        # q = getquaternion(0,0,90)
        # setpoint.pose.orientation.x = q[0]
        # setpoint.pose.orientation.y = q[1]
        # setpoint.pose.orientation.z = q[2]
        # setpoint.pose.orientation.w = q[3]
        # init_time = time.time()
        # while not rospy.is_shutdown():
        #     setpointPub.publish(setpoint);
        #     rate.sleep()
        #     if (time.time()-init_time) > 5:
        #         break

        alti = np.linspace(2.5,1,100)
        yy = np.linspace(1.687,4.22,100)
        p = 0 
        for i in range(len(alti)):
            for j in range(2):
                setpoint.pose.position.x = 0.93
                setpoint.pose.position.y = yy[i]
                setpoint.pose.position.z = alti[i]
                setpointPub.publish(setpoint)
                rate.sleep()
        for j in range(5):
            setpoint.pose.position.x = 0.93
            setpoint.pose.position.y = 4.22
            setpoint.pose.position.z = 1
            setpointPub.publish(setpoint)
            rate.sleep()

        inn = 0

        for j in range(500):
            setpoint.pose.position.x = hx
            setpoint.pose.position.y = hy-0.4
            setpoint.pose.position.z = 1
            setpointPub.publish(setpoint)
            rate.sleep()
        # for j in range(20):
        #     setpoint.pose.position.x = 1.31
        #     setpoint.pose.position.y = hy
        #     setpoint.pose.position.z = hz
        #     setpointPub.publish(setpoint)
        #     rate.sleep()
        
        # breaking_flag = False
        # init_time = time.time()
        # while not rospy.is_shutdown():
        #     setpoint.pose.position.x = 1.2
        #     setpoint.pose.position.y = 4.2
        #     setpoint.pose.position.z = 2.0
        #     setpointPub.publish(setpoint)
        #     rate.sleep()
        #     dx = curx-1.2
        #     dy = cury-4.2
        #     d = (dx**2+dy**2)**0.5
        #     print(d)
        #     if d <= 0.05:
        #         print("breaking")
        #         breaking_flag = True
        #         break
        #     if (time.time()-init_time) > 10:
        #         break

        # print("publish set velocity ")
        # init_time = time.time()
        # setvel.linear.z=-0.1
        # while not rospy.is_shutdown(): 
        #     setvelPub.publish(setvel)
        #     rate.sleep()
        #     if current_alt < 0.8 and current_alt > 2.5:
        #         setvel.linear.z=0
        #         print('breaking2')
        #     if (time.time()-init_time) > 10:
        #         print("breaking3")
        #         break

        # setvel.linear.z=0
        # setvelPub.publish(setvel)
        land(5)
        rospy.sleep(2.5)
        set_arm(False, 5)