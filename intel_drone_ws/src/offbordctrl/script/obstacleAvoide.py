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
import threading
import tf
from math import radians, degrees, pi, cos, sin
from periphery import GPIO


current_state = None
current_alt = 0
setpointPub = None
stopThread = False
rotateFlag = False
gx=0
gy=0
gz=0
rx=0
ry=0
rz=0
rw=-1.0
last_roll = 0
last_pitch = 0
last_yaw = 0
set_yaw = 0

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
    
    for i in xrange(timeout * loop_freq):
        if current_state.armed == False:
            return True
        else:
            try:
                res = land_set(min_pitch=0.0,
                         yaw=0.0,
                         latitude=float('nan'),
                         longitude=float('nan'),
                         altitude=current_alt)  
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
    global rx
    global ry
    global rz
    global rw
    global last_roll 
    global last_pitch 
    global last_yaw 
    current_alt = msg.pose.position.z
    # rx = msg.pose.orientation.x
    # ry = msg.pose.orientation.y
    # rz = msg.pose.orientation.z
    # rw = msg.pose.orientation.w
    # print("************************************")
    eular = getrpy(msg.pose)
    last_roll = degrees(eular[0])
    last_pitch = degrees(eular[1])
    last_yaw = degrees(eular[2])
    # print("roll: ",eular[0],"pitch:",eular[1],"yaw:",eular[2])
    # q = getquaternion(degrees(eular[0]),degrees(eular[1]),degrees(eular[2]))
    # print("quaternion:",q)
    # print("************************************")

def getrpy(pose):
    quaternion = (
    pose.orientation.x,
    pose.orientation.y,
    pose.orientation.z,
    pose.orientation.w)
    # print("quaternion:",quaternion)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    # roll = euler[0]
    # pitch = euler[1]
    # yaw = euler[2]
    return euler

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

def sendTargetPos():
    global setpointPub
    global stopThread
    global rotateFlag
    global gx
    global gy
    global gz
    global rx
    global ry
    global rz
    global rw
    global last_roll 
    global last_pitch 
    global last_yaw 
    global set_yaw
    setpoint = PoseStamped()
    rate = rospy.Rate(20.0)
    while True:
        setpoint.pose.position.x = gx
        setpoint.pose.position.y = gy
        setpoint.pose.position.z = gz
        # if rotateFlag:
        # q = getquaternion(last_roll,last_pitch,set_yaw)
        q = getquaternion(0,0,set_yaw)
        setpoint.pose.orientation.x = q[0]
        setpoint.pose.orientation.y = q[1]
        setpoint.pose.orientation.z = q[2]
        setpoint.pose.orientation.w = q[3]

        # else:
        #     setpoint.pose.orientation.x = rx
        #     setpoint.pose.orientation.y = ry
        #     setpoint.pose.orientation.z = rz
        #     setpoint.pose.orientation.w = rw

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

    rospy.init_node('hover_and_land_scenario', anonymous=True)
    rospy.Subscriber('/mavros/state', State, state_cb)
    rospy.Subscriber('/mavros/local_position/pose',PoseStamped,local_position_cb)

    setpointPub = rospy.Publisher('mavros/setpoint_position/local',PoseStamped,queue_size=100)
    # rospy.spin()
    setpoint = PoseStamped()
    setpoint.pose.position.x = 0
    setpoint.pose.position.y = 0
    setpoint.pose.position.z = 1.5

    obstaclePin = 485

    obstacle = GPIO(obstaclePin, "in") 

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

        gx=0
        gy=0
        gz=1.5

        threading.Thread(target=sendTargetPos).start()
        rate = rospy.Rate(5.0)
        while not rospy.is_shutdown():
            rate.sleep()
            cmd = raw_input("command: ")
            # print(cmd)
            if cmd=='land':
                stopThread = True
            elif cmd=='r':
                set_yaw = 90
            elif cmd=='w':
                gy += 0.4
            elif cmd=='s':
                gy -= 0.4
            elif cmd == 'd':
                gx += 0.4
            elif cmd =='a':
                gx -=0.4
            elif cmd=='o':
                obstacleflag = obstacle.read()
                if obstacleflag:
                    print("there is obstacle in fornt")
                else:
                    print("there is no obstacle in fornt")
            elif cmd=='exit':
                break
        obstacle.close() 