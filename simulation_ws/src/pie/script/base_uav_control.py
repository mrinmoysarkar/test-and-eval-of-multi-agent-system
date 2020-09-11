#!/usr/bin/env python

from __future__ import print_function
import rospy
from std_msgs.msg import String
import time
import os
import sys
from mavros_msgs.msg import State, ParamValue
from geometry_msgs.msg import Pose, PoseStamped, Point, Vector3, TwistStamped, TransformStamped
from std_msgs.msg import Header, ColorRGBA, String
from mavros_msgs.srv import CommandBool, ParamGet, SetMode, CommandTOL, ParamSet
from math import radians, sin, cos
import threading
import rosgraph



class uavControl():
    def __init__(self, uav_num):
        self.current_state = None
        self.setpointPub = None
        self.stopThread = False
        self.uav_num = uav_num
        self.currentPosition = Pose()
        self.targetPos = PoseStamped()
        
        self.setpointPub = rospy.Publisher('/uav'+str(self.uav_num)+'/mavros/setpoint_position/local', PoseStamped, queue_size=100)
        self.setpointVelPub = rospy.Publisher('/uav'+str(self.uav_num)+'/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=100)
        
        rospy.Subscriber('/uav'+str(self.uav_num)+'/mavros/state', State, self.state_cb)
        rospy.Subscriber('/uav'+str(self.uav_num)+'/mavros/local_position/pose', PoseStamped, self.local_position_cb)
        


       
    def state_cb(self,msg):
        self.current_state = msg

    def get_param(self, param_id, timeout):
        loop_freq = 1  # Hz
        rate = rospy.Rate(loop_freq)
        rospy.wait_for_service('/uav'+str(self.uav_num)+'/mavros/param/get')
        paramget_srv = rospy.ServiceProxy('/uav'+str(self.uav_num)+'/mavros/param/get', ParamGet)
        for i in xrange(timeout * loop_freq):
            try:
                res = paramget_srv(param_id=param_id)
                if not res.success:
                    print(res.value)
                    rospy.logerr("failed to set parameter")
                else:
                    print(res.value)
                    return True
            except rospy.ServiceException as e:
                rospy.logerr(e)
            try:
                rate.sleep()
            except rospy.ROSException as e:
                rospy.logerr(e)
        return False

    def set_param(self, param_id, param_val, timeout):
        loop_freq = 1  # Hz
        rate = rospy.Rate(loop_freq)
        rospy.wait_for_service('/uav'+str(self.uav_num)+'/mavros/param/set')
        paramset_srv = rospy.ServiceProxy('/uav'+str(self.uav_num)+'/mavros/param/set', ParamSet)
        paramValue = ParamValue()
        paramValue.integer = param_val
        paramValue.real = 0.0
        for i in xrange(timeout * loop_freq):
            try:
                res = paramset_srv(param_id=param_id, value=paramValue)
                if not res.success:
                    rospy.logerr("failed to set parameter")
                else:
                    return True
            except rospy.ServiceException as e:
                rospy.logerr(e)
            try:
                rate.sleep()
            except rospy.ROSException as e:
                rospy.logerr(e)
        return False  

    def set_arm(self, arm, timeout):
        loop_freq = 1  # Hz
        rate = rospy.Rate(loop_freq)
        rospy.wait_for_service('/uav'+str(self.uav_num)+'/mavros/cmd/arming')
        arming_srv = rospy.ServiceProxy('/uav'+str(self.uav_num)+'/mavros/cmd/arming', CommandBool)
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
        rospy.wait_for_service('/uav'+str(self.uav_num)+'/mavros/cmd/land')
        land_set = rospy.ServiceProxy('/uav'+str(self.uav_num)+'/mavros/cmd/land', CommandTOL)
        
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

    def set_mode(self, mode, timeout, base_mode=0):
        loop_freq = 1  # Hz
        rate = rospy.Rate(loop_freq)
        rospy.wait_for_service('/uav'+str(self.uav_num)+'/mavros/set_mode')
        mode_set = rospy.ServiceProxy('/uav'+str(self.uav_num)+'/mavros/set_mode', SetMode)
        for i in xrange(timeout * loop_freq):
            if self.current_state.mode == mode:
                return True
            else:
                try:
                    # res = mode_set(base_mode,mode)  # 0 is custom mode
                    res = mode_set(custom_mode=mode)  # 0 is custom mode
                    if not res.mode_sent:
                        # print(res)
                        rospy.logerr("failed to send mode command")
                    else:
                        # print(res)
                        return True
                except rospy.ServiceException as e:
                    rospy.logerr(e)

            try:
                rate.sleep()
            except rospy.ROSException as e:
                self.fail(e)

        return False

    def local_position_cb(self,msg):
        self.currentPosition.position.x = msg.pose.position.x
        self.currentPosition.position.y = msg.pose.position.y
        self.currentPosition.position.z = msg.pose.position.z
        self.currentPosition.orientation.x = msg.pose.orientation.x
        self.currentPosition.orientation.y = msg.pose.orientation.y
        self.currentPosition.orientation.z = msg.pose.orientation.z
        self.currentPosition.orientation.w = msg.pose.orientation.w 

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

    def getDistance(self, x=None, y=None, z=None, pos=None):
        if x is not None and y is not None and z is not None:
            dx = self.currentPosition.position.x-x
            dy = self.currentPosition.position.y-y
            dz = self.currentPosition.position.z-z
            return (dx**2+dy**2+dz**2)**0.5
        elif pos is not None:
            dx = self.currentPosition.position.x-pos.position.x
            dy = self.currentPosition.position.y-pos.position.y
            dz = self.currentPosition.position.z-pos.position.z
            return (dx**2+dy**2+dz**2)**0.5
        else:
            dx = self.currentPosition.position.x-self.targetPos.pose.position.x
            dy = self.currentPosition.position.y-self.targetPos.pose.position.y
            dz = self.currentPosition.position.z-self.targetPos.pose.position.z
            return (dx**2+dy**2+dz**2)**0.5

    def loop(self):
        rospy.sleep(5)
        rate = rospy.Rate(20.0)
        
        q = self.getquaternion(0,0,0)
        setpoint = PoseStamped()
        setpoint.pose.position.x = self.currentPosition.position.x
        setpoint.pose.position.y = self.currentPosition.position.y
        setpoint.pose.position.z = 1.5
        # print(setpoint)
        setpoint.pose.orientation.x = q[0]
        setpoint.pose.orientation.y = q[1]
        setpoint.pose.orientation.z = q[2]
        setpoint.pose.orientation.w = q[3]
        for i in range(100):
            self.setpointPub.publish(setpoint)
            rate.sleep()

        
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



   





    


