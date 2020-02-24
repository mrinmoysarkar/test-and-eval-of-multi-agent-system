#!/usr/bin/env python

from __future__ import print_function

import rospy
from std_msgs.msg import String
import numpy as np
import cv2

import time
import os
import sys
from mavros_msgs.msg import State, ParamValue
from geometry_msgs.msg import PoseStamped, Point, Vector3, TwistStamped
from std_msgs.msg import Header, ColorRGBA, String
from mavros_msgs.srv import CommandBool, ParamGet, SetMode, CommandTOL, ParamSet
from math import radians, sin, cos
import time
import threading
from visualization_msgs.msg import Marker
from nav_msgs.msg import Path
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import matplotlib.pyplot as plt



class uavControl():
    def __init__(self,uavno):
        self.current_state = None
        self.setpointPub = None
        self.stopThread = False
        self.direction = 0
        self.uavno = uavno
        self.currentPosition = None
        self.currentImage = None
        self.takeoff_flag = False
        self.land_flag = False
        self.path = Path()
        self.bridge_rgb = CvBridge()
        self.count = 0
        self.targetfound = False
        self.setpoint = [1,1,1]
        rospy.Subscriber('/uav'+str(self.uavno)+'/mavros/state', State, self.state_cb)
        rospy.Subscriber('/uav'+str(self.uavno)+'/mavros/local_position/pose', PoseStamped, self.local_position_cb)
        rospy.Subscriber('/uav'+str(self.uavno)+'/r200_ir/image_raw', Image, self.image_data_cb)
        self.setpointPub = rospy.Publisher('/uav'+str(self.uavno)+'/mavros/setpoint_position/local', PoseStamped, queue_size=100)
        self.setpointVelPub = rospy.Publisher('/uav'+str(self.uavno)+'/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=100)
        # self.marker_publisher = rospy.Publisher('/uav'+str(self.uavno)+'/visualization_marker', Marker, queue_size=5)
        # self.path_pub = rospy.Publisher('/uav'+str(self.uavno)+'/path', Path, queue_size=10)

    def image_data_cb(self, msg):
        self.currentImage = self.bridge_rgb.imgmsg_to_cv2(msg, msg.encoding)
       
    def state_cb(self,msg):
        self.current_state = msg

    def get_param(self, param_id, timeout):
        loop_freq = 1  # Hz
        rate = rospy.Rate(loop_freq)
        rospy.wait_for_service('/uav'+str(self.uavno)+'/mavros/param/get')
        paramget_srv = rospy.ServiceProxy('/uav'+str(self.uavno)+'/mavros/param/get', ParamGet)
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
        rospy.wait_for_service('/uav'+str(self.uavno)+'/mavros/param/set')
        paramset_srv = rospy.ServiceProxy('/uav'+str(self.uavno)+'/mavros/param/set', ParamSet)
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

    def set_mode(self, mode, timeout, base_mode=0):
        loop_freq = 1  # Hz
        rate = rospy.Rate(loop_freq)
        rospy.wait_for_service('/uav'+str(self.uavno)+'/mavros/set_mode')
        mode_set = rospy.ServiceProxy('/uav'+str(self.uavno)+'/mavros/set_mode', SetMode)
        for i in xrange(timeout * loop_freq):
            if self.current_state.mode == mode:
                return True
            else:
                try:
                    # res = mode_set(base_mode,mode)  # 0 is custom mode
                    res = mode_set(custom_mode=mode)  # 0 is custom mode
                    if not res.mode_sent:
                        print(res)
                        rospy.logerr("failed to send mode command")
                    else:
                        print(res)
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
        # to visualize in rviz
         
        # self.path.header = msg.header
        # self.path.poses.append(msg)
        # self.path_pub.publish(self.path)
        
        # marker = Marker(
        #             type=Marker.SPHERE,
        #             id=self.count,
        #             lifetime=rospy.Duration(1000),
        #             pose=msg.pose,
        #             scale=Vector3(0.05, 0.05, 0.05),
        #             header=msg.header,
        #             color=ColorRGBA(0.0, 2.0, 0.0, 0.8))
        # self.count += 1
        # self.marker_publisher.publish(marker) 

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

    def getDistance(self,x,y,z):
        dx = self.currentPosition.position.x-x
        dy = self.currentPosition.position.y-y
        dz = self.currentPosition.position.z-z
        return (dx**2+dy**2+dz**2)**0.5

    def publishSetPoints(self,startX,startY,startZ,endX,endY,endZ):
        rate = rospy.Rate(20.0)
        q = self.getquaternion(0, 0, 0)
        setpoint = PoseStamped()
        setpoint.pose.position.x = startX
        setpoint.pose.position.y = startY
        setpoint.pose.position.z = startZ
        setpoint.pose.orientation.x = q[0]
        setpoint.pose.orientation.y = q[1]
        setpoint.pose.orientation.z = q[2]
        setpoint.pose.orientation.w = q[3]
        n = 50
        x_trajectory = np.linspace(startX, endX, n)
        y_trajectory = np.linspace(startY, endY, n)
        z_trajectory = np.linspace(startZ, endZ, n)
        for j in range(n):
            setpoint.pose.position.x = x_trajectory[j]
            setpoint.pose.position.y = y_trajectory[j]
            setpoint.pose.position.z = z_trajectory[j]
            self.setpointPub.publish(setpoint)
            rate.sleep()

        setpoint.pose.position.x = endX
        setpoint.pose.position.y = endY
        setpoint.pose.position.z = endZ

        # gain altitude
        counter = 0
        while self.getDistance(endX, endY, endZ) > 0.1 and counter < 100:
            self.setpointPub.publish(setpoint)
            rate.sleep()
            counter += 1

    def executeVerticalScenario(self):
        d = 3.0
        deld = 0.5
        x,y,z=self.currentPosition.position.x,self.currentPosition.position.y,5.0
        rate = rospy.Rate(20.0)
        q = self.getquaternion(0,0,0)
        setpoint = PoseStamped()
        setpoint.pose.position.x = x
        setpoint.pose.position.y = y
        setpoint.pose.position.z = z
        setpoint.pose.orientation.x = q[0]
        setpoint.pose.orientation.y = q[1]
        setpoint.pose.orientation.z = q[2]
        setpoint.pose.orientation.w = q[3]

        for i in range(100):
            self.setpointPub.publish(setpoint)
            rate.sleep()

        if self.set_mode('OFFBOARD', 5) and self.set_arm(True, 5):
            #takeoff
            while self.getDistance(x,y,z)>0.1:
                self.setpointPub.publish(setpoint)
                rate.sleep()

            self.publishSetPoints(x,y,z,x+d,y,z+d)
            x, y, z = x + d, y, z + d
            self.publishSetPoints(x, y, z, x, y + d, z)
            x, y, z = x, y + d, z
            self.publishSetPoints(x, y, z, x - d, y, z)
            x, y, z = x - d, y, z
            self.publishSetPoints(x, y, z, x, y + d, z - d)
            x, y, z = x, y + d, z - d
            self.publishSetPoints(x, y, z, x + d, y, z)
            x, y, z = x + d, y, z
            self.publishSetPoints(x, y, z, x - d + deld, y - 2 * d + deld, z)
            x, y, z = x - d + deld, y - 2 * d + deld, z
            self.publishSetPoints(x, y, z, x - deld, y - deld, 0.1)
            x, y, z = x - deld, y - deld, 0.1

            self.land(5)
            rospy.sleep(10)
            self.set_arm(False, 5)

    def executeSquareScenario(self):
        d = 4.0
        deld = 0.5
        x,y,z=self.currentPosition.position.x,self.currentPosition.position.y,3.0
        rate = rospy.Rate(20.0)
        q = self.getquaternion(0,0,0)
        setpoint = PoseStamped()
        setpoint.pose.position.x = x
        setpoint.pose.position.y = y
        setpoint.pose.position.z = z
        setpoint.pose.orientation.x = q[0]
        setpoint.pose.orientation.y = q[1]
        setpoint.pose.orientation.z = q[2]
        setpoint.pose.orientation.w = q[3]

        for i in range(100):
            self.setpointPub.publish(setpoint)
            rate.sleep()

        if self.set_mode('OFFBOARD', 5) and self.set_arm(True, 5):
            #takeoff
            while self.getDistance(x,y,z)>0.1:
                self.setpointPub.publish(setpoint)
                rate.sleep()

            self.publishSetPoints(x,y,z,x+d,y,z)
            x, y, z = x + d, y, z 
            self.publishSetPoints(x, y, z, x, y + d, z)
            x, y, z = x, y + d, z
            self.publishSetPoints(x, y, z, x - d, y, z)
            x, y, z = x - d, y, z
            self.publishSetPoints(x, y, z, x, y - d, z)
            x, y, z = x, y - d, z
            self.publishSetPoints(x, y, z, x, y, 0.1)
            

            self.land(5)
            rospy.sleep(10)
            self.set_arm(False, 5)

    def controlVelocity(self):
        rate = rospy.Rate(20.0)
        setpoint = TwistStamped()
        setpoint.header.stamp = rospy.Time.now()
        # setpoint.header.frame_id = ""
        setpoint.twist.linear.x = 0
        setpoint.twist.linear.y = 0
        setpoint.twist.linear.z = 0.5
        setpoint.twist.angular.x = 0
        setpoint.twist.angular.y = 0
        setpoint.twist.angular.z = 0
        

        for i in range(100):
            self.setpointVelPub.publish(setpoint)
            rate.sleep()

        if self.set_mode('OFFBOARD', 5) and self.set_arm(True, 5):
            #takeoff
            while not self.stopThread:
                if self.currentPosition.position.z < 2:
                    setpoint.twist.linear.z = 0.5
                else:
                    setpoint.twist.linear.z = 0.0
                    break

                self.setpointVelPub.publish(setpoint)
                rate.sleep()
            print("i am going to second loop")
            kpx = 0.2
            kpy = 0.2
            kpz = 0.2
            while not self.stopThread:
                dx = self.setpoint[0]-self.currentPosition.position.x
                dy = self.setpoint[1]-self.currentPosition.position.y
                dz = self.setpoint[2]-self.currentPosition.position.z
                # print("dx: " + str(dx) + " dy: " + str(dy) + " dz: " + str(dz))
                setpoint.twist.linear.x = kpx*dx
                setpoint.twist.linear.y = kpy*dy
                setpoint.twist.linear.z = kpz*dz
                setpoint.twist.angular.x = 0
                setpoint.twist.angular.y = 0
                setpoint.twist.angular.z = 0
                self.setpointVelPub.publish(setpoint)
                rate.sleep()
            # self.land(5)
            print('landing loop begins')
            self.stopThread = False
            while not self.stopThread:
                dx = self.setpoint[0]-self.currentPosition.position.x
                dy = self.setpoint[1]-self.currentPosition.position.y
                dz = 0.1-self.currentPosition.position.z
                # print("dx: " + str(dx) + " dy: " + str(dy) + " dz: " + str(dz))
                setpoint.twist.linear.x = kpx*dx
                setpoint.twist.linear.y = kpy*dy
                setpoint.twist.linear.z = kpz*dz
                setpoint.twist.angular.x = 0
                setpoint.twist.angular.y = 0
                setpoint.twist.angular.z = 0
                self.setpointVelPub.publish(setpoint)
            print(self.current_state)
            self.set_mode('MANUAL', 5)
            rospy.sleep(5)
            print(self.current_state)
            self.set_arm(False, 5)

    def executeSquareScenarioCompactspace(self):
        d = 5.0
        deld = 0.5
        x,y,z=self.currentPosition.position.x,self.currentPosition.position.y,3.0
        rate = rospy.Rate(20.0)
        q = self.getquaternion(0,0,0)
        setpoint = PoseStamped()
        setpoint.pose.position.x = x
        setpoint.pose.position.y = y
        setpoint.pose.position.z = z
        setpoint.pose.orientation.x = q[0]
        setpoint.pose.orientation.y = q[1]
        setpoint.pose.orientation.z = q[2]
        setpoint.pose.orientation.w = q[3]

        for i in range(100):
            self.setpointPub.publish(setpoint)
            rate.sleep()

        if self.set_mode('OFFBOARD', 5) and self.set_arm(True, 5):
            #takeoff
            while self.getDistance(x,y,z)>0.1:
                self.setpointPub.publish(setpoint)
                rate.sleep()
            if self.uavno==0:
                for i in range(180):
                    self.setpointPub.publish(setpoint)
                    rate.sleep()

                self.publishSetPoints(x,y,z,x+d,y,z)
                x, y, z = x + d, y, z 
                self.publishSetPoints(x, y, z, x, y + d, z)
                x, y, z = x, y + d, z
                self.publishSetPoints(x, y, z, x - d, y, z)
                x, y, z = x - d, y, z
                self.publishSetPoints(x, y, z, x, y - d, z)
                x, y, z = x, y - d, z
                self.publishSetPoints(x, y, z, x, y, 0.1)
            elif self.uavno==1:
                for i in range(120):
                    self.setpointPub.publish(setpoint)
                    rate.sleep()
                self.publishSetPoints(x, y, z, x, y + d, z)
                x, y, z = x, y + d, z
                self.publishSetPoints(x, y, z, x - d, y, z)
                x, y, z = x - d, y, z
                self.publishSetPoints(x, y, z, x, y - d, z)
                x, y, z = x, y - d, z
                self.publishSetPoints(x,y,z,x+d,y,z)
                x, y, z = x + d, y, z
                self.publishSetPoints(x, y, z, x, y, 0.1)
            elif self.uavno==2:
                for i in range(80):
                    self.setpointPub.publish(setpoint)
                    rate.sleep()
                self.publishSetPoints(x, y, z, x - d, y, z)
                x, y, z = x - d, y, z
                self.publishSetPoints(x, y, z, x, y - d, z)
                x, y, z = x, y - d, z
                self.publishSetPoints(x,y,z,x+d,y,z)
                x, y, z = x + d, y, z
                self.publishSetPoints(x, y, z, x, y + d, z)
                x, y, z = x, y + d, z
                self.publishSetPoints(x, y, z, x, y, 0.1)
            elif self.uavno==3:
                self.publishSetPoints(x, y, z, x, y - d, z)
                x, y, z = x, y - d, z
                self.publishSetPoints(x,y,z,x+d,y,z)
                x, y, z = x + d, y, z
                self.publishSetPoints(x, y, z, x, y + d, z)
                x, y, z = x, y + d, z
                self.publishSetPoints(x, y, z, x - d, y, z)
                x, y, z = x - d, y, z
                self.publishSetPoints(x, y, z, x, y, 0.1)
                 
            

            self.land(5)
            rospy.sleep(10)
            self.set_arm(False, 5)

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

    def takeoff(self):
         self.takeoff_flag = True

    def land_custom(self):
        self.land_flag = True

    def disarm(self):
        self.land_flag = False

    def shutdown(self):
        self.stopThread = True

    def go(self,direction):
        self.direction = direction

    def control_loop(self):
        target_vel = 0.5
        rate = rospy.Rate(20.0)
        setpoint = TwistStamped()
        setpoint.header.stamp = rospy.Time.now()
        setpoint.twist.linear.x = 0
        setpoint.twist.linear.y = 0
        setpoint.twist.linear.z = 0.5
        setpoint.twist.angular.x = 0
        setpoint.twist.angular.y = 0
        setpoint.twist.angular.z = 0
        
        while not self.stopThread:
            if self.takeoff_flag:
                self.takeoff_flag = False
                for i in range(100):
                    self.setpointVelPub.publish(setpoint)
                    rate.sleep()

                if self.set_mode('OFFBOARD', 5) and self.set_arm(True, 5):
                    #takeoff
                    while not self.stopThread:
                        if self.currentPosition.position.z < 5:
                            setpoint.twist.linear.z = 0.5
                        else:
                            setpoint.twist.linear.z = 0.0
                            break

                        self.setpointVelPub.publish(setpoint)
                        rate.sleep()

                    setpoint.twist.linear.z = 0
                    while not self.land_flag:
                        # if self.targetfound:
                        #     self.survey_state(self.currentPosition.position.x,self.currentPosition.position.y,3)
                        if self.direction == 0:
                            setpoint.twist.linear.x = 0
                            setpoint.twist.linear.y = 0
                            setpoint.twist.linear.z = 0
                        elif self.direction == 1:
                            setpoint.twist.linear.x = target_vel
                            # self.position_ctl_loop(-3,-3,5)
                            # self.survey_state(-3,-3,3)
                        elif self.direction == -1:
                            setpoint.twist.linear.x = -target_vel
                        elif self.direction == 2:
                            setpoint.twist.linear.y = target_vel
                        elif self.direction == -2:
                            setpoint.twist.linear.y = -target_vel
                        elif self.direction == 3:
                            setpoint.twist.linear.z = target_vel
                        elif self.direction == -3:
                            setpoint.twist.linear.z = -target_vel

                        self.setpointVelPub.publish(setpoint)
                        rate.sleep()
                    
                    setpoint.twist.linear.z = -0.5
                    while self.land_flag:
                        if self.currentPosition.position.z < 0.1:
                            break
                        self.setpointVelPub.publish(setpoint)
                        rate.sleep()
                    self.land_flag = False
                    self.set_mode('MANUAL', 5)
                    rospy.sleep(5)
                    self.set_arm(False, 5)

    def position_ctl_loop(self,target_x,target_y,target_z=None):
        if None is target_z:
            target_z = self.currentPosition.position.z 

        kp,ki,kd = 0.5,0.01,0.1
        err_x, err_y, err_z = 0,0,0
        prev_err_x, prev_err_y, prev_err_z = 0,0,0
        del_err_x, del_err_y, del_err_z = 0,0,0
        int_err_x, int_err_y, int_err_z = 0,0,0
        
        dt = 1.0/20.0
        rate = rospy.Rate(20.0)
        setpoint = TwistStamped()
        setpoint.header.stamp = rospy.Time.now()
        setpoint.twist.linear.x = 0
        setpoint.twist.linear.y = 0
        setpoint.twist.linear.z = 0
        setpoint.twist.angular.x = 0
        setpoint.twist.angular.y = 0
        setpoint.twist.angular.z = 0

        # ex = []
        # ey = []
        # ez = []
        t1 = time.time()

        while not self.stopThread:
            err_x = target_x - self.currentPosition.position.x
            err_y = target_y - self.currentPosition.position.y
            err_z = target_z - self.currentPosition.position.z 
            
            del_err_x = err_x - prev_err_x
            del_err_y = err_y - prev_err_y
            del_err_z = err_z - prev_err_z

            int_err_x = int_err_x + err_x
            int_err_y = int_err_y + err_y
            int_err_z = int_err_z + err_z

            setpoint.twist.linear.x = kp*err_x + ki*int_err_x*dt + kd*del_err_x/dt
            setpoint.twist.linear.y = kp*err_y + ki*int_err_y*dt + kd*del_err_y/dt
            setpoint.twist.linear.z = kp*err_z + ki*int_err_z*dt + kd*del_err_z/dt

            prev_err_x, prev_err_y, prev_err_z = err_x, err_y, err_z

            # ex.append(err_x)
            # ey.append(err_y)
            # ez.append(err_z)

            # print(err_x)
            self.setpointVelPub.publish(setpoint)
            rate.sleep()
            err_d = (err_x**2 + err_y**2 + err_z**2)**0.5
            
            if err_d < 0.2 or time.time() - t1 > 5:
                break
        setpoint.twist.linear.x = 0
        setpoint.twist.linear.y = 0
        setpoint.twist.linear.z = 0
        self.setpointVelPub.publish(setpoint)
        # print(err_d)

        # plt.plot(ex,label='err_x')
        # plt.plot(ey,label='err_y')
        # plt.plot(ez,label='err_z')
        # plt.grid()
        # plt.legend()
        # plt.show()

    def survey_state(self,center_x,center_y, n_times):
        r = 1.5
        theta = np.linspace(0,2*np.pi,12)
        x_data = center_x + r*np.cos(theta)
        y_data = center_y + r*np.sin(theta)

        for _ in range(n_times):
            for x,y in zip(x_data,y_data):
                self.position_ctl_loop(x,y)

    def object_detection(self):
        rate = rospy.Rate(20.0)
        
        # plt.axis()
        # plt.ion()
        # plt.show()
        while not self.stopThread:
            if None is not self.currentImage:
                _,img_red = cv2.threshold(self.currentImage[:,:,0], 200, 255, cv2.THRESH_BINARY)
                _,img_green = cv2.threshold(self.currentImage[:,:,1], 200, 255, cv2.THRESH_BINARY_INV)
                _,img_blue = cv2.threshold(self.currentImage[:,:,2], 200, 255, cv2.THRESH_BINARY_INV)
                # img_green_inv = cv2.bitwise_not(img_green)
                # img_blue_inv = cv2.bitwise_not(img_blue)
                obj_img = cv2.bitwise_and(img_green, img_blue)
                obj_img = cv2.bitwise_and(obj_img, img_red)
                # calculate moments of binary image
                M = cv2.moments(obj_img)
                if M["m00"] != 0.0:
                    # calculate x,y coordinate of center
                    cX = int(M["m10"] / M["m00"])-obj_img.shape[0]/2.0
                    cY = int(M["m01"] / M["m00"])-obj_img.shape[1]/2.0
                    # print(cX,cY)
                    self.targetfound = True
                else:
                    self.targetfound = False

                # plt.imshow(obj_img, cmap = 'gray')
                # plt.draw()
                # plt.pause(0.001)
            rate.sleep()
        # plt.close()

