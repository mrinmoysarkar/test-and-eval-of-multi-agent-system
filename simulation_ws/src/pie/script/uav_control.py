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
from geometry_msgs.msg import Pose, PoseStamped, Point, Vector3, TwistStamped, TransformStamped
from std_msgs.msg import Header, ColorRGBA, String
from mavros_msgs.srv import CommandBool, ParamGet, SetMode, CommandTOL, ParamSet
from math import radians, sin, cos
import time
import threading
from visualization_msgs.msg import Marker
from nav_msgs.msg import Path
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge, CvBridgeError
import matplotlib.pyplot as plt
from astar import astar
import pandas as pd
import tf
import rosgraph
import moveit_commander


class uavControl():
    def __init__(self, uav_num, env_path="",scenario_num=0):
        self.uav_num = uav_num
        self.env_path = env_path
        self.scenario_num = scenario_num
        self.current_state = None
        self.setpointPub = None
        self.stopThread = False
        self.direction = 0
        self.currentPosition = Pose()
        self.currentImage = None
        self.takeoff_flag = False
        self.land_flag = False
        self.path = Path()
        self.bridge_rgb = CvBridge()
        self.count = 0
        self.map = None
        self.targetfound = False
        self.br = tf.TransformBroadcaster()
        self.currentVel = [0,0,0,0]
        self.setpoint = [1,1,1]
        self.targetPos = PoseStamped()

        ns = 'uav'+str(self.uav_num)
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander(robot_description="/"+ns+"/robot_description",ns=ns)
        self.scene = moveit_commander.PlanningSceneInterface(ns=ns)
        self.group_name = ns+"_group"
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name,robot_description="/"+ns+"/robot_description",ns=ns)
        self.move_group.set_workspace([-55,-55,0,55,55,20]) #[minX, minY, minZ, maxX, maxY, maxZ] 
        self.move_group.set_planner_id("RRTConnect")
        self.move_group.set_num_planning_attempts(10)

        
        self.setpointPub = rospy.Publisher('/uav'+str(self.uav_num)+'/targetPos', PoseStamped, queue_size=100)
        # self.setpointPub = rospy.Publisher('/uav'+str(self.uav_num)+'/mavros/setpoint_position/local', PoseStamped, queue_size=100)
        self.setpointVelPub = rospy.Publisher('/uav'+str(self.uav_num)+'/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=100)
        # self.marker_publisher = rospy.Publisher('/uav'+str(self.uav_num)+'/visualization_marker', Marker, queue_size=5)
        self.path_pub = rospy.Publisher('/uav'+str(self.uav_num)+'/path', Path, queue_size=10)
        self.pointcloud_pub = rospy.Publisher('/uav'+str(self.uav_num)+'/point_cloud', PointCloud2, queue_size=10)
        self.dataLabel_pub =  rospy.Publisher('/uav'+str(self.uav_num)+'/data_label', String, queue_size=10)


        rospy.Subscriber('/uav'+str(self.uav_num)+'/mavros/state', State, self.state_cb)
        rospy.Subscriber('/uav'+str(self.uav_num)+'/mavros/local_position/pose', PoseStamped, self.local_position_cb)
        # rospy.Subscriber('/uav'+str(self.uav_num)+'/r200_ir/image_raw', Image, self.image_data_cb)
        rospy.Subscriber('/uav'+str(self.uav_num)+'/c920/image_raw', Image, self.image_data_cb)

        # rospy.Subscriber('/map',Image, self.map_cb)
        # rospy.Subscriber('/uav'+str(self.uav_num)+'/r200_ir/points', PointCloud2, self.pointcloud_cb)

    def pointcloud_cb(self, msg):
        # print(msg.header)
        # msg.header.frame_id = 'uav' + str(self.uav_num) +'_camera_frame'

        tf_data = TransformStamped()
        tf_data.header.seq = msg.header.seq
        tf_data.header.stamp = msg.header.stamp
        tf_data.header.frame_id = 'uav' + str(self.uav_num)
        tf_data.child_frame_id = msg.header.frame_id #'uav' + str(self.uav_num) +'_camera_frame'
        tf_data.transform.translation.x = 0.1
        tf_data.transform.translation.y = 0
        tf_data.transform.translation.z = 0
        tf_data.transform.rotation.x = -0.5
        tf_data.transform.rotation.y = 0.5
        tf_data.transform.rotation.z = -0.5
        tf_data.transform.rotation.w = 0.5

        # tf_data = TransformStamped()
        # tf_data.header = msg.header
        # tf_data.header.frame_id = 'world'
        # tf_data.child_frame_id = 'uav' + str(self.uav_num) +'_camera_frame'
        # tf_data.transform.translation.x = self.currentPosition.position.x+0.1
        # tf_data.transform.translation.y = self.currentPosition.position.y
        # tf_data.transform.translation.z = self.currentPosition.position.z
        # tf_data.transform.rotation = self.currentPosition.orientation

        if not self.stopThread:
            self.br.sendTransformMessage(tf_data)
            # self.pointcloud_pub.publish(msg)

    def map_cb(self, msg):
        self.map = self.bridge_rgb.imgmsg_to_cv2(msg, msg.encoding)

    def image_data_cb(self, msg):
        self.currentImage = self.bridge_rgb.imgmsg_to_cv2(msg, msg.encoding)
       
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
        # to visualize in rviz
         
        self.path.header = msg.header
        self.path.poses.append(msg)
        

        # tf_data = TransformStamped()
        # tf_data.header = msg.header
        # tf_data.header.frame_id = 'world'
        # tf_data.child_frame_id = 'uav' + str(self.uav_num)
        # tf_data.transform.translation.x = msg.pose.position.x
        # tf_data.transform.translation.y = msg.pose.position.y
        # tf_data.transform.translation.z = msg.pose.position.z
        # tf_data.transform.rotation = msg.pose.orientation
        # if not self.stopThread:
        # self.path_pub.publish(self.path)
        #     self.br.sendTransformMessage(tf_data)
        
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

    def getquaternion(self, roll, pitch, yaw): # argument in degree
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
            if self.uav_num==0:
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
            elif self.uav_num==1:
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
            elif self.uav_num==2:
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
            elif self.uav_num==3:
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

    def get_waypoints_in_circle(self,xc,yc,r,dtheta,start_ang=0):
        theta = []
        theta.append(start_ang)
        ang = (start_ang + dtheta)%360
        theta.append(ang)

        while ang != start_ang:
            # print(ang)
            ang = (ang + dtheta)%360
            theta.append(ang)
        theta = np.radians(theta)
        # theta = np.linspace(0,np.pi*2,2*np.pi//np.radians(dtheta))
        x = xc+r*np.cos(theta)
        y = yc+r*np.sin(theta)
        z = np.linspace(3,4,len(theta))
        return zip(x,y,z)

    def pos_ctl_loop(self):
        rate = rospy.Rate(20.0)
        while rosgraph.is_master_online() and not self.stopThread:
            self.setpointPub.publish(self.targetPos)
            rate.sleep()

    def planning_loop(self, scenario_num=0):
        scenario_file = self.env_path + 'scenario_' + str(self.scenario_num) + '_uav_' + str(self.uav_num) + '.csv'
        scenario_df =  pd.read_csv(scenario_file)
        # print(scenario_df.head())
        x0 = scenario_df['x0'].values
        y0 = scenario_df['y0'].values
        x1 = scenario_df['x1'].values
        y1 = scenario_df['y1'].values
        h_res = scenario_df['horizontal_res'].values
        v_res = scenario_df['vertical_res'].values
        # print(x0,y0,x1,y1)
        n1 = 2 if h_res == -1 else abs(x0-x1)/h_res
        n2 = 2 if v_res == -1 else abs(y0-y1)/v_res

        x_coordinate = np.linspace(x0, x1, n1)
        y_coordinate = np.linspace(y0, y1, n2)
        waypoints = []

        for i,y in enumerate(y_coordinate):
            if i%2==0:
                for x in x_coordinate:
                    waypoints.append((x[0],y[0]))
            else:
                for x in reversed(x_coordinate):
                    waypoints.append((x[0],y[0]))

        # print(waypoints)
        # print(self.get_waypoints_in_circle(0,0,1,10))

        self.dataLabel_pub.publish('Hold')
        rate = rospy.Rate(10.0)
        q = [0,0,0,1]
        q = self.getquaternion(0,0,0)

        # if self.uav_num==0:
        #     q = self.getquaternion(0,0,0)
        # elif self.uav_num==1:
        #     q = self.getquaternion(0,0,-90)
        # elif self.uav_num==2:
        #     q = self.getquaternion(0,0,90)
        # elif self.uav_num==3:
        #     q = self.getquaternion(0,0,180)
        
        self.targetPos.pose.position.x = self.currentPosition.position.x
        self.targetPos.pose.position.y = self.currentPosition.position.y
        self.targetPos.pose.position.z = 3
        self.targetPos.pose.orientation.x = q[0]
        self.targetPos.pose.orientation.y = q[1]
        self.targetPos.pose.orientation.z = q[2]
        self.targetPos.pose.orientation.w = q[3]

        self.setpointPub.publish(self.targetPos)

        # threading.Thread(target=self.pos_ctl_loop).start()
        threading.Thread(target=self.object_detection).start()

        previous_target_x = 0
        previous_target_y = 0

        previous_ball_target_x = 0
        previous_ball_target_y = 0

        start_state = self.robot.get_current_state()
        goal_state = self.robot.get_current_state()
        goal_state.multi_dof_joint_state.transforms[0].translation.x = -10.0
        goal_state.multi_dof_joint_state.transforms[0].translation.y = 10.0
        goal_state.multi_dof_joint_state.transforms[0].translation.z = 3.0
        goal_state.multi_dof_joint_state.transforms[0].rotation.x = 0.0
        goal_state.multi_dof_joint_state.transforms[0].rotation.y = 0.0
        goal_state.multi_dof_joint_state.transforms[0].rotation.z = 0.0
        goal_state.multi_dof_joint_state.transforms[0].rotation.w = 1.0

        if self.set_mode('OFFBOARD', 5) and self.set_arm(True, 5):
            # mode takeoff
            self.dataLabel_pub.publish('Takeoff')
            while self.getDistance() > 0.15:
                rate.sleep()
            # mode hover
            self.dataLabel_pub.publish('Hover')
            rospy.sleep(30)

            if self.scenario_num != 1:
                # mode search
                ctn = 0

                ignore_ptns = 1
                ignore_counter = 0

                for wp in waypoints:
                    # if ctn==5:
                    #     break
                    self.path_pub.publish(self.path)
                    ctn += 1
                    start_state = self.robot.get_current_state()
                    start_state.multi_dof_joint_state.transforms[0].translation.x = previous_target_x #self.currentPosition.position.x
                    start_state.multi_dof_joint_state.transforms[0].translation.y = previous_target_y #self.currentPosition.position.y
                    goal_state.multi_dof_joint_state.transforms[0].translation.x = wp[0]
                    goal_state.multi_dof_joint_state.transforms[0].translation.y = wp[1]

                    self.move_group.set_workspace([min(previous_target_x+5, previous_target_x-5, wp[0]+5, wp[0]-5),
                                                   min(previous_target_y+15, previous_target_y-15, wp[1]+15, wp[1]-15),
                                                   0,
                                                   max(previous_target_x+5, previous_target_x-5, wp[0]+5, wp[0]-5), 
                                                   max(previous_target_y+15, previous_target_y-15, wp[1]+15, wp[1]-15),
                                                   20])
                    

                    self.move_group.clear_pose_targets()
                    self.move_group.set_start_state(start_state)
                    self.move_group.set_joint_value_target(goal_state)
                    # t1 = time.time()
                    plan = self.move_group.plan()
                    # rospy.sleep(10)
                    # print('planning time: '+str(time.time()-t1)+'s')
                    if ignore_counter >= ignore_ptns:
                        ignore_ptns = 1
                        ignore_counter = 0

                    # for ptn in plan.multi_dof_joint_trajectory.points:
                    #     print("{:.2f},{:.2f}".format(ptn.transforms[0].translation.x,ptn.transforms[0].translation.y))

                    # print("*******************************")
                    for ptn in plan.multi_dof_joint_trajectory.points:
                        self.dataLabel_pub.publish('Search')
                        if ignore_counter < ignore_ptns:
                            ignore_counter += 1
                            continue
                        # print(self.currentPosition)
                        # if self.scenario_num == 4 or self.scenario_num == 5:
                        #     if abs(self.currentPosition.position.x-17)<5 and abs(self.currentPosition.position.y-0)<5 and abs(self.currentPosition.position.z-5)<9:
                        #         self.dataLabel_pub.publish('Obstacleavoid')
                        # print("{:.2f},{:.2f}".format(ptn.transforms[0].translation.x,ptn.transforms[0].translation.y))
                        self.targetPos.pose.position.x = ptn.transforms[0].translation.x
                        self.targetPos.pose.position.y = ptn.transforms[0].translation.y
                        self.targetPos.pose.position.z = 3.0
                        self.targetPos.pose.orientation.x = q[0]
                        self.targetPos.pose.orientation.y = q[1]
                        self.targetPos.pose.orientation.z = q[2]
                        self.targetPos.pose.orientation.w = q[3]
                        self.setpointPub.publish(self.targetPos)
                        t1 = time.time()
                        while self.getDistance() > 0.4 and (time.time()-t1 < .7):
                            rate.sleep()
                            if self.targetfound: 
                                # print("new ball found 0")
                                dx = self.currentPosition.position.x - previous_ball_target_x
                                dy = self.currentPosition.position.y - previous_ball_target_y
                                del_d =  (dx**2 + dy**2)**0.5
                                # print(del_d)
                                self.targetfound = False
                                if del_d > 10 or (previous_ball_target_x==0 and previous_ball_target_y == 0):
                                    ignore_ptns = 10
                                    ignore_counter = 0
                                    dx = wp[0] - previous_target_x
                                    dy = wp[1] - previous_target_y
                                    start_ang = int(np.degrees(np.arctan2(dy,dx)) + 360) % 360
                                    # print("new ball found 1")
                                    # print(start_ang)
                                    rospy.sleep(1)
                                    # print("circle center at {:.2f}, {:.2f}".format(self.currentPosition.position.x, self.currentPosition.position.y))
                                    previous_ball_target_x = self.currentPosition.position.x
                                    previous_ball_target_y = self.currentPosition.position.y
                                    self.dataLabel_pub.publish('Loiter')

                                    wps = self.get_waypoints_in_circle(self.currentPosition.position.x,
                                                                       self.currentPosition.position.y,
                                                                       3,
                                                                       10,
                                                                       start_ang)
                                    for wp1 in wps:
                                        self.targetPos.pose.position.x = wp1[0]
                                        self.targetPos.pose.position.y = wp1[1]
                                        self.targetPos.pose.position.z = wp1[2]#3
                                        self.targetPos.pose.orientation.x = q[0]
                                        self.targetPos.pose.orientation.y = q[1]
                                        self.targetPos.pose.orientation.z = q[2]
                                        self.targetPos.pose.orientation.w = q[3]
                                        self.setpointPub.publish(self.targetPos)
                                        t1 = time.time()
                                        while self.getDistance() > 0.4 and (time.time()-t1 < 3):
                                            rate.sleep()
                                    break
                            
                    
                    previous_target_x = wp[0]
                    previous_target_y = wp[1]

                    # rospy.sleep(10) 
                    # print("*****************************") 



            # mode land
            self.dataLabel_pub.publish('Land') 
            self.land(5)
            while self.current_state.armed:
                # print(self.current_state)
                rate.sleep()

            self.dataLabel_pub.publish('Hold')  
            # for i in range(10):
            #     rate.sleep()

            # self.set_arm(False, 1)
            print("landed safely :).")
            self.stopThread = True
            rate.sleep()
            rospy.sleep(10)
            self.dataLabel_pub.publish('Finished')
            rospy.sleep(5)

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

    def set_velocity(self,vel):
        self.currentVel = vel

    def control_loop(self):
        target_vel = 3
        rate = rospy.Rate(20.0)
        setpoint = TwistStamped()
        setpoint.header.stamp = rospy.Time.now()
        setpoint.twist.linear.x = 0
        setpoint.twist.linear.y = 0
        setpoint.twist.linear.z = 0.5
        setpoint.twist.angular.x = 0
        setpoint.twist.angular.y = 0
        setpoint.twist.angular.z = 0
        
        # start = (50, 60)
        # end = (10, 75)
        # path = astar(self.map, start, end)
        # # print(path)
        # for p in path:
        #     self.map[p] = 255
        #     if self.map[p] == 1:
        #         print('collision')
        # self.map[self.map==1] = 255
        # plt.imshow(self.map, cmap='gray')
        # plt.show()

        while not self.stopThread and rosgraph.is_master_online():
            if self.takeoff_flag:
                self.takeoff_flag = False
                if self.takeoff_state():
                    # self.hover_state()
                    # self.land_state()
                    # self.search_state()

                    # for p in path:
                    #     x = p[0]-50
                    #     y = p[1]-50
                    #     self.position_ctl_loop(x,y,4)
                    # print('path accomplished')

                    while not self.land_flag and rosgraph.is_master_online():
                        # if self.targetfound:
                        #     self.survey_state(self.currentPosition.position.x,self.currentPosition.position.y,3)
                        # if self.direction == 0:
                        #     setpoint.twist.linear.x = 0
                        #     setpoint.twist.linear.y = 0
                        #     setpoint.twist.linear.z = 0
                        # elif self.direction == 1:
                        #     setpoint.twist.linear.x = target_vel
                        # elif self.direction == -1:
                        #     setpoint.twist.linear.x = -target_vel
                        # elif self.direction == 2:
                        #     setpoint.twist.linear.y = target_vel
                        # elif self.direction == -2:
                        #     setpoint.twist.linear.y = -target_vel
                        # elif self.direction == 3:
                        #     setpoint.twist.linear.z = target_vel
                        # elif self.direction == -3:
                        #     setpoint.twist.linear.z = -target_vel
                        setpoint.twist.linear.x = self.currentVel[0]*target_vel
                        setpoint.twist.linear.y = self.currentVel[1]*target_vel
                        setpoint.twist.linear.z = self.currentVel[2]*target_vel
                        setpoint.twist.angular.z = self.currentVel[3]*0.5
                        self.setpointVelPub.publish(setpoint)
                        rate.sleep()
                    
                    if self.land_flag:
                        self.land_flag = False
                        self.land_state()

    def position_ctl_loop(self,target_x,target_y,target_z=None):
        if None is target_z:
            target_z = self.currentPosition.position.z 

        kp,ki,kd = 1.5,0.01,0.1
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
        # setpoint.twist.linear.x = 0
        # setpoint.twist.linear.y = 0
        # setpoint.twist.linear.z = 0
        # self.setpointVelPub.publish(setpoint)
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

    def takeoff_state(self):
        rate = rospy.Rate(20.0)
        setpoint = TwistStamped()
        setpoint.header.stamp = rospy.Time.now()
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
                if self.currentPosition.position.z < 5:
                    setpoint.twist.linear.z = 0.5
                else:
                    setpoint.twist.linear.z = 0.0
                    break

                self.setpointVelPub.publish(setpoint)
                rate.sleep()

            setpoint.twist.linear.z = 0
            self.setpointVelPub.publish(setpoint)
            return True
        return False

    def hover_state(self):
        x = self.currentPosition.position.x
        y = self.currentPosition.position.y
        z = self.currentPosition.position.z
        t1 = time.time()
        while time.time()-t1 < 20:
            self.position_ctl_loop(x,y,z)

    def land_state(self):
        rate = rospy.Rate(20.0)
        setpoint = TwistStamped()
        setpoint.header.stamp = rospy.Time.now()
        setpoint.twist.linear.x = 0
        setpoint.twist.linear.y = 0
        setpoint.twist.linear.z = -0.5
        setpoint.twist.angular.x = 0
        setpoint.twist.angular.y = 0
        setpoint.twist.angular.z = 0
        while not self.stopThread:
            if self.currentPosition.position.z < 0.1:
                break
            self.setpointVelPub.publish(setpoint)
            rate.sleep()
        self.set_mode('MANUAL', 5)
        rospy.sleep(5)
        self.set_arm(False, 5)

    def search_state(self):
        dirpath = os.getcwd()
        path = dirpath+'/ros-intel-uav-rpeo/simulation_ws/src/pie/script/search_boundary.csv'
        boundary = pd.read_csv(path)
        boundary = boundary[boundary['uav_no']==self.uav_num]
        x1 = boundary['x1'].values
        y1 = boundary['y1'].values
        x2 = boundary['x2'].values
        y2 = boundary['y2'].values
        x1 = x1[0]
        y1 = y1[0]
        x2 = x2[0]
        y2 = y2[0]
        delta = 5
        d = 0
        while not self.stopThread:
            for y in range(y1,y2,delta):
                startx = x1+50
                starty = y+50
                endx = x1+x2-1+50
                endy = y+50

                start = (startx, starty)
                end = (endx, endy)
                path = astar(self.map, start, end)
                x1 = x1+x2-1
                x2 = -x2
                for p in path:
                    x = p[0]-50
                    y = p[1]-50
                    self.position_ctl_loop(x,y,5)

    def object_detection(self):
        rate = rospy.Rate(60.0)
        
        # plt.axis()
        # plt.ion()
        # plt.show()
        while rosgraph.is_master_online() and not self.stopThread:
            if None is not self.currentImage and not self.targetfound:
                lower = np.array([240,0,0], dtype = "uint8")
                upper = np.array([255,10,10], dtype = "uint8")
                mask = cv2.inRange(self.currentImage, lower, upper)
                if np.sum(mask)>0:
                    M = cv2.moments(mask)
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                    
                    d = ((mask.shape[0]/2-cY)**2+(mask.shape[1]/2-cX)**2)**.5
                    # print(d)
                    if d < 300:
                        # print("center of ball in image: {}, {}".format(cX,cY))
                        # print(mask.shape)
                        self.targetfound = True
                # else:
                #     self.targetfound = False

                # _,img_red = cv2.threshold(self.currentImage[:,:,0], 200, 255, cv2.THRESH_BINARY)
                # _,img_green = cv2.threshold(self.currentImage[:,:,1], 200, 255, cv2.THRESH_BINARY_INV)
                # _,img_blue = cv2.threshold(self.currentImage[:,:,2], 200, 255, cv2.THRESH_BINARY_INV)
                # # img_green_inv = cv2.bitwise_not(img_green)
                # # img_blue_inv = cv2.bitwise_not(img_blue)
                # obj_img = cv2.bitwise_and(img_green, img_blue)
                # obj_img = cv2.bitwise_and(obj_img, img_red)
                # # calculate moments of binary image
                # M = cv2.moments(obj_img)
                # if M["m00"] != 0.0:
                #     # calculate x,y coordinate of center
                #     cX = int(M["m10"] / M["m00"])-obj_img.shape[0]/2.0
                #     cY = int(M["m01"] / M["m00"])-obj_img.shape[1]/2.0
                #     # print(cX,cY)
                #     self.targetfound = True
                # else:
                #     self.targetfound = False

                # # plt.imshow(obj_img, cmap = 'gray')
                # # plt.draw()
                # # plt.pause(0.001)
            rate.sleep()
        # plt.close()

