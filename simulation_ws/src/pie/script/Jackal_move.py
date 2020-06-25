#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import threading
import numpy as np
import time
import tf
from geometry_msgs.msg import TransformStamped, Point, Pose, Quaternion, Twist, Vector3, PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import degrees, atan2, sqrt, sin, cos, pi
from base_uav_control import uavControl


stopThread = False
pub = None


X = np.zeros((3,2))
x_max = 1.73
x_min = 0.15
y_max = 9.5
y_min = 0
x_data = []
y_data = []
start_x = 0
start_y = 0
counter = 0

t_start = time.time()
drone1SetPointPub = None

yaw = 0  

def odomRead(msg):
    global X 
    global yaw, x_data,y_data
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    X[0,0] = x 
    X[0,1] = y
    x_data.append(x)
    y_data.append(y)
    orientation_list = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    yaw = degrees(yaw)
    print(msg.pose.pose)

def move(pub,x,y,z):
    twist = Twist()
    speed = 0.3  
    twist.linear.x = x*speed 
    twist.linear.y = y*speed
    twist.linear.z = z*speed
    twist.angular.x = 0 
    twist.angular.y = 0 
    twist.angular.z = 0
    pub.publish(twist)

def rotate(pub,deg,leftorright):
    global yaw, stopThread
    twist = Twist()
    speed = 0.3  
    twist.linear.x = 0
    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.x = 0 
    twist.angular.y = 0 
    if leftorright:
        twist.angular.z = speed
    else:
        twist.angular.z = -speed
    last_yaw = yaw
    rate = rospy.Rate(10.0)
    move(pub,0,0,0)
    while abs(yaw-last_yaw) < deg and not stopThread:
        pub.publish(twist)
        rate.sleep()
    move(pub,0,0,0) 
    

def rotateZero(pub):
    global yaw, stopThread
    rate = rospy.Rate(10.0)
    twist = Twist()
    speed = 0.3  
    twist.linear.x = 0
    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.x = 0 
    twist.angular.y = 0 
    if yaw>0:
        twist.angular.z = -speed
    else:
        twist.angular.z = speed
    
    
    move(pub,0,0,0)
    while abs(yaw) > 2 and not stopThread:
        pub.publish(twist)
        rate.sleep()
    move(pub,0,0,0)

def rotateabsoluteAngle(pub,ang):
    global yaw, stopThread
    rate = rospy.Rate(10.0)
    twist = Twist()
    speed = 0.3  
    twist.linear.x = 0
    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.x = 0 
    twist.angular.y = 0 

    dl = ang - yaw

    if (ang >=0 and yaw >=0) or (ang < 0 and yaw < 0):
        if dl >= 0: 
            twist.angular.z = speed #ccw
        else:
            twist.angular.z = -speed #cw
    elif (ang >=0 and yaw <=0):
        if dl <= 180: 
            twist.angular.z = speed #CCW
        else:
            twist.angular.z = -speed #CW
    elif (ang <= 0 and yaw >=0):
        if dl <= -180: 
            twist.angular.z = speed
        else:
            twist.angular.z = -speed
    
    
    move(pub,0,0,0)
    while abs(ang-yaw) > 2 and not stopThread:
        pub.publish(twist)
        rate.sleep()
    move(pub,0,0,0) 

def goto_2dLocation(pub,x,y):
    global X, yaw, stopThread
    #calculate desired angle
    dy = y-X[0,1]
    dx = x-X[0,0]
    dang = degrees(atan2(dy,dx))
    rotateabsoluteAngle(pub,dang)
    d = sqrt(dy**2+dx**2)

    rate = rospy.Rate(10.0)
    twist = Twist()
    speed = 0.3  
    twist.linear.x = 0.3
    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.x = 0 
    twist.angular.y = 0
    twist.angular.z = 0
    kp_ang = 0.05
    kp_dis = 0.1
    counter = 0
    while d > 0.15 and counter<100 and not stopThread:
        dy = y-X[0,1]
        dx = x-X[0,0]
        d = sqrt(dy**2+dx**2)
        twist.linear.x = max(min(kp_dis*d, 0.7),0.2)
        twist.angular.z = min(kp_ang*(dang-yaw), 0.3)
        pub.publish(twist)
        rate.sleep()
        counter += 1
    move(pub,0,0,0)

def loop():
    global stopThread
    global pub
    global X
    global x_max
    global x_min
    global y_max, y_min
    global start_x, start_y
    start_x = X[0,0]
    stary_y = X[0,1]
   
    print("x:", start_x)
    print("y:", start_y)

    # This is what the Jackal will be performing
    rate = rospy.Rate(10.0)
    rotateZero(pub)
    rospy.sleep(1)
    goto_2dLocation(pub,4,0)
    rospy.sleep(1)
    goto_2dLocation(pub,4,2)
    rospy.sleep(1)
    goto_2dLocation(pub,2,2)
    rospy.sleep(1)
    goto_2dLocation(pub,2,0)
    rospy.sleep(1)
    rotateZero(pub)
    rospy.sleep(1)
    goto_2dLocation(pub,1,1)
    rospy.sleep(1)
    print("stopping the loop")



if __name__ == '__main__':          
    try:
        rospy.init_node('Jackal_move', anonymous=True)

        # code for check jackal
        rospy.Subscriber('/jackal1/odometry/global_filtered',Odometry,odomRead)
        pub  = rospy.Publisher("/jackal1/jackal_velocity_controller/cmd_vel", Twist, queue_size=10)  
        loop()

        # while not rospy.is_shutdown():

        #     cmd = raw_input("command: \"q\" exit program: ")
        #     if cmd=='q':
        #         stopThread = True
        #         rospy.sleep(5)
        #         break

        # code for check uav
        # uav0 = uavControl(0)
        # rospy.sleep(5)
        # rate = rospy.Rate(20.0)
        # q = uav0.getquaternion(0,0,0)
        # setpoint = PoseStamped()
        # setpoint.pose.position.x = uav0.currentPosition.position.x
        # setpoint.pose.position.y = uav0.currentPosition.position.y
        # setpoint.pose.position.z = 1.5
        # # print(setpoint)
        # setpoint.pose.orientation.x = q[0]
        # setpoint.pose.orientation.y = q[1]
        # setpoint.pose.orientation.z = q[2]
        # setpoint.pose.orientation.w = q[3]
        # for i in range(100):
        #     uav0.setpointPub.publish(setpoint)
        #     rate.sleep()


        # if uav0.set_mode('OFFBOARD', 5) and uav0.set_arm(True, 5):
        #     t1 = time.time()
        #     while (time.time()-t1) < 30:
        #         uav0.setpointPub.publish(setpoint)
        #         rate.sleep()

        #     uav0.land(5)
        #     rospy.sleep(10)
        #     print("landed safely :).")

    except rospy.ROSInterruptException:
        pass