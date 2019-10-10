#!/usr/bin/env python


import rospy
from std_msgs.msg import String
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
#import pandas as pd
import threading
import numpy as np
from geometry_msgs.msg import TransformStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import degrees, atan2, sqrt
import serial
from time import sleep
from std_msgs.msg import Empty

def flight2(pub1):
    print("flight2 is called")
    
    
    #rospy.Subscriber("/ardrone/odometry",Odometry,odomCallback)
    #pub2 = rospy.Publisher("/ardrone/land", Empty, queue_size=10 )
    rate = rospy.Rate(10) # 10hz
    for i in range(10):
        pub1.publish(Empty())
        rate.sleep()

def move(pub,x,y,z):
    twist = Twist()
    speed = 0.1  
    twist.linear.x = x*speed 
    twist.linear.y = y*speed
    twist.linear.z = z*speed
    twist.angular.x = 0 
    twist.angular.y = 0 
    twist.angular.z = 0
    pub.publish(twist)


if __name__ == '__main__':          
    try:
		rospy.init_node('move_ARDrone_node', anonymous=True)

		pub1 = rospy.Publisher("/ardrone/takeoff", Empty, queue_size=10)
		pub2 = rospy.Publisher("/ardrone/land", Empty, queue_size=10)
		pub  = rospy.Publisher("/cmd_vel", Twist, queue_size=10) 
		flight2(pub1)
		rospy.sleep(5)
		rate = rospy.Rate(5.0)

		move(pub,0,0,1)
		rospy.sleep(2.5)
		move(pub,0,0,0)
		rospy.sleep(5)

    except rospy.ROSInterruptException:
        pass