#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import numpy as np
from geometry_msgs.msg import TransformStamped
from scipy.spatial import distance
from copy import deepcopy
import pandas as pd
from matplotlib import pyplot as plt
from sklearn.cluster import KMeans
from std_msgs.msg import Float32MultiArray
import threading
import time


X = pd.DataFrame(columns=['time', 'X', 'Y', 'Z', 'SOURCE'])
stopThread = False


def jackal1_cb(msg):
    global X 
    x = msg.transform.translation.x
    y = msg.transform.translation.y
    z = msg.transform.translation.z
    t = time.time()
    X = X.append({'time':t, 'X':x, 'Y':y, 'Z':z, 'SOURCE':'jackal1'}, ignore_index=True)


def jackal2_cb(msg):
    global X 
    x = msg.transform.translation.x
    y = msg.transform.translation.y
    z = msg.transform.translation.z
    t = time.time()
    X = X.append({'time':t, 'X': x, 'Y':y, 'Z':z, 'SOURCE':'jackal2'}, ignore_index=True)
    

def custom2_cb(msg):
    global X 
    x = msg.transform.translation.x
    y = msg.transform.translation.y
    z = msg.transform.translation.z
    t = time.time()
    X = X.append({'time':t, 'X': x, 'Y':y, 'Z':z, 'SOURCE':'custom2'}, ignore_index=True)
   

def intel1_cb(msg):
    global X 
    x = msg.transform.translation.x
    y = msg.transform.translation.y
    z = msg.transform.translation.z
    t = time.time()
    X = X.append({'time':t, 'X': x, 'Y':y, 'Z':z, 'SOURCE':'intel1'}, ignore_index=True)
    
   
def loop():
    global stopThread
    rospy.Subscriber("/vicon/jackal1/jackal1", TransformStamped, jackal1_cb)
    rospy.Subscriber("/vicon/jackal2/jackal2", TransformStamped, jackal2_cb)
    rospy.Subscriber("/custom2/mavros/mocap/tf", TransformStamped, custom2_cb)
    rospy.Subscriber("/vicon/intel1/intel1", TransformStamped, intel1_cb)
    while not rospy.is_shutdown() and not stopThread:
        rospy.sleep(1)



if __name__ == '__main__':
    rospy.init_node('log_trajectory', anonymous=True)
    
    threading.Thread(target=loop).start()
    while True:
        cmd = raw_input("\"q\" for exit: ")
        if cmd=='q':
            stopThread = True
            X.to_csv("/home/inteldrone/ros-intel-uav-rpeo/ubuntu_pc_ws/src/offbordctrl/logged_data/trajectory_"+str(time.ctime(time.time()))+".csv")
            rospy.sleep(2)
            break