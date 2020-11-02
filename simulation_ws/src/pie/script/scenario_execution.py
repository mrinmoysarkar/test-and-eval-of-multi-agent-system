#!/usr/bin/env python

from __future__ import print_function

import rospy
import time
import threading
from uav_control import uavControl
from sensor_msgs.msg import Joy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import matplotlib.pyplot as plt
import cv2
import numpy as np
from astar import main
import octomap
import inspect
from octomap_msgs.msg import Octomap
from pylib import get_path
from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray
import queue as Q
import rosgraph



if __name__ == '__main__':
    rospy.init_node('uav_scenario_execution_node', anonymous=True)
    uav_num = rospy.get_param('~uav_num')
    env_boundry_path = rospy.get_param('~env_boundry_path')
    scenario_num = rospy.get_param('~scenario_num')
    uav_controller = uavControl(uav_num, env_boundry_path, scenario_num)
    rospy.sleep(5)
    rospy.sleep(90*(uav_num))
    uav_controller.planning_loop()
    

    
    
   
