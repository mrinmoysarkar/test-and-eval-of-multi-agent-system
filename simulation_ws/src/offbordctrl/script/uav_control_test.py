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

noofUav = 1
current_uav = 0
uavs = []
bridge_rgb = CvBridge()
count = 0

# 384 pixel is equivalent to 100m

def joy_controll_cb(msg):
    global uavs, current_uav, noofUav
    if 1 == msg.buttons[6]:
        current_uav += 1
        current_uav %= noofUav
        print('current UAV no. is ', current_uav)
    elif 1 == msg.buttons[4]: # takeoff
        uavs[current_uav].takeoff()
    elif 1 == msg.buttons[5]: # land
        uavs[current_uav].land_custom()
    elif 1 == msg.buttons[7]: # disarm
        uavs[current_uav].disarm()
    elif 1 == msg.buttons[3]: # go forward
        uavs[current_uav].go(1)
    elif 1 == msg.buttons[1]: # go backward
        uavs[current_uav].go(-1)
    elif 1 == msg.buttons[0]: # go left
        uavs[current_uav].go(2)
    elif 1 == msg.buttons[2]: # go right
        uavs[current_uav].go(-2)
    elif 1.0 == msg.axes[5]: # go up
        uavs[current_uav].go(3)
    elif -1.0 == msg.axes[5]: # go down
        uavs[current_uav].go(-3)
    else: #hover
        uavs[current_uav].go(0)

def map_image_cb(msg):
    global count, bridge_rgb
    if count==0:
        cv_image = bridge_rgb.imgmsg_to_cv2(msg, msg.encoding)
        # cv_image = cv2.cvtColor(cv_image,cv2.COLOR_RGB2GRAY)
        cv_image = cv_image[48:432,128:512,:]
        _,img_red = cv2.threshold(cv_image[:,:,0], 200, 255, cv2.THRESH_BINARY_INV)
        _,img_green = cv2.threshold(cv_image[:,:,1], 200, 255, cv2.THRESH_BINARY_INV)
        _,img_blue = cv2.threshold(cv_image[:,:,2], 200, 255, cv2.THRESH_BINARY)
        # img_green_inv = cv2.bitwise_not(img_green)
        # img_blue_inv = cv2.bitwise_not(img_blue)
        obj_img = cv2.bitwise_and(img_green, img_blue)
        obj_img = cv2.bitwise_and(obj_img, img_red)
        plt.imshow(obj_img,cmap='gray')
        plt.show()

        _, contours, _ = cv2.findContours(obj_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
            # calculate moments for each contour
            M = cv2.moments(c)
            if M["m00"] != 0.0:
               # calculate x,y coordinate of center
               cX = int(M["m10"] / M["m00"])
               cY = int(M["m01"] / M["m00"])
               print(cX,cY)

    count = 1

if __name__ == '__main__':
    rospy.init_node('hover_and_land_scenario_uav', anonymous=True)
    rospy.Subscriber('/joy', Joy, joy_controll_cb)
    rospy.Subscriber('/sensor_stand/r200_ir/image_raw', Image, map_image_cb)
    # for i in range(noofUav):
    #     uavs.append(uavControl(i))
    #     rospy.sleep(5)
        # threading.Thread(target=uavs[i].control_loop).start()
        # threading.Thread(target=uavs[i].object_detection).start()
    # uavs[0].set_param(param_id='NAV_RCL_ACT', param_val=0, timeout=5) 
    # uavs[0].set_param(param_id='COM_OBL_RC_ACT', param_val=2, timeout=5)
    # uavs[0].get_param(param_id='COM_OBL_RC_ACT', timeout=5)

    
    while not rospy.is_shutdown():
        try:
            # uavNo = raw_input('input uav no 0 to '+str(noofUav-1)+': ')
            # uavNo = 0 #int(uavNo)
            cmd = raw_input("command: \"q\" for exit the main loop: ")
            if cmd=='q':
                # for i in range(noofUav):
                #     uavs[i].stopThread = True
                # rospy.sleep(10)
                break
        except Exception as e:
            pass
