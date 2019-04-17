#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import numpy as np
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

bridge_rgb = CvBridge()
bridge_ir_left = CvBridge()
bridger_ir_right = CvBridge()
bridge_ir_depth = CvBridge()

def cameraImage_rgb_callback(data):
    cv_image = bridge_rgb.imgmsg_to_cv2(data, "bgr8")
    #flip_image=cv2.flip(cv_image, -1)
    cv2.imshow("Image window rgb", cv_image)
    cv2.waitKey(3)
    
def cameraImage_ir_left_callback(data):
    cv_image = bridge_ir_left.imgmsg_to_cv2(data, "8UC1")
    cv2.imshow("Image window ir left", cv_image)
    cv2.waitKey(3)

def cameraImage_ir_right_callback(data):
    cv_image = bridger_ir_right.imgmsg_to_cv2(data, "8UC1")
    cv2.imshow("Image window right", cv_image)
    cv2.waitKey(3)

def cameraImage_depth_callback(data):
    cv_image = bridge_ir_depth.imgmsg_to_cv2(data, "16UC1")
    cv2.imshow("Image window depth", cv_image)
    cv2.waitKey(3)

if __name__ == '__main__':
    rospy.init_node('camera_test', anonymous=True)
    rospy.Subscriber("/camera/color/image_raw", Image, cameraImage_rgb_callback)
    #rospy.Subscriber("/camera/depth/image_raw", Image, cameraImage_depth_callback)
    #rospy.Subscriber("/camera/ir2/image_raw", Image, cameraImage_ir_left_callback)
    #rospy.Subscriber("/camera/ir/image_raw", Image, cameraImage_ir_right_callback)
    #pub = rospy.Publisher('target/search/status', String, queue_size=100)
    rate = rospy.Rate(10) # 10hz
    counter = 0
    rospy.spin()

