#!/usr/bin/env python

#import pcl
import rospy
from std_msgs.msg import String
import numpy as np
import cv2
#from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import PointCloud2
#import sensor_msgs.point_cloud2 as pc2

bridge_rgb = CvBridge()

def cameraDepth_pc_callback(data):
    print(data)

def cameraImage_rgb_callback(data):
    cv_image = bridge_rgb.imgmsg_to_cv2(data, "bgr8")
    flip_image=cv2.flip(cv_image, -1)
    cv2.imshow("Image window rgb", flip_image)
    cv2.waitKey(3)
    

if __name__ == '__main__':
    rospy.init_node('camera_test', anonymous=True)
    #rospy.Subscriber("/camera/color/image_raw", Image, cameraImage_rgb_callback)
    rospy.Subscriber("/camera/depth/points", PointCloud2, cameraDepth_pc_callback)
    rate = rospy.Rate(10) # 10hz
    counter = 0
    rospy.spin()
    


