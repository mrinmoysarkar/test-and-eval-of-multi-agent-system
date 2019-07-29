#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import numpy as np
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import PointCloud2
import struct
import math
import thread
import time



bridge_rgb = CvBridge()
pcloud = PointCloud2()

def cameraImage_rgb_callback(data):
	cv_image = bridge_rgb.imgmsg_to_cv2(data, data.encoding)
	#flip_image=cv2.flip(cv_image, -1)
	#cv2.imshow("Image window rgb", cv_image)
	#cv2.waitKey(3)
	print(cv_image.shape)

def cameraPointcloud_callback(data):
	#print(data.is_dense)
	print(data.height)
	print(data.width)
	#print(data.fields)
	#print(data.point_step)
	#print(data.row_step)
	#print(data.is_bigendian)
	offset = 0
	for j in range(data.height):
		for i in range(data.width):
			[x,y,z]=struct.unpack_from('fff',data.data,offset=offset)
			offset += data.point_step
			if not math.isnan(x):
				#print(x,y,z)
				pass
	#print('in call back')

def process_pointcloud_data():
	while True:
		print('in thread')
		time.sleep(5)
	pass

if __name__ == '__main__':
	rospy.init_node('camera_hole_detection', anonymous=True)
	rospy.Subscriber("/camera/depth/image_raw", Image, cameraImage_rgb_callback)
	rospy.Subscriber("/camera/depth/points", PointCloud2, cameraPointcloud_callback)
	#try:
   	#	thread.start_new_thread( process_pointcloud_data, ( ) )
	#except:
   	#	print "Error: unable to start thread"
	rospy.spin()
	

    #rospy.Subscriber("/camera/ir2/image_raw", Image, cameraImage_ir_left_callback)
    #rospy.Subscriber("/camera/ir/image_raw", Image, cameraImage_ir_right_callback)
    #pub = rospy.Publisher('target/search/status', String, queue_size=100)
	#rate = rospy.Rate(10) # 10hz
	#counter = 0
	
	
