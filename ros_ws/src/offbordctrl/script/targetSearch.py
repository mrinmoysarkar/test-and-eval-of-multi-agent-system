#!/usr/bin/env python

from __future__ import print_function
import pyzbar.pyzbar as pyzbar
import rospy
from std_msgs.msg import String
import numpy as np
import cv2
import pygame
import pygame.camera
import time
import os
import sys
import imutils


cameraStatus = "notdone"
pub = []
class VideoCaptureYUV:
    def __init__(self, filename, size):
        self.height, self.width = size
        self.frame_len = self.width * self.height * 3 / 2
        self.f = open(filename, 'rb')
        self.shape = (int(self.height*1.5), self.width)

    def read_raw(self):
        try:
            raw = self.f.read(self.frame_len)
            yuv = np.frombuffer(raw, dtype=np.uint8)
            yuv = yuv.reshape(self.shape)
        except Exception as e:
            print(str(e))
            return False, None
        return True, yuv

    def read(self):
        ret, yuv = self.read_raw()
        if not ret:
            return ret, yuv
        bgr = cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_NV21)
        return ret, bgr

def decode(im) : 
	decodedObjects = pyzbar.decode(im)
	for obj in decodedObjects:
            print('Type : ', obj.type)
	    print('Data : ', obj.data,'\n')
	    return obj.data
	return "nothing" #decodedObjects

def search():
    os.system("sh /home/intel1/ros_repo/ros_ws/src/offbordctrl/script/cap_image.sh")
    filename = "/home/intel1/ros_repo/ros_ws/src/offbordctrl/script/Image-video2-640x480-0.yuv420"
    size = (480, 640)
    cap = VideoCaptureYUV(filename, size)
    ret, frame = cap.read()
    if ret:
        image = frame
	decodedObjects = decode(image)
	if decodedObjects == "Target1":
	    print("Target1 found")			
	    return "found"
        #gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        #blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        #thresh = cv2.threshold(blurred, 195, 255, cv2.THRESH_BINARY)[1]
        #thresh = thresh[20:460,20:620]
        #cv2.imshow("thresh",thresh)
        #cv2.imshow("thresh",image)
        #cv2.waitKey(0)
        #cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        #cnts = cnts[0] if imutils.is_cv2() else cnts[1] 
        #for c in cnts:
		#	arclen = cv2.arcLength(c,True)
		#	if arclen >=100:
		#		print('target found')
		#		return "found"
    return "done"




def statuscallback(data):
    global cameraStatus
    global pub
    trajectorystatus = data.data
    print("trajectory status",trajectorystatus)
    if trajectorystatus=="hold":
	for i in range(5):
            status = search()
            rospy.loginfo(status)
	    if status == "found":
                break
	cameraStatus = status
        pub.publish(status)
        if status == "found":
	    return


def targetSearch():
    global cameraStatus
    global pub
    rospy.init_node('target', anonymous=True)
    rospy.Subscriber("pubTrajectory/currentStatus", String, statuscallback)
    pub = rospy.Publisher('target/search/status', String, queue_size=100)
    rate = rospy.Rate(10) # 10hz
    counter = 0
    rospy.spin()
   # while not rospy.is_shutdown():
        #counter += 1
	#pub.publish(cameraStatus)
	#if cameraStatus != "notdone":
	#    cameraStatus = "notdone"
        #rate.sleep()

if __name__ == '__main__':
    try:
        targetSearch()
    except rospy.ROSInterruptException:
        pass
