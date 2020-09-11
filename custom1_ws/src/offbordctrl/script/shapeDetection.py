#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import numpy as np
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

bridge_rgb = CvBridge()
#detector = cv2.SimpleBlobDetector()

def cameraImage_rgb_callback(data):
    cv_image = bridge_rgb.imgmsg_to_cv2(data, "bgr8")
    resize = cv2.resize(cv_image, (640,480))
    blur = cv2.GaussianBlur(resize, (5,5), 0)
    #laplacian = cv2.Laplacian(blur, cv2.CV_64F)
    #sboelx = cv2.Sobel(resize, cv2.CV_64F, 1, 0)
    #sboely = cv2.Sobel(resize, cv2.CV_64F, 0, 1)
    canny = cv2.Canny(blur,100,150)
    cv2.imshow("Edge Detection", canny)
    _ ,binary = cv2.threshold(canny, 127, 255, cv2.THRESH_BINARY_INV)
    cv2.imshow("Binary Image", binary)
    #flip_image=cv2.flip(cv_image, -1)
    #hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    #mask1 = cv2.inRange(hsv, lower_green, upper_green)
    #mask2 = cv2.erode(hsv, None, iterations=0)
    #mask3 = cv2.dilate(mask2, None, iterations=0)
    #mask4 = cv2.medianBlur(mask3,17)
    #mask5 = cv2.morphologyEx(mask3, cv2.MORPH_OPEN, None)
    #hsv = cv2.cvtColor(cv_image,cv2.COLOR_BGR2HSV)
    #keypoints = detector.detect(cv_image)
    #im_with_keypoints = cv2.drawKeypoints(cv_image, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    #cv2.imshow("HSV window", im_with_keypoints)
    cv2.waitKey(3)


if __name__ == '__main__':
    rospy.init_node('camera_test', anonymous=True)
    rospy.Subscriber("/camera/rgb/image_mono", Image, cameraImage_rgb_callback)
    #pub = rospy.Publisher('target/search/status', String, queue_size=100)
    rate = rospy.Rate(10) # 10hz
    counter = 0
    rospy.spin()
