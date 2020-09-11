# -*- coding: utf-8 -*-
"""
Created on Thu May 17 12:33:22 2018

@author: intel1
"""

import cv2
import numpy as np

image = cv2.imread("image.png")
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
cv2.imshow("gray",gray)
print(gray.shape)
for i in range(gray.shape[0]):
    for j in range(gray.shape[1]):
	if gray[i][j] < 256 and gray[i][j] > 210:
	    gray[i][j] = 255
	else:
	    gray[i][j] = 0
kernel = np.ones((20,20),np.uint8)
img_dilation = cv2.dilate(gray,kernel,iterations=1)
cv2.imshow("thresh",img_dilation)
cv2.waitKey(0)
