# import the necessary packages
from collections import deque
#from imutils.video import VideoStream
#from matplotlib import pyplot as plt
#import numpy as np
#import argparse
import cv2
#import imutils
#import time
#import matplotlib.pyplot as plt
#import logging
import pyrealsense as pyrs
import sys
#logging.basicConfig(level = logging.INFO)

# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video", help="path to the (optional) video file")
ap.add_argument("-b", "--buffer", type=int, default=64, help="max buffer size")
args = vars(ap.parse_args())

## start the service - also available as context manager
serv = pyrs.Service()

# define the lower and upper boundaries of the "green"
# ball in the HSV color space, then initialize the

# list of tracked points
lower_green = np.array([29,86,6])
upper_green = np.array([64,255,255])

pts = deque(maxlen=args["buffer"])

## create a device from device id and streams of interest
camera = serv.Device(device_id = 0, streams = [pyrs.stream.ColorStream(name='color', width= 640, height= 480, fps= 60, color_format= 'rgb')])

# allow the camera or video file to warm up
time.sleep(2.0)

while True:   
# retrieve 60 frames of data
    camera.wait_for_frames()
    frame = camera.color
	# if we are viewing a video and we did not grab a frame,
	# then we have reached the end of the video
    if frame is None:
	        break
 
	# (resize the frame), blur it, and convert it to the HSV
	# color space
    blurred = cv2.GaussianBlur(frame, (5, 5), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    mask1 = cv2.inRange(hsv, lower_green, upper_green)
    mask2 = cv2.erode(mask1, None, iterations=0)
    mask3 = cv2.dilate(mask2, None, iterations=0)
    mask4 = cv2.medianBlur(mask3,17)
    mask5 = cv2.morphologyEx(mask4, cv2.MORPH_OPEN, None)
    
	# show the frame to our screen
    #cv2.imshow("old mask", mask)
    cv2.imshow('new mask', mask5)
    key = cv2.waitKey(1) & 0xFF
 
	# if the 'q' key is pressed, stop the loop
    if key == ord("q"):
		    break

# When everything done, release the capture   
print ('closing program')   

## stop camera and service
cam.stop()
serv.stop()
cv2.destroyAllWindows()
