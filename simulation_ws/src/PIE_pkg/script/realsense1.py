# -*- coding: utf-8 -*-
"""
Created on Wed May  9 15:51:29 2018

@author: intel1
"""

## setup logging
import matplotlib.pyplot as plt
import logging
logging.basicConfig(level = logging.INFO)

## import the package
import pyrealsense as pyrs

## start the service - also available as context manager
serv = pyrs.Service()

## create a device from device id and streams of interest
cam = serv.Device(device_id = 0, streams = [pyrs.stream.ColorStream(fps = 60)])

## retrieve 60 frames of data
for _ in range(2):
    cam.wait_for_frames()
    print(cam.color)
    plt.imshow(cam.color)
    plt.show()

## stop camera and service
cam.stop()
serv.stop()


