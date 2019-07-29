import numpy as np
import cv2

import pygame
import pygame.camera
import time
import os
import sys

import imutils



class ShapeDetector:
    def __init__(self):
        pass

    def detect(self, c):
        # initialize the shape name and approximate the contour
        shape = "unidentified"
        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, 0.04 * peri, True)

        # if the shape is a triangle, it will have 3 vertices
        if len(approx) == 3:
            shape = "triangle"

		# if the shape has 4 vertices, it is either a square or
		# a rectangle
        elif len(approx) == 4:
            # compute the bounding box of the contour and use the
            # bounding box to compute the aspect ratio
            (x, y, w, h) = cv2.boundingRect(approx)
            print(w,h)
            ar = w / float(h)
            # a square will have an aspect ratio that is approximately
            # equal to one, otherwise, the shape is a rectangle
            shape = "square" if ar >= 0.95 and ar <= 1.05 else "rectangle"

		# if the shape is a pentagon, it will have 5 vertices
        elif len(approx) == 5:
		shape = "pentagon"

		# otherwise, we assume the shape is a circle
        else:
		shape = "circle"
 
		# return the name of the shape
	return shape


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
            print str(e)
            return False, None
        return True, yuv

    def read(self):
        ret, yuv = self.read_raw()
        if not ret:
            return ret, yuv
        bgr = cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_NV21)
        return ret, bgr


if __name__ == "__main__":
    while 1:
        os.system("sh cap_image.sh")
        time.sleep(1.0)
        filename = "Image-video2-640x480-0.yuv420"
        size = (480, 640)
        cap = VideoCaptureYUV(filename, size)
        ret, frame = cap.read()
        if ret:
            image = frame
            #cv2.imshow("frame", frame)
                
            resized = image#imutils.resize(image, width=300)
            ratio = image.shape[0] / float(resized.shape[0])
    
            # convert the resized image to grayscale, blur it slightly,
            # and threshold it
            gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
            blurred = cv2.GaussianBlur(gray, (5, 5), 0)
            thresh = cv2.threshold(blurred, 217, 255, cv2.THRESH_BINARY)[1]
            kernel = np.ones((20,20), np.uint8)
#            thresh = cv2.dilate(thresh,kernel,iterations=1)
	    thresh = thresh[20:460,20:620]
	    cv2.imshow("frame", image)
	    cv2.imshow("thresh",thresh)
	    cv2.imwrite("image.png",image)
            # find contours in the thresholded image and initialize the
            # shape detector
            cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cnts = cnts[0] if imutils.is_cv2() else cnts[1]
            sd = ShapeDetector()  
    
            # loop over the contours
            for c in cnts:
		arclen = cv2.arcLength(c,True)
		if arclen >=100:
		    print('target found')
                #shape = sd.detect(c)
                #if shape == 'square' or shape == 'rectangle':
                #    print(shape)
                	                	
                 
                 
                     # compute the center of the contour, then detect the name of the
                	# shape using only the contour
                	#M = cv2.moments(c)
                	#cX = int((M["m10"] / M["m00"]) * ratio)
                	#cY = int((M["m01"] / M["m00"]) * ratio)
                    # multiply the contour (x, y)-coordinates by the resize ratio,
                	# then draw the contours and the name of the shape on the image
#                	c = c.astype("float")
#                	c *= ratio
#                	c = c.astype("int")
#                	cv2.drawContours(image, [c], -1, (0, 255, 0), 2)
#                	cv2.putText(image, shape, (cX, cY), cv2.FONT_HERSHEY_SIMPLEX,
#                		0.5, (255, 255, 255), 2)
                
                	# show the output image
                	#cv2.imshow("Image", image)
            if cv2.waitKey(5) & 0xFF == ord('q'):
			break


#if __name__=='__main__':
#    img = cv2.imread('Image-video2-640x480-0.yuv420')
#    cv2.imshow('frame',img)
#    cap = cv2.VideoCapture('rtsp://127.0.0.1:8554/bottom')
#    flag = False
#    if cap.isOpened():
#        print('open')
#        flag=True
#    else:
#        print('close')
#        #cap.open()
#
#    while(flag):
#        # Capture frame-by-frame
#        ret, frame = cap.read()
#        #print(ret)
#        # Our operations on the frame come here
#        #time.sleep(1.0)
#        # Display the resulting frame
#        if ret:
#            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
#            cv2.imshow('frame',gray)
#        if cv2.waitKey(1) & 0xFF == ord('q'):
#            break
#
#    # When everything done, release the capture
#    cap.release()
#    cv2.destroyAllWindows()


#    pygame.camera.init()
#    pygame.camera.list_cameras()
#    cam = pygame.camera.Camera("/dev/video14", (640, 480))
#    cam.start()
#    time.sleep(1.0)  # You might need something higher in the beginning
#    img = cam.get_image()
#    pygame.image.save(img, "pygame.jpg")
#    cam.stop()
