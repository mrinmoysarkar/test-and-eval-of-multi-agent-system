from __future__ import print_function
import pyzbar.pyzbar as pyzbar
import numpy as np
import cv2
import os
import sys 




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
	return decodedObjects
   
def display(im, decodedObjects):
	for decodedObject in decodedObjects: 
		points = decodedObject.polygon
		if len(points) > 4 : 
			hull = cv2.convexHull(np.array([point for point in points], dtype=np.float32))
			hull = list(map(tuple, np.squeeze(hull)))
		else : 
			hull = points;
		n = len(hull)
		try:
			x=0
			y=0
			for i in range(n):
				x += hull[i].x
				y += hull[i].y
			x = x/n - 320
			y = y/n - 240
			if abs(x) < 20 and abs(y) < 20:
				print("0")
			elif x<0 and y>0:
				print("1")
			elif x<0 and y<0:
				print("2")
			elif x<0 and y>0:
				print("3")
			elif x>0 and y>0:
				print("4")
			print( x/n,y/n)
		except Exception as e:
			print(str(e))
		for j in range(0,n):
			cv2.line(im, hull[j], hull[ (j+1) % n], (255,0,0), 3)
	cv2.imshow("Results", im)
	cv2.waitKey(0)
 
if __name__ == '__main__':
	os.system("sh /home/intel1/ros_repo/ros_ws/src/offbordctrl/script/cap_image.sh")
	filename = "/home/intel1/ros_repo/ros_ws/src/offbordctrl/script/Image-video2-640x480-0.yuv420"
	size = (480, 640)
	cap = VideoCaptureYUV(filename, size)
	ret, im = cap.read()
	 
	decodedObjects = decode(im)
	#print(decodedObjects)
	display(im, decodedObjects)
