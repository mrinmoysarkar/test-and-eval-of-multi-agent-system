import pygame
import pygame.camera
import cv2
import numpy as np
import base64
import io
from imageio import imread

def from_base64(base64_data):
    nparr = np.fromstring(base64_data.decode('base64'), np.uint8)
    print(nparr)
    return np.reshape(nparr,(640,480))


if __name__=="__main__":
    pygame.camera.init()
    cam = pygame.camera.Camera("/dev/video14",(480,640))
    cam.start()
    img = cam.get_image()
    pygame.image.save(img,"filename.png")
    im =cv2.imread("filename.png")
    cv2.imshow("Results", im)
    cv2.waitKey(0)
