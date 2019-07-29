#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import numpy as np
from sensor_msgs.msg import PointCloud2
from tf2_msgs.msg import TFMessage
from math import cos, sin, radians, pi

tfPub = None
rot1 = [0, 0.707, 0, 0.707]#[cos(pi/4.0), 0, 0, sin(pi/4.0)]
rot2 = [0, 0, -0.707, 0.707]
# rotate in y 90 degree [0, 0, 0.707, 0.707]
# rotate in x 90 degree []
# rotate in z 90 degree []

def camera_tf_cb(msg):
    print(msg.header)

def tf_cb(msg):
    global tfPub
    transforms = msg.transforms
    # trans = []
    if len(transforms)>0:
        # trans.append(transforms[0])
        tform = None
        for transform in transforms:
        	if transform.child_frame_id == "vicon/quad1/quad1":
        		tform = transform
        		break

        if tform == None:
        	return

        tform.child_frame_id = "camera_depth_optical_frame"
        # print(transform.child_frame_id)
        # trans.append(transform)
        x=tform.transform.rotation.x  
        y=tform.transform.rotation.y 
        z=tform.transform.rotation.z 
        w=tform.transform.rotation.w 

        tform.transform.rotation.x = x*rot1[3] + y*rot1[2] - z*rot1[1] + w*rot1[0]  
        tform.transform.rotation.y = -x*rot1[2] + y*rot1[3] + z*rot1[0] + w*rot1[1]
        tform.transform.rotation.z = x*rot1[1] - y*rot1[0] + z*rot1[3] + w*rot1[2]
        tform.transform.rotation.w = -x*rot1[0] - y*rot1[1] - z*rot1[2] + w*rot1[3]

        x=tform.transform.rotation.x  
        y=tform.transform.rotation.y 
        z=tform.transform.rotation.z 
        w=tform.transform.rotation.w 

        tform.transform.rotation.x = x*rot2[3] + y*rot2[2] - z*rot2[1] + w*rot2[0]  
        tform.transform.rotation.y = -x*rot2[2] + y*rot2[3] + z*rot2[0] + w*rot2[1]
        tform.transform.rotation.z = x*rot2[1] - y*rot2[0] + z*rot2[3] + w*rot2[2]
        tform.transform.rotation.w = -x*rot2[0] - y*rot2[1] - z*rot2[2] + w*rot2[3]


        tform.header.stamp = rospy.Time.now()
        msg.transforms = [tform]

        tfPub.publish(msg)


if __name__=='__main__':
    rospy.init_node('camera_tf_node', anonymous=True)
    tfPub = rospy.Publisher("/tf",TFMessage,queue_size=100)
    # rospy.Subscriber("/camera/depth/points", PointCloud2 , camera_tf_cb)
    rospy.Subscriber("/tf", TFMessage , tf_cb)
    # rate=rospy.Rate(10)
    rospy.spin()
