#!/usr/bin/env python

import rospy
from geometry_msgs.msg import TransformStamped
from mavros_msgs.srv import CommandHome
from mavros_msgs.msg import HomePosition

if __name__ == '__main__':
    rospy.init_node('simulate_gps', anonymous=True)
    pub = rospy.Publisher('/mavros/mocap/tf', TransformStamped, queue_size=100)
    pub1  = rospy.Publisher('mavros/home_position/set', HomePosition, queue_size=100)
    hp = HomePosition()
    #hp.latitude = 0
    #hp.longitude = 0
    #hp.altitude = 0
    hp.position.x = 0
    hp.position.y = 0
    hp.position.z = 0
    hp.orientation.x=0
    hp.orientation.y=0
    hp.orientation.z=0
    hp.orientation.w=1

 
    rate = rospy.Rate(200) # 10hz
    data = TransformStamped()
    data.header.frame_id = "/world"
    data.child_frame_id = "/vicon/quad1/quad1" 
    data.transform.translation.x = 0.5
    data.transform.translation.y = 1.5
    data.transform.translation.z = 3.5
    data.transform.rotation.x = 0.0
    data.transform.rotation.y = 0.0
    data.transform.rotation.z = 0.383
    data.transform.rotation.w = 0.924
    
    seq = 0
    while not rospy.is_shutdown():
        data.header.seq = seq
        seq += 1
        data.header.stamp = rospy.Time.now() 
        pub.publish(data)
        #pub1.publish(hp)
        rate.sleep()
