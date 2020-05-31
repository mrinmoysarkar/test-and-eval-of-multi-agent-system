#!/usr/bin/env python

from __future__ import print_function
import rospy
import rosgraph
from geometry_msgs.msg import PoseStamped


def targetPos_cb(msg):
    global targetPos
    targetPos = msg


if __name__ == '__main__':
    rospy.init_node('targetPos_publisher', anonymous=True)
    uav_num = rospy.get_param('~uav_num')
    setpointPub = rospy.Publisher('/uav'+str(uav_num)+'/mavros/setpoint_position/local', PoseStamped, queue_size=100)
    rospy.Subscriber('/uav'+str(uav_num)+'/targetPos', PoseStamped, targetPos_cb)
    rate = rospy.Rate(20.0)
    targetPos = PoseStamped()
    while rosgraph.is_master_online():
        setpointPub.publish(targetPos)
        rate.sleep()