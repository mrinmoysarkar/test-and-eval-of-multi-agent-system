#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def targetSearch():
    pub = rospy.Publisher('target/search/status', String, queue_size=10)
    rospy.init_node('target', anonymous=True)
    rate = rospy.Rate(1) # 10hz
    counter = 0
    while not rospy.is_shutdown():
        counter += 1
        if counter == 5:
            status = 'done'
            rospy.loginfo(status)
            pub.publish(status)
            counter = 0
        rate.sleep()

if __name__ == '__main__':
    try:
        targetSearch()
    except rospy.ROSInterruptException:
        pass
