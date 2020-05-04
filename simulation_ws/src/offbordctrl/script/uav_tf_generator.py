#!/usr/bin/env python

import rospy
import tf
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped, TransformStamped

no_of_uavs = 4


def local_position_cb(msg,uavno):
    global br
    tf_data = TransformStamped()
    # tf_data.header = msg.header
    # tf_data.header.frame_id = 'map'
    # tf_data.child_frame_id = 'uav' + str(uavno)
    tf_data.transform.translation.x = msg.pose.position.x
    tf_data.transform.translation.y = msg.pose.position.y
    tf_data.transform.translation.z = msg.pose.position.z
    tf_data.transform.rotation = msg.pose.orientation
    # br.sendTransformMessage(tf_data)

    tf_data.header = msg.header
    tf_data.header.frame_id = 'map'
    tf_data.child_frame_id = 'uav' + str(uavno)+'_base_link'
    br.sendTransformMessage(tf_data)

def pointcloud_cb(msg,uavno):
    global br
    #print(msg.header)
    tf_data = TransformStamped()
    tf_data.header.seq = msg.header.seq
    tf_data.header.stamp = msg.header.stamp
    # tf_data.header.frame_id = 'uav' + str(uavno)
    tf_data.header.frame_id = 'uav' + str(uavno)+'_base_link'
    tf_data.child_frame_id = msg.header.frame_id #'uav' + str(self.uavno) +'_camera_frame'
    tf_data.transform.translation.x = 0.1
    tf_data.transform.translation.y = 0
    tf_data.transform.translation.z = 0
    tf_data.transform.rotation.x = -0.5
    tf_data.transform.rotation.y = 0.5
    tf_data.transform.rotation.z = -0.5
    tf_data.transform.rotation.w = 0.5
    br.sendTransformMessage(tf_data)

    


if __name__ == '__main__':
    rospy.init_node('uav_tf_generator', anonymous=True)
    br = tf.TransformBroadcaster()
    for i in range(no_of_uavs):
        rospy.Subscriber('/uav'+str(i)+'/mavros/local_position/pose', PoseStamped, callback=local_position_cb,callback_args=i)
        rospy.Subscriber('/uav'+str(i)+'/r200_ir/points', PointCloud2, callback=pointcloud_cb,callback_args=i)

    rospy.spin()