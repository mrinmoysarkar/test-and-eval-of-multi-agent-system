#!/usr/bin/env python

import rospy
import tf
from sensor_msgs.msg import PointCloud2, JointState
from geometry_msgs.msg import PoseStamped, TransformStamped
import rosgraph

pos_buffer = None
pcl_buffer = None


update_data = 0

def jointstates_cb(msg, uavno):
    # publish_data(stamp=msg.header.stamp)
    pass

def local_position_cb(msg, uavno):
    global pos_buffer, update_data
    if update_data == 0:
        pos_buffer = msg
        update_data += 1
    

def pointcloud_cb(msg, uavno):
    global pcl_buffer, update_data
    if update_data == 1:
        pcl_buffer = msg
        update_data += 1


def publish_data(uav_num, stamp=None):
    global br, pos_buffer, pcl_buffer, pcl_publisher
    if stamp is None:
        stamp = rospy.Time(0)#.now()
    tf_data = TransformStamped()
    
    pcl_buffer.header.stamp = stamp
    tf_data.header.seq = pcl_buffer.header.seq
    tf_data.header.stamp = pcl_buffer.header.stamp
    tf_data.header.frame_id = 'uav' + str(uav_num)+'_base_link'
    tf_data.child_frame_id = pcl_buffer.header.frame_id 
    tf_data.transform.translation.x = 0.1
    tf_data.transform.translation.y = 0
    tf_data.transform.translation.z = 0
    tf_data.transform.rotation.x = -0.5
    tf_data.transform.rotation.y = 0.5
    tf_data.transform.rotation.z = -0.5
    tf_data.transform.rotation.w = 0.5
    br.sendTransformMessage(tf_data)
            
    tf_data.header.frame_id = 'map'
    tf_data.child_frame_id = 'uav' + str(uav_num)+'_base_link'
    tf_data.transform.translation.x = pos_buffer.pose.position.x
    tf_data.transform.translation.y = pos_buffer.pose.position.y
    tf_data.transform.translation.z = pos_buffer.pose.position.z
    tf_data.transform.rotation = pos_buffer.pose.orientation
    br.sendTransformMessage(tf_data)

    tf_data.header.frame_id = 'uav' + str(uav_num)+'_base_link'
    tf_data.child_frame_id = 'uav'+str(uav_num)+'_base_link_inertia'
    tf_data.transform.translation.x = 0
    tf_data.transform.translation.y = 0
    tf_data.transform.translation.z = 0
    tf_data.transform.rotation.x = 0
    tf_data.transform.rotation.y = 0
    tf_data.transform.rotation.z = 0
    tf_data.transform.rotation.w = 1
    br.sendTransformMessage(tf_data)

    tf_data.header.frame_id = 'uav'+str(uav_num)+'_base_link_inertia'
    tf_data.child_frame_id = 'uav'+str(uav_num)+'_rotor_0'
    tf_data.transform.translation.x = 0.13
    tf_data.transform.translation.y = -0.22
    tf_data.transform.translation.z = 0.023
    tf_data.transform.rotation.x = 0
    tf_data.transform.rotation.y = 0
    tf_data.transform.rotation.z = 0
    tf_data.transform.rotation.w = 1
    br.sendTransformMessage(tf_data)

    tf_data.header.frame_id = 'uav'+str(uav_num)+'_base_link_inertia'
    tf_data.child_frame_id = 'uav'+str(uav_num)+'_rotor_1'
    tf_data.transform.translation.x = -0.13
    tf_data.transform.translation.y = 0.2
    tf_data.transform.translation.z = 0.023
    tf_data.transform.rotation.x = 0
    tf_data.transform.rotation.y = 0
    tf_data.transform.rotation.z = 0
    tf_data.transform.rotation.w = 1
    br.sendTransformMessage(tf_data)

    tf_data.header.frame_id = 'uav'+str(uav_num)+'_base_link_inertia'
    tf_data.child_frame_id = 'uav'+str(uav_num)+'_rotor_2'
    tf_data.transform.translation.x = 0.13
    tf_data.transform.translation.y = 0.22
    tf_data.transform.translation.z = 0.023
    tf_data.transform.rotation.x = 0
    tf_data.transform.rotation.y = 0
    tf_data.transform.rotation.z = 0
    tf_data.transform.rotation.w = 1
    br.sendTransformMessage(tf_data)

    tf_data.header.frame_id = 'uav'+str(uav_num)+'_base_link_inertia'
    tf_data.child_frame_id = 'uav'+str(uav_num)+'_rotor_3'
    tf_data.transform.translation.x = -0.13
    tf_data.transform.translation.y = -0.2
    tf_data.transform.translation.z = 0.023
    tf_data.transform.rotation.x = 0
    tf_data.transform.rotation.y = 0
    tf_data.transform.rotation.z = 0
    tf_data.transform.rotation.w = 1
    br.sendTransformMessage(tf_data)

    pcl_publisher.publish(pcl_buffer)



            
    


if __name__ == '__main__':
    rospy.init_node('uav_tf_generator', anonymous=True)
    br = tf.TransformBroadcaster(queue_size=100)
    uav_num = rospy.get_param('~uav_num')
    
    rospy.Subscriber('/uav'+str(uav_num)+'/mavros/local_position/pose', PoseStamped, callback=local_position_cb, callback_args=uav_num)
    rospy.Subscriber('/uav'+str(uav_num)+'/r200_ir/points', PointCloud2, callback=pointcloud_cb, callback_args=uav_num)
    # rospy.Subscriber('/uav'+str(i)+'/joint_states', JointState, callback=jointstates_cb, callback_args=i)
    pcl_publisher = rospy.Publisher('/uav'+str(uav_num)+'/points', PointCloud2, queue_size=10)

    # rospy.spin()
    try:
        while rosgraph.is_master_online():
            if update_data == 2:
                publish_data(uav_num)
                update_data = 0
            rospy.sleep(1)
    except Exception as e:
        print(e)


  

   

   

  