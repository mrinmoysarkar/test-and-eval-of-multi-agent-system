#!/usr/bin/env python

import rospy
import tf
from sensor_msgs.msg import PointCloud2, JointState
from geometry_msgs.msg import PoseStamped, TransformStamped
import rosgraph

no_of_uavs = 4
pos_buffer = {}
pcl_buffer = {}
counter = [0]*no_of_uavs

update_data = True

def jointstates_cb(msg, uavno):
    # publish_data(stamp=msg.header.stamp)
    pass

def local_position_cb(msg, uavno):
    global pos_buffer, update_data
    if update_data:
        pos_buffer[uavno] = msg
    

def pointcloud_cb(msg, uavno):
    global pcl_buffer, counter, no_of_uavs, update_data
    if update_data:
        pcl_buffer[uavno] = msg
        counter[uavno] = 1
    if sum(counter) == no_of_uavs:
        update_data = False
        counter = [0]*no_of_uavs

    #print(msg.header)
    # tf_data = TransformStamped()
    # tf_data.header.seq = msg.header.seq
    # tf_data.header.stamp = msg.header.stamp
    # tf_data.header.frame_id = 'uav' + str(uavno)+'_base_link'
    # tf_data.child_frame_id = msg.header.frame_id #'uav' + str(self.uavno) +'_camera_frame'
    # tf_data.transform.translation.x = 0.1
    # tf_data.transform.translation.y = 0
    # tf_data.transform.translation.z = 0
    # tf_data.transform.rotation.x = -0.5
    # tf_data.transform.rotation.y = 0.5
    # tf_data.transform.rotation.z = -0.5
    # tf_data.transform.rotation.w = 0.5
    # br.sendTransformMessage(tf_data)
    # # print(uavno, msg.header.stamp)
    # if uavno in pos_buffer:
    #     tf_data.header.frame_id = 'map'
    #     tf_data.child_frame_id = 'uav' + str(uavno)+'_base_link'
    #     tf_data.transform.translation.x = pos_buffer[uavno].pose.position.x
    #     tf_data.transform.translation.y = pos_buffer[uavno].pose.position.y
    #     tf_data.transform.translation.z = pos_buffer[uavno].pose.position.z
    #     tf_data.transform.rotation = pos_buffer[uavno].pose.orientation
    #     br.sendTransformMessage(tf_data)

def publish_data(stamp=None):
    global br, pos_buffer, pcl_buffer, pcl_publisher, no_of_uavs
    if stamp is None:
        stamp = rospy.Time(0)#.now()
    tf_data = TransformStamped()
    for i in range(no_of_uavs):
        if i in pos_buffer and i in pcl_buffer:
            pcl_buffer[i].header.stamp = stamp
            tf_data.header.seq = pcl_buffer[i].header.seq
            tf_data.header.stamp = pcl_buffer[i].header.stamp
            tf_data.header.frame_id = 'uav' + str(i)+'_base_link'
            tf_data.child_frame_id = pcl_buffer[i].header.frame_id 
            tf_data.transform.translation.x = 0.1
            tf_data.transform.translation.y = 0
            tf_data.transform.translation.z = 0
            tf_data.transform.rotation.x = -0.5
            tf_data.transform.rotation.y = 0.5
            tf_data.transform.rotation.z = -0.5
            tf_data.transform.rotation.w = 0.5
            br.sendTransformMessage(tf_data)
            
            tf_data.header.frame_id = 'map'
            tf_data.child_frame_id = 'uav' + str(i)+'_base_link'
            tf_data.transform.translation.x = pos_buffer[i].pose.position.x
            tf_data.transform.translation.y = pos_buffer[i].pose.position.y
            tf_data.transform.translation.z = pos_buffer[i].pose.position.z
            tf_data.transform.rotation = pos_buffer[i].pose.orientation
            br.sendTransformMessage(tf_data)

            tf_data.header.frame_id = 'uav' + str(i)+'_base_link'
            tf_data.child_frame_id = 'uav'+str(i)+'_base_link_inertia'
            tf_data.transform.translation.x = 0
            tf_data.transform.translation.y = 0
            tf_data.transform.translation.z = 0
            tf_data.transform.rotation.x = 0
            tf_data.transform.rotation.y = 0
            tf_data.transform.rotation.z = 0
            tf_data.transform.rotation.w = 1
            br.sendTransformMessage(tf_data)

            tf_data.header.frame_id = 'uav'+str(i)+'_base_link_inertia'
            tf_data.child_frame_id = 'uav'+str(i)+'_rotor_0'
            tf_data.transform.translation.x = 0.13
            tf_data.transform.translation.y = -0.22
            tf_data.transform.translation.z = 0.023
            tf_data.transform.rotation.x = 0
            tf_data.transform.rotation.y = 0
            tf_data.transform.rotation.z = 0
            tf_data.transform.rotation.w = 1
            br.sendTransformMessage(tf_data)

            tf_data.header.frame_id = 'uav'+str(i)+'_base_link_inertia'
            tf_data.child_frame_id = 'uav'+str(i)+'_rotor_1'
            tf_data.transform.translation.x = -0.13
            tf_data.transform.translation.y = 0.2
            tf_data.transform.translation.z = 0.023
            tf_data.transform.rotation.x = 0
            tf_data.transform.rotation.y = 0
            tf_data.transform.rotation.z = 0
            tf_data.transform.rotation.w = 1
            br.sendTransformMessage(tf_data)

            tf_data.header.frame_id = 'uav'+str(i)+'_base_link_inertia'
            tf_data.child_frame_id = 'uav'+str(i)+'_rotor_2'
            tf_data.transform.translation.x = 0.13
            tf_data.transform.translation.y = 0.22
            tf_data.transform.translation.z = 0.023
            tf_data.transform.rotation.x = 0
            tf_data.transform.rotation.y = 0
            tf_data.transform.rotation.z = 0
            tf_data.transform.rotation.w = 1
            br.sendTransformMessage(tf_data)

            tf_data.header.frame_id = 'uav'+str(i)+'_base_link_inertia'
            tf_data.child_frame_id = 'uav'+str(i)+'_rotor_3'
            tf_data.transform.translation.x = -0.13
            tf_data.transform.translation.y = -0.2
            tf_data.transform.translation.z = 0.023
            tf_data.transform.rotation.x = 0
            tf_data.transform.rotation.y = 0
            tf_data.transform.rotation.z = 0
            tf_data.transform.rotation.w = 1
            br.sendTransformMessage(tf_data)

            pcl_publisher[i].publish(pcl_buffer[i])



            
    


if __name__ == '__main__':
    rospy.init_node('uav_tf_generator', anonymous=True)
    br = tf.TransformBroadcaster(queue_size=100)
    pcl_publisher = []
    for i in range(no_of_uavs):
        rospy.Subscriber('/uav'+str(i)+'/mavros/local_position/pose', PoseStamped, callback=local_position_cb, callback_args=i)
        rospy.Subscriber('/uav'+str(i)+'/r200_ir/points', PointCloud2, callback=pointcloud_cb, callback_args=i)
        # rospy.Subscriber('/uav'+str(i)+'/joint_states', JointState, callback=jointstates_cb, callback_args=i)
        pcl_publisher.append(rospy.Publisher('/uav'+str(i)+'/points', PointCloud2,queue_size=10))

    # rospy.spin()
    while rosgraph.is_master_online():
        if not update_data:
            publish_data()
            update_data = True
        rospy.sleep(1)



  

   

   

  