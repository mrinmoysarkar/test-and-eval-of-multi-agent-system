#!/usr/bin/env python

from __future__ import print_function

import rospy
from octomap_msgs.msg import Octomap
import rosgraph
from moveit_msgs.msg import PlanningScene, PlanningSceneWorld







def octomap_cb(msg):
    global pScene, pwScene
    pwScene.octomap.header.stamp = rospy.Time.now()
    pwScene.octomap.header.frame_id = 'map'
    pwScene.octomap.octomap = msg
    pwScene.octomap.origin.position.x=0
    pwScene.octomap.origin.orientation.w=1
    pScene.world = pwScene
    pScene.is_diff = True



if __name__ == '__main__':
    rospy.init_node("octomap_loader", anonymous=True)
    uav_num = rospy.get_param('~uav_num')
    rospy.Subscriber('/octomap_binary', Octomap, octomap_cb, queue_size=10)
    monitored_planning_scene_pub = rospy.Publisher("/uav"+str(uav_num)+"/move_group/monitored_planning_scene", PlanningScene, queue_size=10)
    planning_scene_pub = rospy.Publisher("/uav"+str(uav_num)+"/planning_scene", PlanningScene, queue_size=10)
    # planning_scene_world_pub = rospy.Publisher("/uav"+str(uav_num)+"/planning_scene_world", PlanningSceneWorld, queue_size=10)

    pScene = PlanningScene()
    pwScene = PlanningSceneWorld()
    rate = rospy.Rate(0.5)

    while rosgraph.is_master_online():
        monitored_planning_scene_pub.publish(pScene)
        planning_scene_pub.publish(pScene)
        # planning_scene_world_pub.publish(pwScene)
        rate.sleep()