#!/usr/bin/env python

import rospy
import sys

import moveit_commander
from geometry_msgs.msg import Pose


if __name__ == '__main__':
    rospy.init_node('ariac_competetion_solution_node', anonymous=True)
    ns = 'uav3'
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander(robot_description="/"+ns+"/robot_description",ns=ns)
    scene = moveit_commander.PlanningSceneInterface(ns=ns)
    group_name = ns+"_group"
    move_group = moveit_commander.MoveGroupCommander(group_name,robot_description="/"+ns+"/robot_description",ns=ns)
    
    # print(move_group.get_planning_frame())
    # print(robot.get_current_state())

    start_state = robot.get_current_state()
    goal_state = robot.get_current_state()

    goal_state.multi_dof_joint_state.transforms[0].translation.x = -10.0
    goal_state.multi_dof_joint_state.transforms[0].translation.y = 10.0
    goal_state.multi_dof_joint_state.transforms[0].translation.z = 5.0
    goal_state.multi_dof_joint_state.transforms[0].rotation.x = 0.0
    goal_state.multi_dof_joint_state.transforms[0].rotation.y = 0.0
    goal_state.multi_dof_joint_state.transforms[0].rotation.z = 0.0
    goal_state.multi_dof_joint_state.transforms[0].rotation.w = 1.0


    move_group.set_workspace([-50,-50,-1,50,50,50]) #[minX, minY, minZ, maxX, maxY, maxZ] 
    move_group.set_start_state(start_state)
    move_group.set_planner_id("RRTConnect")
    move_group.set_num_planning_attempts(10)

    # print(move_group.has_end_effector_link())
    # print(move_group.get_end_effector_link())
    # print(move_group.get_current_joint_values())

    
    move_group.set_joint_value_target(goal_state)
    # pose_target = Pose()
    # pose_target.orientation.w = 1.0
    # pose_target.position.x = 0
    # pose_target.position.y = 0
    # pose_target.position.z = 10
    # move_group.set_pose_target(pose_target)
    # for i in range(10):
    plan1 = move_group.plan()
    move_group.clear_pose_targets()
    print(plan1)