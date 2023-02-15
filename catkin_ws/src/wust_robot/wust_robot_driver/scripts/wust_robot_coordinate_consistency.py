#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import geometry_msgs.msg
from tf import transformations
from wust_robot_moveit_control_full import Wust_Robot_Moveit_Control, Object_parameters
import visualization_msgs.msg
import numpy as np
import moveit_msgs.msg
from std_msgs.msg import String
import commands
import os

if __name__ == '__main__':
    wust = Wust_Robot_Moveit_Control()

    # plate = Object_parameters()
    # plate.name = "plate"
    # plate.size = (0.15, 0.4, 0.04)
    # plate.pose = (1.03, 0, 1.02, 0, 0, 0)
    #
    # plate_pose_stamped = wust.Pose_Transform(plate.pose)
    #
    # wust.Add_Box(plate.name, plate_pose_stamped, plate.size)


    # joint_left = [-0.15560724529656925, 1.1811683815526075, -1.2934605670900978, 1.590284874515088,
    #               1.0981166972302192, -0.7913915850502877, -3.047076050846285]
    # joint_right = [0.1612086892562114, -1.2554126573214153, 1.3249594244522804, -1.5551076493825036,
    #                1.9931390329554974, -0.7530252596214994, 3.0456108613750974]
    # wust.dual_arm_joint_control(joint_left, joint_right)

    # pos_left = [0.3, 0.12, 0.9]
    # rot_left = [1.571, 0, 1.571]
    # pos_right = [0.3, -0.12, 0.9]
    # rot_right = [-1.571, 0, -1.571]
    # wust.dual_arm_pose_control(pos_left, rot_left, pos_right, rot_right)
    #wust.Plan_Cartesian_Path("left_arm", [0, 0, 0.2])

    #wust.dual_gripper_joint_control("close", "close", 0.2, 0.2)
    #wust.dual_gripper_joint_control("open", "open", 0.2, 0.2)

    pos_left = [0.25, 0.260, 0.94]
    rot_left = [-2.711, 1.502, 2.247]
    pos_right = [0.25, -0.260, 0.94]
    rot_right = [-0.843, -1.474, -1.006]
    wust.dual_arm_pose_control(pos_left, rot_left, pos_right, rot_right)

    #pos_left = [0.300, 0.180, 0.940]
    #rot_left = [-2.711, 1.502, 2.247]
    #pos_right = [0.300, -0.180, 0.940]
    #rot_right = [-0.843, -1.474, -1.006]
    #wust.dual_arm_pose_control(pos_left, rot_left, pos_right, rot_right)

    #wust.dual_Plan_Cartesian_Path_Inter([0, 0, 0.18], [0, 0, 0.18], 20)
    # 参考位置
    wust.dual_Plan_Cartesian_Path_Inter([0, -0.14, 0], [0, 0.14, 0], 16)
    wust.dual_Plan_Cartesian_Path_Inter([0, 0, 0.1], [0, 0.0, 0.1], 12)

    # 向上后还原
    wust.dual_Plan_Cartesian_Path_Inter([0, 0, 0.1], [0, 0.0, 0.1], 12)
    rospy.sleep(1)
    wust.dual_Plan_Cartesian_Path_Inter([0, 0, -0.1], [0, 0.0, -0.1], 12)

    # 向下后还原
    wust.dual_Plan_Cartesian_Path_Inter([0, 0, -0.1], [0, 0.0, -0.1], 12)
    rospy.sleep(1)
    wust.dual_Plan_Cartesian_Path_Inter([0, 0, 0.1], [0, 0.0, 0.1], 12)

    # # 向前后还原
    wust.dual_Plan_Cartesian_Path_Inter([0.1, 0, 0], [0.1, 0.0, 0], 12)
    rospy.sleep(1)
    wust.dual_Plan_Cartesian_Path_Inter([-0.1, 0, 0], [-0.1, 0.0, 0], 12)
    #
    # # 向后后还原
    #wust.dual_Plan_Cartesian_Path_Inter([-0.1, 0, 0], [-0.1, 0.0, 0], 12)
    #rospy.sleep(1)
    #wust.dual_Plan_Cartesian_Path_Inter([0.1, 0, 0], [0.1, 0.0, 0], 12)

    # # 向左后还原
    wust.dual_Plan_Cartesian_Path_Inter([0, 0.1, 0], [0, 0.1, 0], 12)
    rospy.sleep(1)
    wust.dual_Plan_Cartesian_Path_Inter([0, -0.1, 0], [0, -0.1, 0], 12)
    #
    # # 向右后还原
    wust.dual_Plan_Cartesian_Path_Inter([0, -0.1, 0], [0, -0.1, 0], 12)
    rospy.sleep(1)
    wust.dual_Plan_Cartesian_Path_Inter([0, 0.1, 0], [0, 0.1, 0], 12)
    #


    # pos_left = [0.3, 0.16, 1.1]
    # rot_left = [1.571, 0, 1.571]
    # pos_r_from_l = [-0.240, 0.005, 0.001]
    # rot_r_from_l = [0, 0, -3.141]
    #
    # wust.dual_arm_coordinate(pos_left, rot_left, pos_r_from_l, rot_r_from_l)

    # pos_left = [0.300, 0.180, 0.940]
    # rot_left = [-2.711, 1.502, 2.247]
    # pos_right = [0.300, -0.180, 0.940]
    # rot_right = [-0.843, -1.474, -1.006]
    # wust.dual_arm_pose_control(pos_left, rot_left, pos_right, rot_right)

    #wust.dual_Plan_Cartesian_Path_Inter([0, 0.05, 0.15], [0, 0.05, 0.15], 20)

    # pos_left = [0.300, 0.180, 1.120]
    # rot_left = [-2.711, 1.502, 2.247]
    # pos_right = [0.300, -0.180, 1.120]
    # rot_right = [-0.843, -1.474, -1.006]
    #
    # wust.dual_arm_pose_control_display(pos_left, rot_left, pos_right, rot_right, 1)

    # main = "/home/zjy/catkin_ws/devel/lib/wust_robot_driver/dual_arm_coordination"
    # print '调用cpp'
    #
    # print '*' * 10, 'begin', '*' * 10
    # os.system(main)
    # print '*' * 10, 'end', '*' * 10

    # pos_left = [0.300, 0.180, 1.12]
    # rot_left = [-2.711, 1.502, 2.247]
    # pos = [-0.013, -0.059, 0.232]
    # rot = [2.620, -0.031, 3.140]
    # wust.dual_arm_coordinate(pos_left, rot_left, pos, rot)
