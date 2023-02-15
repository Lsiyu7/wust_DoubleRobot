#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
import geometry_msgs.msg
import moveit_msgs.msg
from tf import transformations
from wust_robot_moveit_control_full import Wust_Robot_Moveit_Control, Object_parameters
from move import MOVE
import numpy as np
import math
import matplotlib as mpl
import matplotlib.pyplot as plt
from math import pi
from std_msgs.msg import String


if __name__ == '__main__':
    wust = Wust_Robot_Moveit_Control()

    # box_name = "box"
    # box_size = (0.1, 0.16, 0.7)
    # box_pose = (0.5, 0, 1.0, 0, 0, 0)
    #
    # box_pose_stamped = wust.Pose_Transform(box_pose)
    #
    # wust.Add_Box(box_name,box_pose_stamped, box_size)
    # # wust.head_joint_control([0, 0.7])
    # #
    # # pos_left = [0.7, 0.25, 1.1]
    # # pos_right = [0.7, -0.25, 1.1]
    # # rot_left = [1.57, 0, 0]
    # # rot_right = [-1.57, 0, 0]
    # # wust.dual_arm_pose_control(pos_left, rot_left, pos_right, rot_right)
    # # pos_left = [0.3, 0.2, 1.1]
    # # rot_left = [-3.14, 0.79, -3.14]
    # # pos_right = [0.3, -0.2, 1.1]
    # # rot_right = [3.14, -0.96, 0]
    # # wust.dual_arm_pose_control(pos_left, rot_left, pos_right, rot_right)
    #
    # # joint = [-1, 0, 0, 0, 0, 0, 0]
    # # wust.arm_joint_control("right_arm", joint)
    #
    # joint_left = [-0.5, 0, 0, 0, 0, 0, 0]
    # joint_right = [0.5, 0, 0, 0, 0, 0, 0]
    # wust.dual_arm_joint_control(joint_left, joint_right)
    #
    # joint_left = [-0.5, 1, 0, 0, 0, 0, 0]
    # joint_right = [0.5, -1, 0, 0, 0, 0, 0]
    # wust.dual_arm_joint_control(joint_left, joint_right)
    #
    # joint_left = [-0.5, 1, -1, 0, 0, 0, 0]
    # joint_right = [0.5, -1, 1, 0, 0, 0, 0]
    # wust.dual_arm_joint_control(joint_left, joint_right)
    #
    # joint_left = [-0.5, 1, -1, 0.5, 0, 0, 0]
    # joint_right = [0.5, -1, 1, -0.5, 0, 0, 0]
    # wust.dual_arm_joint_control(joint_left, joint_right)
    #
    # joint_left = [-0.5, 1, -1, 0.5, -1, 0, 0]
    # joint_right = [0.5, -1, 1, -0.5, 1, 0, 0]
    # wust.dual_arm_joint_control(joint_left, joint_right)
    #
    # joint_left = [-0.5, 1, -1, 0.5, -1, 1, 0]
    # joint_right = [0.5, -1, 1, -0.5, 1, -1, 0]
    # wust.dual_arm_joint_control(joint_left, joint_right)
    #
    # joint_left = [-0.5, 1, -1, 0.5, -1, 1, -1]
    # joint_right = [0.5, -1, 1, -0.5, 1, -1, 1]
    # wust.dual_arm_joint_control(joint_left, joint_right)
    #
    # joint_left = [0, 0, 0, 0, 0, 0, 0]
    # joint_right = [0, 0, 0, 0, 0, 0, 0]
    # wust.dual_arm_joint_control(joint_left, joint_right)
    #
    # # # 桌子信息
    # # table = Object_parameters()
    # # table.name = "table"
    # # table.size = (1, 2, 0.9)
    # # table.pose = (0.55, 0, 0.45, 0, 0, 0)
    # #
    # # # 水杯信息
    # # cup = Object_parameters()
    # # cup.name = "cup"
    # # cup.size = (0.2, 0.2, 0.1)
    # # cup.pose = (0.22, 0, 0.95, 0, 0, 0)
    # #
    # # # 水瓶信息
    # # bottle = Object_parameters()
    # # bottle.name = "bottle"
    # # bottle.size = (0.2, 0.23, 0.03)
    # # bottle.pose = (                                                                                                                                                                                        , 0, 0, 0)
    # #
    # # table_pose_stamped = wust.Pose_Transform(table.pose)
    # # cup_pose_stamped = wust.Pose_Transform(cup.pose)
    # # bottle_pose_stamped = wust.Pose_Transform(bottle.pose)
    # #
    # # wust.Add_Box(table.name, table_pose_stamped, table.size)
    # # wust.Add_Box(cup.name, cup_pose_stamped, cup.size)
    # # wust.Add_Box(bottle.name, bottle_pose_stamped, bottle.size)
    # #
    # # pos_left = [0.22, 0.22, 1.02]
    # # rot_left = [1.57, 0, 0]
    # # pos_right = [0.22, -0.22, 1.02]
    # # rot_right = [-1.57, 0, 0]
    # # wust.dual_arm_pose_control(pos_left, rot_left, pos_right, rot_right)
    # #
    # # wust.Remove_Box(cup.name)
    # # wust.Remove_Box(bottle.name)
    # #
    # # wust.dual_gripper_joint_control("open", "open", 0, 0)
    # # wust.dual_Plan_Cartesian_Path_Inter([0, -0.1, 0], [0.0, 0.1, 0], 12)
    # # wust.dual_gripper_joint_control("close", "close", 0.34, 0.34)
    # # wust.dual_Plan_Cartesian_Path_Inter([0, 0, 0.1], [0.0, 0, 0.1], 12)
    # # rospy.sleep(2)
    # #
    # # wust2.pos_move([-0.2, 0, 0], 2.0)
    # # rospy.sleep(1)
    # #
    # # wust2.rot_move([0, 0, 0.5], 2.08*pi)
    # # rospy.sleep(1)
    # #
    # # wust2.pos_move([0.2, 0, 0], 7.0)
    # # rospy.sleep(1)
    # #
    # # wust2.rot_move([0, 0, -0.5], 2.08*pi)
    # # rospy.sleep(1)
    # #
    # # wust2.pos_move([0.2, 0, 0], 14.0)
    # # rospy.sleep(1)
    # #
    # # wust2.rot_move([0, 0, -0.5], 2.08*pi)
    # # rospy.sleep(1)
    # #
    # # wust2.pos_move([0.2, 0, 0], 6.4)
    # # rospy.sleep(1)
    # #
    # # wust.Add_Box(bottle.name, bottle_pose_stamped, bottle.size)
    # # wust2.rot_move([0, 0, 0.5], 2.08*pi)
    # # rospy.sleep(1)
    # #
    # # wust2.pos_move([0.2, 0, 0], 7.5)
    # # rospy.sleep(1)
    #
    # # 水瓶信息
    # # bottle = Object_parameters()
    # # bottle.name = "bottle"
    # # bottle.size = (0.2, 0.23, 0.03)
    # # bottle.pose = (0.22, 0, 1.015, 0, 0, 0)
    # #
    # # bottle_pose_stamped = wust.Pose_Transform(bottle.pose)

    # joints = [1.2930468619576105, -0.2343073379017514, 1.0644097599465199, -1.942919398150267,
    #           0.5363087564734575, 1.3122057222899124, -3.025596302054855]
    # # joints = [2.9546065484213173, -1.440573599488638, -2.285479917649294, -1.942656287262584,
    # #           -0.34368220035891117, 0.6268950931792165, -0.44972667737898614]
    # wust.arm_joint_control("left_arm", joints)

    # wust.arm_angle("left_arm", joints)



    # #SphericalJoint = [0.5, -0.5, 0.5, 0, 0, 0, 0]
    # SphericalJoint = [-2.6413573757513698, 0.5, -2.6415636882783424, 0, 0, 0, 0]
    # # # # SphericalJoint = [2.237, 0.8, -0.65, 0, 0, 0, 0]
    # # # #
    # wust.arm_joint_control("left_arm", SphericalJoint)
    # wust.arm_angle(SphericalJoint)
    # # #
    # # # print transformations.euler_matrix(0.919, 0.472, 2.237)
    # #
    # #wust.forward_kinematics_elbow([1, 1, 1, 0, 0, 0, 0])
    #
    # T = transformations.euler_matrix(0.919, 0.472, 2.237)
    # t = transformations.euler_matrix(0, 1.5708, 0)
    # T_Spherical = np.dot(T, t)
    # print T_Spherical

    # T = transformations.euler_matrix(1.571, -0.000, 0.000)
    #
    #
    # T2 = transformations.euler_matrix(-1.039, 0.231, -1.735)
    #
    # T3 = np.dot(np.linalg.inv(T),T2)
    #
    #
    # #
    # # R = np.dot(R, )
    # # wust.SphericalJoint_inverse(T)
    #
    # # T2 = transformations.euler_matrix(-0.413, -1.288, -2.714)
    # wust.SphericalJoint_inverse(T3)

    # T_base = transformations.euler_matrix(1.583, -0.525, -1.577)
    # T_base = np.array([[T_base[0][0], T_base[0][1], T_base[0][2], 1.153],
    #                       [T_base[1][0], T_base[1][1], T_base[1][2], -0.278],
    #                       [T_base[2][0], T_base[2][1], T_base[2][2],0.085],
    #                       [0,0,0,1]])
    #
    # T_from_b = transformations.euler_matrix(-1.571, 0.898, -1.571)
    # T_from_b = np.array([[T_from_b[0][0], T_from_b[0][1], T_from_b[0][2], 0.299],
    #                       [T_from_b[1][0], T_from_b[1][1], T_from_b[1][2], 0.266],
    #                       [T_from_b[2][0], T_from_b[2][1], T_from_b[2][2], 1.003],
    #                       [0,0,0,1]])
    #
    # T_from_L0 = np.dot(T_base, T_from_b)
    # print T_from_L0
    # print transformations.euler_from_matrix(T_from_L0)

    # joints = [-1.316462310461337, 1.2798359172342517, 2.845345334712854,
    #           -1.1510139018890524, 0.3191529523858807, 1.2245782252481263, 1.606225406396628]
    # joints = [-0.3484488336393126, 0.6710457672521338, -1.633637587992854, 1.156293723293174,
    #           2.9428848149434117, 0.09932848245409227, 2.4895359113695568]
    # wust.arm_joint_control("right_arm", joints)

    # joints = [1.283208990385707, -1.637361966278735, 0.5301131382665901,
    #           1.7797747535902115, 0.6277186745347357, -1.568511667596115, 1.0625262416019117]
    # wust.arm_angle("right_arm", joints)

    # 盘子信息
    plate = Object_parameters()
    plate.name = "plate"
    plate.size = (0.15, 0.4, 0.04)
    plate.pose = (1.03, 0, 1.02, 0, 0, 0)

    plate_pose_stamped = wust.Pose_Transform(plate.pose)

    wust.Add_Box(plate.name, plate_pose_stamped, plate.size)
