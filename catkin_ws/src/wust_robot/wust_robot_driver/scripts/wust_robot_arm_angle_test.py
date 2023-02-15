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

    #joints = [1.2930468619576105, -0.2343073379017514, 1.0644097599465199, -1.942919398150267,
    #           0.5363087564734575, 1.3122057222899124, -3.025596302054855]
    # joints = [2.9546065484213173, -1.440573599488638, -2.285479917649294, -1.942656287262584,
    #           -0.34368220035891117, 0.6268950931792165, -0.44972667737898614]
    # joints = [1.4537906361056707, 0.5292814414446128, 0.6504809725299684, -1.9004967824024614,
    #           -2.0507958096308316, -0.6352609605378292, 0.521632023732725]
    #wust.arm_joint_control("left_arm", joints)
    # arm_angle = wust.arm_angle("left_arm", joints)
    pos = [0.299, 0.270, 1.002]
    rot = [-1.575, -0.976, -1.565]
    # pos = [0.299, 0.266, 1.003]
    # rot = [-1.572, 0.898, -1.572]
    # #wust.arm_angle_inverse_test(arm_angle)
    #
    wust.arm_angle_inverse("left_arm", pos, rot, -2.20449203651)
    # wust.arm_joint_control("left_arm", joints[4])
    # # wust.arm_joint_control("left_arm", joints[1])
    # # wust.arm_joint_control("left_arm", joints[2])
    # # wust.arm_joint_control("left_arm", joints[3])


    # pos = [0.147, -0.404, 0.455]
    # rot = [0.937, 0.560, -0.377]
    # pos = [0.233, 0.424, 1.018]
    # rot = [-1.572, 1.450, -1.572]

    # joints = [-0.3484488336393126, 0.6710457672521338, -1.633637587992854, 1.156293723293174,
    #           2.9428848149434117, 0.09932848245409227, 2.4895359113695568]
    # # joints = [1.283208990385707, -1.637361966278735, 0.5301131382665901,
    # #           1.7797747535902115, 0.6277186745347357, -1.568511667596115, 1.0625262416019117]
    # #wust.arm_joint_control("left_arm", joints)
    # wust.arm_joint_control("right_arm", joints)
    # arm_angle = wust.arm_angle("right_arm", joints)
    # # pos = [0.442, -0.330, 1.295]
    # # rot = [-2.071, 1.501, -1.966]
    # pos = [0.310, -0.259, 1.013]
    # rot = [-0.338, 1.385, -0.228]
    #
    # #wust.arm_angle_inverse_test(arm_angle)
    #
    # joints = wust.arm_angle_inverse("right_arm", pos, rot, arm_angle)
    # wust.arm_joint_control("right_arm", joints[0])
    # wust.arm_joint_control("right_arm", joints[1])
    # # wust.arm_joint_control("right_arm", joints[2])
    # # wust.arm_joint_control("right_arm", joints[3])

