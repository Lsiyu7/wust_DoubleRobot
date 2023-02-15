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

# 移动抓取
if __name__ == '__main__':
    wust = Wust_Robot_Moveit_Control()

    #a = transformations.euler_matrix(0.013, 1.280, 0.010)
    # a = transformations.euler_matrix(3.133, 1.051, 3.127)
    #
    # r = np.array([[a[0][0], a[0][1], a[0][2]],
    #               [a[1][0], a[1][1], a[1][2]],
    #               [a[2][0], a[2][1], a[2][2]]])
    #
    # tw = np.array([0.00, 0.035, -0.234])
    #
    # bt = np.array([0.413, -0.002, 0.220])
    # bs = np.array([0, 0.052, 0.164])
    # tw0 = np.dot(r, tw)
    # wt0 = -tw0
    #
    # sw = bt-bs-wt0
    #
    # sw2 = np.linalg.norm(sw)
    #
    # x = math.pow(0.292, 2) + math.pow(0.242, 2) - math.pow(sw2, 2)
    # y = 2*0.292*0.242
    # z = x/y
    #
    # print tw0
    # print wt0
    # print x
    # print y
    # print z
    # print math.cos(2.09)
    pos = [0.30, 0.26, 1.12]
    rot = [-2.713, 1.503, 2.244]
    wust.arm_pose_control("left_arm", pos, rot)
