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

    pos_left = [0.300, 0.120, 0.940]
    rot_left = [-2.711, 1.502, 2.247]
    pos_right = [0.300, -0.120, 0.940]
    rot_right = [-0.843, -1.474, -1.006]
    wust.dual_arm_pose_control(pos_left, rot_left, pos_right, rot_right)

    # main = "/home/zjy/catkin_ws/devel/lib/wust_robot_driver/dual_arm_coordination"
    # print '调用cpp'
    # 
    # print '*' * 10, 'begin', '*' * 10
    # os.system(main)
    # print '*' * 10, 'end', '*' * 10

    pos_left = [0.300, 0.180, 1.12]
    rot_left = [-2.711, 1.502, 2.247]
    pos = [-0.013, -0.059, 0.232]
    rot = [2.620, -0.031, 3.140]
    wust.dual_arm_coordinate(pos_left, rot_left, pos, rot)