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

    joint_left = [-1.331631050747329, 0.6677687533218706, -1.6921075122496427, -0.5282955354037345,
                  -1.4833200166306773, -1.674355861921446, -0.9026436636415404]
    joint_right = [1.1529981269236893, -0.7154785723522133, 1.8471283744841749, 0.442508761475337,
                   -1.5384899696401644, -1.7098109653363878, -2.20134602165293]
    wust.dual_arm_joint_control(joint_left, joint_right)

    # pos_left = [0.300, 0.180, 0.940]
    # rot_left = [-2.711, 1.502, 2.247]
    # pos_right = [0.300, -0.180, 0.940]
    # rot_right = [-0.843, -1.474, -1.006]
    # wust.dual_arm_pose_control(pos_left, rot_left, pos_right, rot_right)

    #wust.dual_Plan_Cartesian_Path_Inter([0, 0, 0.18], [0, 0, 0.18], 20)

    pos_left = [0.300, 0.180, 1.120]
    rot_left = [-2.711, 1.502, 2.247]
    pos_right = [0.300, -0.180, 1.120]
    rot_right = [-0.843, -1.474, -1.006]

    #wust.dual_arm_pose_control_display(pos_left, rot_left, pos_right, rot_right, 1)

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
