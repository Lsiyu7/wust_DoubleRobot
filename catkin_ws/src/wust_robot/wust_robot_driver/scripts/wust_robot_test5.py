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

# 移动抓取
if __name__ == '__main__':
    mrp2a = Wust_Robot_Moveit_Control()

    # ikfast问题1
    # pos_left = [0.25, -0.06, 0.95]
    # rot_left = [-2.711, 1.502, 2.247]
    # mrp2a.arm_pose_control("left_arm", pos_left, rot_left)
    #
    # q1 = []
    # q2 = []
    # q3 = []
    # q4 = []
    # q5 = []
    # q6 = []
    # q7 = []
    #
    # for i in range(5):
    #
    #     q = mrp2a.Plan_Cartesian_Path_Rot("left_arm", [0.250, 0.24, 0.950], [-2.719, 1.502, 2.240], 0)
    #     mrp2a.Plan_Cartesian_Path_Rot("left_arm", [0.250, -0.060, 1.250], [-2.719, 1.502, 2.240], 0)
    #     mrp2a.Plan_Cartesian_Path_Rot("left_arm", [0.250, -0.060, 0.950], [-2.719, 1.502, 2.240], 0)
    #
    #     for j in range(len(q)):
    #         q1.append(q[j][0])
    #         q2.append(q[j][1])
    #         q3.append(q[j][2])
    #         q4.append(q[j][3])
    #         q5.append(q[j][4])
    #         q6.append(q[j][5])
    #         q7.append(q[j][6])
    #
    # q0 = []
    # for i in range(len(q1)):
    #     q0.append(i + 1)
    #
    # x = np.array(q0)
    #
    # plt.plot(q1)
    # plt.plot(q2)
    # plt.plot(q3)
    # plt.plot(q4)
    # plt.plot(q5)
    # plt.plot(q6)
    # plt.plot(q7)
    # plt.grid(True)
    # plt.axis('tight')
    # plt.xlabel('points')
    # plt.ylabel('joints')
    # plt.show()


    box = Object_parameters()
    box.name = "box"
    box.size = (0.1, 0.16, 0.7)
    box.pose = (0.5, 0, 1.0, 0, 0, 0)

    box_pose_stamped = mrp2a.Pose_Transform(box.pose)

    mrp2a.Add_Box(box.name,box_pose_stamped, box.size)
    #桌子信息
    table_name = "table"
    table_size = (1, 2, 0.84)
    table_pose = (1.15, 0, 0.42, 0, 0, 0)

    # 物体1信息
    wood_box1 = Object_parameters()
    wood_box1.name = "box1"
    wood_box1.size = (0.15, 0.15, 0.15)
    wood_box1.pose = (0.85, 0, 0.975, 0, 0, 0)

    # 物体2信息
    wood_box2 = Object_parameters()
    wood_box2.name = "box2"
    wood_box2.size = (0.18, 0.18, 0.1)
    wood_box2.pose = (0.85, 0, 1.09, 0, 0, 0)

    table_pose_stamped = mrp2a.Pose_Transform(table_pose)

    box1_pose_stamped = mrp2a.Pose_Transform(wood_box1.pose)
    box2_pose_stamped = mrp2a.Pose_Transform(wood_box2.pose)

    mrp2a.Add_Box(table_name, table_pose_stamped, table_size)
    mrp2a.Add_Box(wood_box1.name, box1_pose_stamped, wood_box1.size)
    mrp2a.Add_Box(wood_box2.name, box2_pose_stamped, wood_box2.size)


    # pos_left = [0.3, 0.260, 0.9]
    # rot_left = [-2.711, 1.502, 2.247]
    # pos_right = [0.3, -0.260, 0.9]
    # rot_right = [-0.843, -1.474, -1.006]
    # pos_left = [0.321, 0.260, 1.09]
    # rot_left = [-0.511, 1.557, -1.796]
    # pos_right = [0.321, -0.260, 1.09]
    # rot_right = [2.512, -1.557, 2.014]
    pos_left = [0.85, 0.26, 1.14]
    rot_left = [-2.713, 1.503, 2.244]
    pos_right = [0.85, -0.26, 1.15]
    rot_right = [-0.837, -1.473, -1.011]
    mrp2a.dual_arm_pose_control(pos_left, rot_left, pos_right, rot_right)

    mrp2a.Remove_Box(wood_box1.name, 4)
    mrp2a.Remove_Box(wood_box2.name, 4)

    mrp2a.dual_Plan_Cartesian_Path_Inter([0, -0.14, 0], [0, 0.14, 0], 16)

    # 端碗
    mrp2a.dual_Plan_Cartesian_Path_Inter([0, 0, 0.1], [0, 0, 0.1], 12)
    # #
    # # 斜上
    # #mrp2a.dual_Plan_Cartesian_Path_Inter([0, 0.1, 0.2], [0, 0.1, 0.2], 28)
    # # #
    # 向前
    mrp2a.dual_Plan_Cartesian_Path_Inter([0.1, 0, 0], [0.1, 0, 0], 12)

    # 向后
    mrp2a.dual_Plan_Cartesian_Path_Inter([-0.1, 0, 0], [-0.1, 0, 0], 12)

    # 向左
    mrp2a.dual_Plan_Cartesian_Path_Inter([0, 0.1, 0], [0, 0.1, 0], 12)

    # 还原
    mrp2a.dual_Plan_Cartesian_Path_Inter([0, -0.1, 0], [0, -0.1, 0], 12)

    # 向右
    mrp2a.dual_Plan_Cartesian_Path_Inter([0, -0.1, 0], [0, -0.1, 0], 12)

    # 还原
    mrp2a.dual_Plan_Cartesian_Path_Inter([0, 0.1, 0], [0, 0.1, 0], 12)

    # 放碗
    mrp2a.dual_Plan_Cartesian_Path_Inter([0, 0, -0.1], [0, 0, -0.1], 12)

    # 离开碗
    mrp2a.dual_Plan_Cartesian_Path_Inter([0, 0.14, 0], [0, -0.14, 0], 17)

    joint_left = [0, 0, 0, 0, 0, 0, 0]
    joint_right = [0, 0, 0, 0, 0, 0, 0]
    mrp2a.dual_arm_joint_control(joint_left, joint_right)


    # joint_left = [2.8780125044801674, -0.6129521828229348, -1.3961682898672172, -1.1086789209156103,
    #               -2.548032162360116, 1.0777460351265526, 1.6724047448928314]
    #
    # joint_right = [-0.5702862254376674, -0.4491114336056171, 2.642516032421445, 1.056002993438003,
    #                2.817683884658341, -1.0630895851534452, 2.2025208896459585]
    #
    # mrp2a.dual_arm_joint_control(joint_left, joint_right)

    # # 端碗
    # # mrp2a.dual_Plan_Cartesian_Path_Inter([0, 0, 0.2], [0, 0, 0.2], 22)
    # mrp2a.dual_Plan_Cartesian_Path_Inter([0, 0, 0.15], [0, 0, 0.15], 17)
    #
    # # 向前
    # mrp2a.dual_Plan_Cartesian_Path_Inter([0.1, 0, 0], [0.1, 0, 0], 12)
    #
    # # 向后
    # mrp2a.dual_Plan_Cartesian_Path_Inter([-0.1, 0, 0], [-0.1, 0, 0], 12)
    #
    # # 向左
    # mrp2a.dual_Plan_Cartesian_Path_Inter([0, 0.1, 0], [0, 0.1, 0], 12)
    #
    # # 还原
    # mrp2a.dual_Plan_Cartesian_Path_Inter([0, -0.1, 0], [0, -0.1, 0], 12)
    #
    # # 向右
    # mrp2a.dual_Plan_Cartesian_Path_Inter([0, -0.1, 0], [0, -0.1, 0], 12)
    #
    # # 还原
    # mrp2a.dual_Plan_Cartesian_Path_Inter([0, 0.1, 0], [0, 0.1, 0], 12)
    #
    # # 放碗
    # # mrp2a.dual_Plan_Cartesian_Path_Inter([0, 0, -0.2], [0, 0, -0.2], 12)
    # mrp2a.dual_Plan_Cartesian_Path_Inter([0, 0, -0.15], [0, 0, -0.15], 17)



    # joint_left = [2.8780125044801674, -0.6129521828229348, -1.3961682898672172, -1.1086789209156103,
    #               -2.548032162360116, 1.0777460351265526, 1.6724047448928314]
    #
    # mrp2a.arm_joint_control("left_arm", joint_left)
    #
    # pos_left = [0.25, 0.120, 0.95]
    # rot_left = [-2.711, 1.502, 2.247]
    #
    # mrp2a.arm_pose_control("left_arm", pos_left, rot_left)
    #
    # for i in range(5):
    #
    #     mrp2a.Plan_Cartesian_Path_Rot("left_arm", [0.250, 0.24, 0.950], [-2.719, 1.502, 2.240], 0)
    #     mrp2a.Plan_Cartesian_Path_Rot("left_arm", [0.250, -0.060, 1.250], [-2.719, 1.502, 2.240], 0)
    #     mrp2a.Plan_Cartesian_Path_Rot("left_arm", [0.250, -0.060, 0.950], [-2.719, 1.502, 2.240], 0)
    #     # mrp2a.dual_Plan_Cartesian_Path_Inter([0.250, -0.060, 0.950], [0, -0.10, 0.10], 15)
    #     # mrp2a.dual_Plan_Cartesian_Path_Inter([0, -0.10, 0.10], [0, -0.10, 0.10], 15)
    #     # mrp2a.dual_Plan_Cartesian_Path_Inter([0, -0.10, 0.10], [0, -0.10, 0.10], 15)
    #     #
    #     # mrp2a.dual_Plan_Cartesian_Path_Inter([0, 0, -0.30], [0, 0, -0.30], 32)
    #     #
    #     # mrp2a.dual_Plan_Cartesian_Path_Inter([0, 0.10, 0], [0, 0.10, 0], 15)
    #     # mrp2a.dual_Plan_Cartesian_Path_Inter([0, 0.10, 0], [0, 0.10, 0], 15)
    #     # mrp2a.dual_Plan_Cartesian_Path_Inter([0, 0.10, 0], [0, 0.10, 0], 15)





