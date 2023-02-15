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

# 移动抓取

if __name__ == '__main__':
    wust = Wust_Robot_Moveit_Control()
    wust2 = MOVE()
    # pos_left = [0.22, 0.25, 0.95]
    # rot_left = [1.57, 0, 0]
    # pos_right = [0.22, -0.25, 0.95]
    # rot_right = [-1.57, 0, 0]
    # wust.dual_arm_pose_control(pos_left, rot_left, pos_right, rot_right)

    # 桌子信息
    table = Object_parameters()
    table.name = "table"
    table.size = (1, 2, 0.9)
    table.pose = (0.8, 0, 0.45, 0, 0, 0)

    # 水杯信息
    cup = Object_parameters()
    cup.name = "cup"
    cup.size = (0.05, 0.05, 0.12)
    cup.pose = (0.44, 0.3, 0.96, 0, 0, 0)

    # 水瓶信息
    bottle = Object_parameters()
    bottle.name = "bottle"
    bottle.size = (0.05, 0.05, 0.2)
    bottle.pose = (0.44, -0.3, 1.0, 0, 0, 0)

    table_pose_stamped = wust.Pose_Transform(table.pose)
    cup_pose_stamped = wust.Pose_Transform(cup.pose)
    bottle_pose_stamped = wust.Pose_Transform(bottle.pose)

    wust.Add_Box(table.name, table_pose_stamped, table.size)
    wust.Add_Box(cup.name, cup_pose_stamped, cup.size)
    wust.Add_Box(bottle.name, bottle_pose_stamped, bottle.size)

    # 抓取水瓶和水杯
    pos_left = [0.35, 0.3, 0.94]
    rot_left = [1.57, 1.57, 1.57]
    pos_right = [0.35, -0.3, 0.97]
    rot_right = [1.57, -1.57, 1.57]
    wust.dual_arm_pose_control(pos_left, rot_left, pos_right, rot_right)

    # joint_left = [2.364541123298391, -0.17350603498252148, 0.03898873592993812, -1.7705584862151875,
    #               0.7525086002994117, 1.2084344224192387, -2.3929536554087223]
    # joint_right = [-0.05161674231453883, -0.22947992695526587, 2.2702262980315915, 1.8080869872673837,
    #                0.7669545499549324, -1.065404395260126, -2.2067931836120813]
    # wust.dual_arm_joint_control(joint_left, joint_right)
    #
    wust.dual_gripper_joint_control("open", "open", 0, 0)
    wust.dual_Plan_Cartesian_Path_Inter([0.1, 0, 0], [0.1, 0, 0], 12)
    wust.Remove_Box(bottle.name)
    wust.Attach_Box("left_arm", cup.name, 4)
    # wust.Attach_Box("right_arm", bottle.name, 4)
    wust.dual_gripper_joint_control("close", "close", 0.15, 0.15)
    wust.Plan_Cartesian_Path_Inter("left_arm", [0, 0, 0.10], 12)
    wust.Plan_Cartesian_Path_Inter("right_arm", [0, 0, 0.18], 20)
    # wust.Plan_Cartesian_Path_Inter("right_arm", [0, 0.22, 0], 25)
    wust.dual_Plan_Cartesian_Path_Inter([0, -0.24, 0], [0, 0.24, 0], 25)
    rospy.sleep(2)
    wust.arm_joint_change("R_joint7", 1.5)
    rospy.sleep(5)

    # 将水瓶放下,水杯放入托盘中
    joint_right = [-0.6001370762565172, -0.560869580844884, 2.3146626491116726, 1.0711386271567538,
                   0.8873040986724313, -0.49803614028792725, -2.1450571471926168]
    wust.arm_joint_control("right_arm", joint_right)
    rospy.sleep(1)
    wust.gripper_joint_control("right_gripper", "open", 0)
    wust.Plan_Cartesian_Path_Inter("left_arm", [-0.12, 0, 0], 14)
    wust.Plan_Cartesian_Path_Inter("right_arm", [-0.12, 0, 0], 14)
    # joint_left = [2.081330024913328, -0.3072171685371522, -0.45433584441188196, -1.9176769926230248,
    #               -0.1850446612815198, 0.06891613709744275, -1.2246124172166748]
    # joint_right = [-0.45017612363821813, -0.1418685342487244, 2.820748676971199, 1.8874875789261287,
    #                0.7496877263546844, -1.225202050228991, -2.314910200790483]
    # wust.dual_arm_joint_control(joint_left, joint_right)

    # wust2.pos_move([0, 0.4, 0], 1.61)
    wust2.pos_move([0, 0.4, 0], 1.75)

    wust.Plan_Cartesian_Path_Inter("left_arm", [0.12, -0.06, -0.09], 15)
    wust.dual_gripper_joint_control("open", "open", 0, 0)
    wust.Detach_Box("left_arm", cup.name, 4)
    wust.Remove_Box(cup.name)
    wust.Plan_Cartesian_Path_Inter("left_arm", [-0.1, 0, 0], 12)

    rospy.sleep(1)

    # def callback(data):
    #     rospy.loginfo("I heard %s", data.data)
    #     if data.data =="来杯热水吧":
    #         rospy.sleep(5)


            # 双臂协调抓起托盘并放到指定地点
            # 支架1信息
            # box1 = Object_parameters()
            # box1.name = "box1"
            # box1.size = (0.1, 0.1, 0.16)
            # box1.pose = (0.445, 0.16, 0.98, 0, 0, 0)
            #
            # # 支架2信息
            # box2 = Object_parameters()
            # box2.name = "box2"
            # box2.size = (0.1, 0.1, 0.16)
            # box2.pose = (0.445, -0.16, 0.98, 0, 0, 0)
            #
            # box1_pose_stamped = wust.Pose_Transform(box1.pose)
            # box2_pose_stamped = wust.Pose_Transform(box2.pose)
            #
            # wust.Add_Box(box1.name, box1_pose_stamped, box1.size)
            # wust.Add_Box(box2.name, box2_pose_stamped, box2.size)
            #
            # pos_left = [0.35, 0.16, 0.97]
            # rot_left = [1.571, 1.571, 1.571]
            # pos_right = [0.35, -0.16, 0.97]
            # rot_right = [1.571, -1.571, 1.571]
            #
            # wust.Remove_Box(box1.name)
            # wust.Remove_Box(box2.name)
            # wust.dual_arm_pose_control(pos_left, rot_left, pos_right, rot_right)
            # wust.dual_gripper_joint_control("open", "open", 0, 0)
            # wust.Plan_Cartesian_Path_Inter("left_arm", [0.1, 0, 0], 12)
            # wust.Plan_Cartesian_Path_Inter("right_arm", [0.1, 0, 0], 12)
            # # wust.Attach_Box("left_arm", box1.name, 4)
            # # wust.Attach_Box("right_arm", box2.name, 4)
            # wust.dual_gripper_joint_control("close", "close", 0.13, 0.13)
            # wust.dual_Plan_Cartesian_Path_Inter([0, 0, 0.1], [0, 0, 0.1], 12)
            #
            # wust2 = MOVE()
            # wust2.rot_move([0, 0, 0.2], 5.5*pi)
            # wust2.pos_move([0.4, 0, 0], 2.5)
            # wust.Remove_Box(table.name)
            #
            # wust.dual_Plan_Cartesian_Path_Inter([0, 0, -0.15], [0, 0, -0.15], 17)
            # wust.dual_gripper_joint_control("open", "open", 0, 0)
            # wust.dual_Plan_Cartesian_Path_Inter([-0.1, 0, 0], [-0.1, 0, 0], 12)
            # joint_left = [0, 0, 0, 0, 0, 0, 0]
            # joint_right = [0, 0, 0, 0, 0, 0, 0]
            # wust.dual_arm_joint_control(joint_left, joint_right)

#     def listener():
#         # In ROS, nodes are uniquely named. If two nodes with the same
#         # node are launched, the previous one is kicked off. The
#         # anonymous=True flag means that rospy will choose a unique
#         # name for our 'listener' node so that multiple listeners can
#         # run simultaneously.
#         #rospy.init_node('listener', anonymous=True)
#
#         rospy.Subscriber("/recognizer/output", String, callback)
#
#         # spin() simply keeps python from exiting until this node is stopped
#         rospy.spin()
#
# listener()
    # #
    # # wust.dual_gripper_joint_control("open", "open", 0, 0)
    # # wust.dual_Plan_Cartesian_Path_Inter([0, -0.04, 0], [0, 0.04, 0], 5)
    # #
    # # wust.dual_gripper_joint_control("close", "close", 0.14, 0.14)
    # #
    # # wust.dual_Plan_Cartesian_Path_Inter([0, 0, 0.08], [0, 0, 0.08], 10)
    #
    #

