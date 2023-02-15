#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import geometry_msgs.msg
from tf import transformations
from mrp2a_moveit_control_full import Mrp2a_Moveit_Control, Object_parameters

# 避障规划
if __name__ == '__main__':
    mrp2a = Mrp2a_Moveit_Control()
    #
    # # 桌子信息
    # table_name = "table"
    # table_size = (1, 2, 0.9)
    # table_pose = (1.25, 0, 0.45, 0, 0, 0)
    #
    # # 障碍物1信息
    # obstacle1 = Object_parameters()
    # obstacle1.name = "obstacle1"
    # obstacle1.size = (0.2, 0.05, 0.6)
    # obstacle1.pose = (1.1, 0.3, 0.95, 0, 0, 0)
    #
    # # 障碍物2信息
    # obstacle2 = Object_parameters()
    # obstacle2.name = "obstacle2"
    # obstacle2.size = (0.2, -0.05, 0.6)
    # obstacle2.pose = (1.1, -0.3, 0.95, 0, 0, 0)
    #
    # # # 障碍物信息
    # # obstacle = Object_parameters()
    # # obstacle.name = "obstacle"
    # # obstacle.size = (0.5, 0.1, 0.5)
    # # obstacle.pose = (1.12, 0.6, 1.2, 0, 0, 0)
    #
    # table_pose_stamped = mrp2a.Pose_Transform(table_pose)
    # obstacle1_pose_stamped = mrp2a.Pose_Transform(obstacle1.pose)
    # obstacle2_pose_stamped = mrp2a.Pose_Transform(obstacle2.pose)
    #
    # mrp2a.Add_Box(table_name, table_pose_stamped, table_size)
    # mrp2a.Add_Box(obstacle1.name, obstacle1_pose_stamped, obstacle1.size)
    # mrp2a.Add_Box(obstacle2.name, obstacle2_pose_stamped, obstacle2.size)
    #
    # pos_left = [1, 0.15, 1.02]
    # rot_left = [1.57, 0, 1.57]
    # pos_right = [1, -0.15, 1.02]
    # rot_right = [1.57, 0, 1.57]
    # mrp2a.dual_arm_pose_control(pos_left, rot_left, pos_right, rot_right)

    # 障碍物1信息
    obstacle1 = Object_parameters()
    obstacle1.name = "obstacle1"
    obstacle1.size = (0.4, 0.05, 0.2)
    obstacle1.pose = (0.95, 0.3, 1.35, 0, 0, 0)

    # 障碍物2信息
    obstacle2 = Object_parameters()
    obstacle2.name = "obstacle2"
    obstacle2.size = (0.4, 0.05, 1)
    obstacle2.pose = (0.95, 0.3, 0.5, 0, 0, 0)

    # 障碍物3信息
    obstacle3 = Object_parameters()
    obstacle3.name = "obstacle3"
    obstacle3.size = (0.4, 0.05, 0.2)
    obstacle3.pose = (0.95, -0.3, 1.35, 0, 0, 0)

    # 障碍物4信息
    obstacle4 = Object_parameters()
    obstacle4.name = "obstacle4"
    obstacle4.size = (0.4, 0.05, 1)
    obstacle4.pose = (0.95, -0.3, 0.5, 0, 0, 0)

    obstacle1_pose_stamped = mrp2a.Pose_Transform(obstacle1.pose)
    obstacle2_pose_stamped = mrp2a.Pose_Transform(obstacle2.pose)
    obstacle3_pose_stamped = mrp2a.Pose_Transform(obstacle3.pose)
    obstacle4_pose_stamped = mrp2a.Pose_Transform(obstacle4.pose)

    mrp2a.Add_Box(obstacle1.name, obstacle1_pose_stamped, obstacle1.size)
    mrp2a.Add_Box(obstacle2.name, obstacle2_pose_stamped, obstacle2.size)
    mrp2a.Add_Box(obstacle3.name, obstacle3_pose_stamped, obstacle3.size)
    mrp2a.Add_Box(obstacle4.name, obstacle4_pose_stamped, obstacle4.size)

    pos_left = [1, 0.15, 1.02]
    rot_left = [1.57, 0, 1.57]
    pos_right = [1, -0.15, 1.02]
    rot_right = [1.57, 0, 1.57]

    # joint_right = [0.15856033573230466, 0.056523908157398495, -0.09909351393843213, -1.1539856962795687,
    #               2.0195059289534996, -1.2968013089797754, 1.9639641765907587]
    # joint_left = [-0.15856033573230466, -0.056523908157398495, 0.09909351393843213, 1.1539856962795687,
    #                -2.0195059289534996, 1.2968013089797754, -1.9639641765907587]

    #mrp2a.dual_arm_joint_control(joint_left, joint_right)
    #mrp2a.dual_arm_joint_control([0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0])
    mrp2a.dual_arm_pose_control(pos_left, rot_left, pos_right, rot_right)
    mrp2a.dual_arm_joint_control([0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0])

    # i = 1
    # while i <= 20:
    #     #mrp2a.dual_arm_joint_control([0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0])
    #     #rospy.sleep(5)
    #     mrp2a.dual_arm_pose_control(pos_left, rot_left, pos_right, rot_right)
    #     rospy.sleep(5)
    #     i = i + 1




