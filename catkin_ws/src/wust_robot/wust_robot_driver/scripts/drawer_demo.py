#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import geometry_msgs.msg
import moveit_msgs.msg
from tf import transformations
from mrp2a_moveit_control_full import Mrp2a_Moveit_Control, Object_parameters
from move import MOVE

# 抽屉demo
if __name__ == '__main__':
    mrp2a = Mrp2a_Moveit_Control()

    # 桌子信息
    table_name = "table"
    table_size = (1, 2, 0.8)
    table_pose = (1.55, -0.18, 0.4, 0, 0, 0)

    # 柜子信息
    cupboard = Object_parameters()
    cupboard.name = "drawer"
    cupboard.size = (0.44, 0.40, 0.60)
    cupboard.pose = (1.47, -0.18, 1.1, 0, 0, 0)

    # 柜子1信息
    box1 = Object_parameters()
    box1.name = "box1"
    box1.size = (0.02, 0.40, 0.13)
    box1.pose = (0.96, -0.18, 1.065, 0, 0, 0)

    # 柜子2信息
    box2 = Object_parameters()
    box2.name = "box2"
    box2.size = (0.265, 0.01, 0.08)
    box2.pose = (1.10, 0.02, 1.065, 0, 0, 0)

    # 柜子3信息
    box3 = Object_parameters()
    box3.name = "box3"
    box3.size = (0.265, 0.01, 0.08)
    box3.pose = (1.10, -0.38, 1.065, 0, 0, 0)

    # 柜子4信息
    box4 = Object_parameters()
    box4.name = "box4"
    box4.size = (0.265, 0.40, 0.01)
    box4.pose = (1.12, -0.18, 1.01, 0, 0, 0)

    table_pose_stamped = mrp2a.Pose_Transform(table_pose)
    cupboard_pose_stamped = mrp2a.Pose_Transform(cupboard.pose)
    box1_pose_stamped = mrp2a.Pose_Transform(box1.pose)
    box2_pose_stamped = mrp2a.Pose_Transform(box2.pose)
    box3_pose_stamped = mrp2a.Pose_Transform(box3.pose)
    box4_pose_stamped = mrp2a.Pose_Transform(box4.pose)

    mrp2a.Add_Box(table_name, table_pose_stamped, table_size)
    mrp2a.Add_Box(cupboard.name, cupboard_pose_stamped, cupboard.size)

    # # 打开柜子
    # mrp2a.arm_pose_control("right_arm", [1.02, -0.18, 1.115], [1.57, 0., 1.57])
    # mrp2a.gripper_joint_control("right_gripper", "open", 0)
    # rospy.sleep(1)
    # mrp2a.Plan_Cartesian_Path_Inter("right_arm", [0.07, 0, 0], 7)
    # mrp2a.gripper_joint_control("right_gripper", "close", 0.35)
    # mrp2a.Plan_Cartesian_Path_Inter("right_arm", [-0.27, 0, 0], 28)
    # mrp2a.gripper_joint_control("right_gripper", "open", 0)
    # mrp2a.Plan_Cartesian_Path_Inter("right_arm", [0, -0.20, 0], 22)
    #
    mrp2a.Add_Box(box1.name, box1_pose_stamped, box1.size)
    mrp2a.Add_Box(box2.name, box2_pose_stamped, box2.size)
    mrp2a.Add_Box(box3.name, box3_pose_stamped, box3.size)
    mrp2a.Add_Box(box4.name, box4_pose_stamped, box4.size)

    # # 抓取物品
    # mrp2a.arm_pose_control("right_arm", [1.047, -0.17, 1.289], [-3.132, 0.404, -3.137])
    # mrp2a.Plan_Cartesian_Path_Inter("right_arm", [0, 0, -0.14], 17)
    # mrp2a.gripper_joint_control("right_gripper", "close", 0.35)
    # mrp2a.Plan_Cartesian_Path_Inter("right_arm", [0, 0, 0.14], 17)
    # #
    # # # joint_left = [0, 0, 0, 0, 0, 0, 0]
    # # # joint_right = [0, 0, 0, 0, 0, 0, 0]
    # # # mrp2a.dual_arm_joint_control(joint_left, joint_right)
    #
    # mrp2a.Remove_Box(box1.name, 4)
    # mrp2a.Remove_Box(box2.name, 4)
    # mrp2a.Remove_Box(box3.name, 4)
    # mrp2a.Remove_Box(box4.name, 4)
    # mrp2a.Remove_Box(table_name, 4)
    # mrp2a.Remove_Box(cupboard.name, 4)
    # rospy.sleep(1)
    #
    # # 移动
    # MOVE()
    #
    # # 放置
    # mrp2a.Plan_Cartesian_Path_Inter("right_arm", [-0.13, 0.18, 0], 25)
    # mrp2a.gripper_joint_control("right_gripper", "open", 0)
    #
    # # 双臂端盆
    # # 桌子1信息
    # table1_name = "table1"
    # table1_size = (0.5, 2, 1)
    # table1_pose = (1.3, 0, 0.5, 0, 0, 0)
    #
    # # 桌子2信息
    # table2_name = "table2"
    # table2_size = (0.3, 1.5, 0.9)
    # table2_pose = (0.9, 0, 0.45, 0, 0, 0)
    #
    # # 盘子信息
    # plate = Object_parameters()
    # plate.name = "plate"
    # plate.size = (0.2, 0.2, 0.10)
    # plate.pose = (0.85, 0, 1.21, 0, 0, 0)
    #
    # # 垫高物品信息
    # wood_box1 = Object_parameters()
    # wood_box1.name = "box1"
    # wood_box1.size = (0.05, 0.05, 0.3)
    # wood_box1.pose = (0.85, 0, 1.05, 0, 0, 0)
    #
    # table1_pose_stamped = mrp2a.Pose_Transform(table1_pose)
    # table2_pose_stamped = mrp2a.Pose_Transform(table2_pose)
    # plate_pose_stamped = mrp2a.Pose_Transform(plate.pose)
    # wood_box1_stamped = mrp2a.Pose_Transform(wood_box1.pose)
    #
    # mrp2a.Add_Box(table1_name, table1_pose_stamped, table1_size)
    # mrp2a.Add_Box(table2_name, table2_pose_stamped, table2_size)
    # mrp2a.Add_Box(plate.name, plate_pose_stamped, plate.size)
    # mrp2a.Add_Box(wood_box1.name, wood_box1_stamped, wood_box1.size)
    #
    # pos_left = [0.85, 0.26, 1.08]
    # rot_left = [-2.713, 1.503, 2.244]
    # pos_right = [0.85, -0.26, 1.08]
    # rot_right = [-0.837, -1.473, -1.011]
    # mrp2a.dual_arm_pose_control(pos_left, rot_left, pos_right, rot_right)
    # rospy.sleep(1)
    #
    # mrp2a.dual_gripper_joint_control("open", "open", 0, 0)
    # mrp2a.Remove_Box(plate.name, 4)
    # mrp2a.Remove_Box(wood_box1.name, 4)
    # # 靠近碗
    # mrp2a.dual_Plan_Cartesian_Path_Inter([0, -0.14, 0], [0, 0.14, 0], 17)
    #
    # # 端碗
    # mrp2a.dual_Plan_Cartesian_Path_Inter([0, 0, 0.2], [0, 0, 0.2], 22)
    #
    # # # 斜上
    # # mrp2a.dual_Plan_Cartesian_Path_Inter([0, 0.1, 0.2], [0, 0.1, 0.2], 28)
    #
    # # 向前
    # mrp2a.dual_Plan_Cartesian_Path_Inter([0.2, 0, 0], [0.2, 0, 0], 22)
    #
    # # 向后
    # mrp2a.dual_Plan_Cartesian_Path_Inter([-0.2, 0, 0], [-0.2, 0, 0], 22)
    #
    # # # 向左
    # # mrp2a.dual_Plan_Cartesian_Path_Inter([0, 0.1, 0], [0, 0.1, 0], 12)
    # #
    # # # 还原
    # # mrp2a.dual_Plan_Cartesian_Path_Inter([0, -0.1, 0], [0, -0.1, 0], 12)
    # #
    # # # 向右
    # # mrp2a.dual_Plan_Cartesian_Path_Inter([0, -0.1, 0], [0, -0.1, 0], 12)
    # #
    # # # 还原
    # # mrp2a.dual_Plan_Cartesian_Path_Inter([0, 0.1, 0], [0, 0.1, 0], 12)
    #
    # # 放碗
    # mrp2a.dual_Plan_Cartesian_Path_Inter([0, 0, -0.2], [0, 0, -0.2], 22)
    #
    # # 离开碗
    # mrp2a.dual_Plan_Cartesian_Path_Inter([0, 0.14, 0], [0, -0.14, 0], 17)





