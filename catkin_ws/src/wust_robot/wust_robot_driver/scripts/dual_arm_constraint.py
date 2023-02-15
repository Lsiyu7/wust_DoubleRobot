#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import geometry_msgs.msg
from tf import transformations
import numpy
import trajectory_msgs.msg
from mrp2a_moveit_control_full import Mrp2a_Moveit_Control, Object_parameters
import commands
import os

if __name__ == '__main__':
    # mrp2a = Mrp2a_Moveit_Control()
    #
    # # 桌子信息
    # table_name = "table"
    # table_size = (1, 2, 0.9)
    # table_pose = (1.25, 0, 0.45, 0, 0, 0)
    #
    # # 物体信息
    # wood_box1 = Object_parameters()
    # wood_box1.name = "box"
    # wood_box1.size = (0.05, 0.05, 0.1)
    # wood_box1.pose = (1.03, 0, 0.95, 0, 0, 0)
    #
    # # 盘子信息
    # plate = Object_parameters()
    # plate.name = "plate"
    # plate.size = (0.15, 0.4, 0.04)
    # plate.pose = (1.03, 0, 1.02, 0, 0, 0)
    #
    # table_pose_stamped = mrp2a.Pose_Transform(table_pose)
    #
    # box1_pose_stamped = mrp2a.Pose_Transform(wood_box1.pose)
    # plate_pose_stamped = mrp2a.Pose_Transform(plate.pose)
    # #
    # mrp2a.Add_Box(table_name, table_pose_stamped, table_size)
    # mrp2a.Add_Box(wood_box1.name, box1_pose_stamped, wood_box1.size)
    # mrp2a.Add_Box(plate.name, plate_pose_stamped, plate.size)
    # #
    # pos_left = [0.85, 0.12, 1.02]
    # rot_left = [1.57, 0, 1.57]
    # pos_right = [0.85, -0.12, 1.02]
    # rot_right = [1.57, 0, 1.57]
    # mrp2a.dual_arm_pose_control(pos_left, rot_left, pos_right, rot_right)
    # rospy.sleep(0.5)
    #
    # mrp2a.dual_gripper_joint_control("open", "open", 0, 0)
    # mrp2a.Remove_Box(plate.name, 4)
    # rospy.sleep(0.5)
    #
    # # 靠近盘子
    # mrp2a.dual_Plan_Cartesian_Path_Inter([0.12, 0, 0], [0.12, 0, 0], 14)
    #
    # # 接触物体
    # # mrp2a.Attach_Box("left_arm", plate.name, 4)
    # # mrp2a.Attach_Box("right_arm", plate.name, 4)
    #
    # # mrp2a.Remove_Box(table_name, 4)
    # # mrp2a.Remove_Box(wood_box1.name, 4)
    #
    # mrp2a.dual_gripper_joint_control("close", "close", 0.15, 0.15)
    # rospy.sleep(0.5)
    #
    # # 双臂协调
    # pos_left = [0.98, 0.12, 1.22]
    # rot_left = [1.57, 0, 1.57]
    # pos = [-0.240, 0.000, 0.000]
    # rot = [0.001, 0.001, -0.001]
    # mrp2a.dual_arm_coordinate(pos_left, rot_left, pos, rot)
    #
    # # dual_arm = Dual_Arm_Constraints()
    # # master_pos = [0.850, 0.120, 1.080]
    # # master_quaternion = [0.440, 0.580, -0.430, 0.533]
    # # s_from_m_pos = [-0.012, -0.058, 0.232]
    # # s_from_m_quaternion = [0.005, 0.966, 0.257, -0.016]
    # #dual_arm.rot_from_quaternoin([0.440, 0.580, -0.430, 0.533])
    # #dual_arm.slaver_from_base(master_pos, master_quaternion, s_from_m_pos, s_from_m_quaternion)


    mrp2a = Mrp2a_Moveit_Control()

    # # 桌子信息
    # table_name = "table"
    # table_size = (1, 2, 0.95)
    # table_pose = (1.25, 0, 0.475, 0, 0, 0)
    #
    # # 物体1信息
    # wood_box1 = Object_parameters()
    # wood_box1.name = "box1"
    # wood_box1.size = (0.05, 0.05, 0.3)
    # wood_box1.pose = (0.85, 0, 1.1, 0, 0, 0)
    #
    # # 物体2信息
    # wood_box2 = Object_parameters()
    # wood_box2.name = "box2"
    # wood_box2.size = (0.15, 0.15, 0.02)
    # wood_box2.pose = (0.85, 0, 1.26, 0, 0, 0)
    #
    # table_pose_stamped = mrp2a.Pose_Transform(table_pose)
    #
    # box1_pose_stamped = mrp2a.Pose_Transform(wood_box1.pose)
    # box2_pose_stamped = mrp2a.Pose_Transform(wood_box2.pose)
    #
    # mrp2a.Add_Box(table_name, table_pose_stamped, table_size)
    # mrp2a.Add_Box(wood_box1.name, box1_pose_stamped, wood_box1.size)
    # mrp2a.Add_Box(wood_box2.name, box2_pose_stamped, wood_box2.size)

    pos_left = [0.85, 0.26, 1.08]
    rot_left = [-2.713, 1.503, 2.244]
    pos_right = [0.85, -0.26, 1.08]
    rot_right = [-0.837, -1.473, -1.011]
    mrp2a.dual_arm_pose_control(pos_left, rot_left, pos_right, rot_right)
    rospy.sleep(0.5)
    #
    # mrp2a.dual_gripper_joint  _control("open", "open", 0, 0)

    # mrp2a.Remove_Box(wood_box1.name, 4)
    # mrp2a.Remove_Box(wood_box2.name, 4)
    # mrp2a.Remove_Box(table_name, 4)
    # rospy.sleep(0.5)

    # 靠近碗
    mrp2a.dual_Plan_Cartesian_Path_Inter([0, -0.14, 0], [0, 0.14, 0], 17)
    rospy.sleep(2)


    main = "/home/zjy/catkin_ws/devel/lib/mrp2a_driver/dual_arm_coordinate"
    print '调用cpp'

    print '*' * 10, 'begin', '*' * 10
    os.system(main)
    print '*' * 10, 'end', '*' * 10
    # 双臂协调
    # 向上
    # pos_left = [0.85, 0.12, 1.28]
    # rot_left = [-2.713, 1.503, 2.244]
    # pos = [-0.013, -0.059, 0.232]
    # rot = [2.620, -0.031, 3.140]
    # mrp2a.dual_arm_coordinate(pos_left, rot_left, pos, rot)

    # 向下
    # pos_left = [0.85, 0.12, 1.08]
    # rot_left = [-2.713, 1.503, 2.244]
    # mrp2a.dual_arm_coordinate(pos_left, rot_left, pos, rot)

    # # 向右
    # pos_left = [0.85, 0.02, 1.08]
    # rot_left = [-2.713, 1.503, 2.244]
    # pos = [-0.013, -0.059, 0.232]
    # rot = [2.620, -0.031, 3.140]
    # mrp2a.dual_arm_coordinate(pos_left, rot_left, pos, rot)
    # rospy.sleep(10)
    #
    # # 斜上
    # pos_left = [0.85, 0.22, 1.28]
    # rot_left = [-2.713, 1.503, 2.244]
    # mrp2a.dual_arm_coordinate(pos_left, rot_left, pos, rot)

    # dual_arm.master_from_base([0.440, 0.580, -0.430, 0.533])
    # # 主臂相对base_link
    # master_quaternion_init = [0.440, 0.580, -0.430, 0.533]
    # # 从臂相对base_link
    # slaver_quaternion_init = [0.560, 0.391, 0.567, -0.460]
    # # 从臂相对主臂
    # relative_quaternion_init = [0.005, 0.966, 0.257, -0.016]
    #
    # #rpy1 = transformations.euler_from_quaternion(quaternion1)
    #
    # master_matrix_init = transformations.quaternion_matrix(master_quaternion_init)
    # slaver_matrix_init = transformations.quaternion_matrix(slaver_quaternion_init)
    # relative_matrix_init = transformations.quaternion_matrix(relative_quaternion_init)
    #
    # # print "matrix1 = ", master_matrix_init
    # # print "matrix2 = ", slaver_matrix_init
    # # print "matrix3 = ", relative_matrix_init
    #
    # slaver_rpy_init = transformations.euler_from_matrix(dot(master_matrix_init, relative_matrix_init))
    #
    # print "rpy3 = ", slaver_rpy_init
    #
    # # 主臂相对base_link
    # master_quaternion_goal = [0.440, 0.580, -0.430, 0.533]
    # # 从臂相对base_link
    # slaver_quaternion_goal = [0.560, 0.391, 0.567, -0.460]
    # # 从臂相对主臂
    # relative_quaternion_goal = [0.005, 0.966, 0.257, -0.016]
    #
    # master_matrix_goal = transformations.quaternion_matrix(master_quaternion_goal)
    # slaver_matrix_goal = transformations.quaternion_matrix(slaver_quaternion_goal)
    # relative_matrix_goal = transformations.quaternion_matrix(relative_quaternion_goal)

    # L_ee = [-0.355, 0.613, 0.352, 0.612]
    # L0 = [-0.355, 0.613, 0.352, 0.612]
    # L_ee_L0 = [0, 0, 0, 1]
    #
    # # 左臂末端相对于base_link的变换矩阵
    # L_ee_matrix = transformations.quaternion_matrix(L_ee)
    # # 左臂基坐标系相对于base_link的变换矩阵
    # L0_matrix = transformations.quaternion_matrix(L0)
    # # 左臂末端相对于左臂基坐标系的变换矩阵
    # L_ee_L0_matrix = transformations.quaternion_matrix(L_ee_L0)
    #
    # L_ee_matrix[0][3] = 0.863
    # L_ee_matrix[1][3] = 0.879
    # L_ee_matrix[2][3] = 1.388
    #
    # L0_matrix[0][3] = 0.402
    # L0_matrix[1][3] = 0.080
    # L0_matrix[2][3] = 1.390
    #
    # L_ee_L0_matrix[0][3] = 0
    # L_ee_L0_matrix[1][3] = 0
    # L_ee_L0_matrix[2][3] = 0.922
    #
    # print L_ee_matrix
    # print L0_matrix
    # print L_ee_L0_matrix
    #
    # print numpy.dot(L0_matrix, L_ee_L0_matrix)

    # print transformations.quaternion_matrix([-0.354, 0.612, 0.354, 0.612])
    # print transformations.quaternion_matrix([0.612, -0.354, 0.612, 0.354])
    # print transformations.quaternion_from_matrix(a)
    # print transformations.quaternion_from_matrix(b)

    # pos_right = [0.85, -0.12, 1.28]
    # rot_right = [-0.837, -1.473, -1.011]
    # pos = [-0.020, -0.065, 0.230]
    # rot = [2.620, 0.027, 3.126]
    # mrp2a.dual_arm_coordinate2(pos_right, rot_right, pos, rot)


    # pos_left = [0.85, 0.12, 1.08]
    # rot_left = [-2.713, 1.503, 2.244]
    # pos_right = [0.85, -0.12, 1.08]
    # rot_right = [-0.837, -1.473, -1.011]
    # mrp2a.dual_arm_pose_control(pos_left, rot_left, pos_right, rot_right)
    # rospy.sleep(0.5)
    #
    # mrp2a.dual_gripper_joint_control("open", "open", 0, 0)
    #
    # #mrp2a.Remove_Box(wood_box1.name, 4)
    # #mrp2a.Remove_Box(wood_box2.name, 4)
    # #mrp2a.Remove_Box(table_name, 4)
    # rospy.sleep(0.5)
    #
    # # 靠近盘子
    # #mrp2a.Plan_Cartesian_Path("right_arm", [0, 0.14, 0])
    #
    # pos_right = [0, 0, 0.15]
    # pos = [-0.020, -0.065, 0.230]
    # rot = [2.620, 0.027, 3.126]
    # mrp2a.dual_Plan_Cartesian_Path_Inter([0, 0, 0.15], [0, 0, 0.15], 17)
