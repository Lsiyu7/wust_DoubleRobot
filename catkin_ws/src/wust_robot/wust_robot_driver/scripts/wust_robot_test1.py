#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
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

    # pos_left = [0.30, 0.26, 1.12]
    # rot_left = [-2.713, 1.503, 2.244]
    # pos_right = [0.30, -0.26, 1.12]
    # rot_right = [-0.837, -1.473, -1.011]
    # wust.dual_arm_pose_control(pos_left, rot_left, pos_right, rot_right)
    # rospy.sleep(0.5)


    # 关节空间运动
    # joint_left = [-0.8938021606637827, 0.6477844766050458, 2.733789242970616, -1.7156256211236638,
    # -2.7000346418110306, -1.2555944537843917, 3.049923976357862,]
    # joint_right = [-0.4853739151199443, -1.0242822820735453, 1.9481379075334981, 1.6877580834950354,
    #                0.6244975538631738, -0.9781498956736501, 0.43103792813840247]
    # wust.dual_arm_joint_control(joint_left, joint_right)


    # 点到点运动
    # joint_left = [2.1688119870559923, -0.38854871092445187, -1.3825736780268603, -0.5555959631029586,
    # 0.8754500787822543, -1.6814338880570388, -1.7892983178713466]
    # joint_right = [1.9799830774432365, 0.22708586625711025, -0.3234329310594193, 0.6671038642725939,
    # 3.049044000356595, -1.4705617306816627, 1.6303825787598125]
    # mrp2a.dual_arm_joint_control(joint_left, joint_right)


    # # 抓取和放置
    #
    # # 台子1信息
    # table1_name = "table1"
    # table1_size = (1, 3, 0.8)
    # table1_pose = (0.9, 0, 0.45, 0, 0, 0)
    # table1_pose_stamped = wust.Pose_Transform(table1_pose)
    #
    #
    # # 物体1信息
    # box1_name = "box1"
    # box1_size = (0.05, 0.05, 0.3)
    # box1_pose = (0.53, 0.3, 0.95, 0, 0, 0)
    # box1_pose_stamped = wust.Pose_Transform(box1_pose)
    #
    # # 物体2信息
    # box2_name = "box2"
    # box2_size = (0.05, 0.05, 0.3)
    # box2_pose = (0.53, -0.3, 0.95, 0, 0, 0)
    # box2_pose_stamped = wust.Pose_Transform(box2_pose)
    #
    #
    # wust.Add_Box(table1_name, table1_pose_stamped, table1_size)
    # wust.Add_Box(box1_name, box1_pose_stamped, box1_size)
    # wust.Add_Box(box2_name, box2_pose_stamped, box2_size)
    #
    # wust.setColor(box1_name, 1.0, 0, 0, 1.0)
    # wust.setColor(box2_name, 0, 0, 1.0, 1.0)
    # wust.sendColors()
    #
    # pos_left = [0.4, 0.3, 0.95]
    # rot_left = [1.57, 1.57, 1.57]
    # pos_right = [0.4, -0.3, 0.95]
    # rot_right = [-1.57, -1.57, -1.57]
    # wust.dual_arm_pose_control(pos_left, rot_left, pos_right, rot_right)
    # rospy.sleep(0.5)
    #
    # wust.dual_gripper_joint_control("open", "open", 0.35, 0.35)
    # rospy.sleep(5)
    #
    # wust.dual_Plan_Cartesian_Path_Inter([0.12, 0, 0], [0.12, 0, 0], 14)
    #
    # wust.Attach_Box("left_arm", box1_name, 4)
    # wust.Attach_Box("right_arm", box2_name, 4)
    #
    # wust.dual_gripper_joint_control("close", "close", 0.15, 0.15)
    # rospy.sleep(5)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, 0, 0.10], 12)
    #
    # pos_left = [0.4, 0.3, 1.07]
    # rot_left = [1.57, 1.57, 1.57]
    # pos_right = [0.4, -0.3, 1.15]
    # rot_right = [-1.57, -1.57, -1.57]
    # wust.dual_arm_pose_control(pos_left, rot_left, pos_right, rot_right)

    #双臂协调
    # joint_left = [2.1688119870559923, -0.38854871092445187, -1.3825736780268603, -0.5555959631029586,
    # 0.8754500787822543, -1.6814338880570388, -1.7892983178713466]
    # joint_right = [1.9799830774432365, 0.22708586625711025, -0.3234329310594193, 0.6671038642725939,
    # 3.049044000356595, -1.4705617306816627, 1.6303825787598125]
    # wust.dual_arm_joint_control(joint_left, joint_right)
    #
    # wust.Plan_Cartesian_Path("left_arm", [0, -0.14, 0])
    # wust.Plan_Cartesian_Path("right_arm", [0, 0.14, 0])
    # wust.Plan_Cartesian_Path("left_arm", [0, 0, -0.18])
    # wust.Plan_Cartesian_Path("right_arm", [0, 0, -0.18])
    #jr1 =((80) * (math.pi/180))
    #jr2 =((-80) * (math.pi/180))
    #jr3 =((0) * (math.pi/180))
   # jr4 =(0) * (math.pi/180)
    #jr5 =0.0
    #jr6 =0.0
    #jr7 =0.0
    #joint_right = [jr1,jr2,jr3,jr4,jr5,jr6,jr7]
    #joint_left = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
    #joint_left = [2.5867593028479696, -0.7418074901756775, -1.445051931257681, -0.7484279142166841,
    #             0.8479679097924393, -1.301719679390592, -1.4951954798821117]
    #joint_right = [2.3273010194406734, 0.5989473064686228, -0.3186432656912217, 0.6810080426312117,
    #               2.8454118270822732, -1.230305673371495, 2.0919275742255863]
    #wust.dual_arm_joint_control(joint_left, joint_right)
    # main = "/home/zjy/catkin_ws/devel/lib/wust_robot_driver/dual_arm_coordination"
    # print '调用cpp'
    # 
    # print '*' * 10, 'begin', '*' * 10
    # os.system(main)
    # print '*' * 10, 'end', '*' * 10
