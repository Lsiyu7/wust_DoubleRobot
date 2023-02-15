#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
import geometry_msgs.msg
import moveit_msgs.msg
from tf import transformations
from wust_robot_moveit_control_full import Wust_Robot_Moveit_Control, Object_parameters
from move import MOVE

# 移动抓取
if __name__ == '__main__':
    mrp2a = Wust_Robot_Moveit_Control()

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
    # 盘子信息
    # plate = Object_parameters()
    # plate.name = "plate"
    # plate.size = (0.15, 0.24, 0.04)
    # plate.pose = (0.451, 0.170, 1.120, 0, 0, 0)
    #
    # plate2 = Object_parameters()
    # plate2.name = "plate2"
    # plate2.size = (0.15, 0.24, 0.04)
    # plate2.pose = (0.451, -0.07, 1.120, 0, 0, 0)
    #
    # table_pose_stamped = mrp2a.Pose_Transform(table_pose)
    #
    # box1_pose_stamped = mrp2a.Pose_Transform(wood_box1.pose)
    # box2_pose_stamped = mrp2a.Pose_Transform(plate.pose)
    # box3_pose_stamped = mrp2a.Pose_Transform(plate2.pose)
    #
    # mrp2a.Add_Box(table_name, table_pose_stamped, table_size)
    # mrp2a.Add_Box(wood_box1.name, box1_pose_stamped, wood_box1.size)
    # mrp2a.Add_Box(plate.name, box2_pose_stamped, plate.size)
    # mrp2a.Add_Box(plate2.name, box3_pose_stamped, plate2.size)

    joint_left = [-1.2449394588267104, 0.7267167390338871, -2.7558013889594584, -1.5485230372048226,
                  -2.7618887530979057, -1.3956576699697596, 2.5138084457886407]
    joint_right = [-1.3889021011863294, -0.8357131346902591, 0.6378633155083309, -1.516311710545823,
                   -2.8896298042591093, -1.4522432336642295, -0.7718102472528701]
    mrp2a.dual_arm_joint_control(joint_left, joint_right)

    pos_left = [0.35, 0.12, 0.95]
    rot_left = [1.53, 0, 1.57]
    pos_right = [0.35, -0.12, 0.95]
    rot_right = [1.53, 0, 1.57]
    mrp2a.dual_arm_pose_control(pos_left, rot_left, pos_right, rot_right)
    rospy.sleep(0.5)

    mrp2a.dual_gripper_joint_control("open", "open", 0, 0)

    #端盘

    mrp2a.dual_Plan_Cartesian_Path_Inter([0.10, 0, 0], [0.10, 0, 0],12)
    mrp2a.dual_gripper_joint_control("close", "close", 0.15, 0.15)
    rospy.sleep(2)
    mrp2a.dual_Plan_Cartesian_Path_Inter([0, 0, 0.1], [0, 0, 0.1],12)

    rospy.sleep(2)
    # MOVE()


    # joint_left = [-1.0278344379066373, 0.8967719000829083, -2.7278648427233705, -1.2059770001387564,
    #               -2.6741106823242, -1.3602497069333348, 2.286220218355717]
    # joint_right = [-1.1430640887897392, -0.9910793741804279, 0.6439530278739244, -1.183955325316464,
    #                -2.835797090379469, -1.4101664181829394, -0.9586746278335323,]
    # mrp2a.dual_arm_joint_control(joint_left, joint_right)

    # pos_left = [0.35, 0.12, 0.856]
    # rot_left = [1.57, 0, 1.57]
    # pos_right = [0.35, -0.12, 0.856]
    # rot_right = [1.57, 0, 1.57]
    # mrp2a.dual_arm_pose_control(pos_left, rot_left, pos_right, rot_right)
    # rospy.sleep(0.5)

    # 靠近盘子
    # rospy.sleep(1)

    # mrp2a.Attach_Box("left_arm", plate.name, 4)
    # mrp2a.Attach_Box("right_arm", plate2.name, 4)
    # #
    # mrp2a.Remove_Box(table_name, 4)
    # mrp2a.Remove_Box(wood_box1.name, 4)

    # rospy.sleep(2)
    # mrp2a.dual_Plan_Cartesian_Path_Inter([0, 0.30, 0.22], [0, 0.30, 0.22],40)



