#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import geometry_msgs.msg
import moveit_msgs.msg
from tf import transformations
from mrp2a_moveit_control_full import Mrp2a_Moveit_Control, Object_parameters
from move import MOVE

# 移动抓取放置demo
if __name__ == '__main__':
    mrp2a = Mrp2a_Moveit_Control()

    # 桌子1信息
    table1_name = "table1"
    table1_size = (0.5, 2, 1.05)
    table1_pose = (1.3, 0, 0.525, 0, 0, 0)

    # 桌子2信息
    table2_name = "table2"
    table2_size = (0.3, 1.5, 0.85)
    table2_pose = (0.9, 0, 0.425, 0, 0, 0)

    # 桌子3信息
    table3_name = "table3"
    table3_size = (0.5, 0.5, 0.96)
    table3_pose = (1.0, 0, 0.48, 0, 0, 0)


    # 盘子信息
    plate = Object_parameters()
    plate.name = "plate"
    plate.size = (0.30, 0.30, 0.1)
    plate.pose = (0.85, 0, 1.1, 0, 0, 0)

    # 垫高物品信息
    wood_box1 = Object_parameters()
    wood_box1.name = "box1"
    wood_box1.size = (0.05, 0.05, 0.2)
    wood_box1.pose = (0.85, 0, 0.95, 0, 0, 0)

    # 物品信息
    # box = Object_parameters()
    # box.name = "box"
    # box.size = (0.05, 0.05, 0.15)
    # box.pose = (1.18, -0.28, 1.1, 0, 0, 0)

    table1_pose_stamped = mrp2a.Pose_Transform(table1_pose)
    table2_pose_stamped = mrp2a.Pose_Transform(table2_pose)
    table3_pose_stamped = mrp2a.Pose_Transform(table3_pose)
    plate_pose_stamped = mrp2a.Pose_Transform(plate.pose)
    wood_box1_stamped = mrp2a.Pose_Transform(wood_box1.pose)
    #box_pose_stamped = mrp2a.Pose_Transform(box.pose)

    mrp2a.Add_Box(table1_name, table1_pose_stamped, table1_size)
    mrp2a.Add_Box(table2_name, table2_pose_stamped, table2_size)
    #mrp2a.Add_Box(box.name, box_pose_stamped, box.size)
    mrp2a.Add_Box(plate.name, plate_pose_stamped, plate.size)
    mrp2a.Add_Box(wood_box1.name, wood_box1_stamped, wood_box1.size)

    #mrp2a.gripper_joint_control("right_gripper", "open", 0)

    pos = [0.975, -0.28, 1.1]
    #rot = [-1.57, -1.57, -1.57]
    rot = [1.57, -1.57, 1.57]
    mrp2a.arm_pose_control("right_arm", pos, rot)

    mrp2a.gripper_joint_control("right_gripper", "open", 0)
    rospy.sleep(0.5)

    mrp2a.Plan_Cartesian_Path_Inter("right_arm", [0.16, 0, 0], 20)

    #mrp2a.Attach_Box("right_arm", box.name, 4)
    mrp2a.gripper_joint_control("right_gripper", "close", 0.35)

    mrp2a.Plan_Cartesian_Path_Inter("right_arm", [0, 0, 0.15], 18)
    rospy.sleep(0.5)

    pos = [0.837, 0, 1.28]
    rot = [-1.094, -1.514, -0.905]
    mrp2a.arm_pose_control("right_arm", pos, rot)

    #mrp2a.Plan_Cartesian_Path_Inter("right_arm", [0, 0, 0.06], 8)

    mrp2a.gripper_joint_control("right_gripper", "open", 0)

    #mrp2a.Detach_Box("right_arm", box.name, 4)
    #mrp2a.Remove_Box(box.name, 4)

    mrp2a.arm_joint_control("right_arm", [0, 0, 0, 0, 0, 0, 0])

    pos_left = [0.85, 0.26, 1.08]
    rot_left = [-2.713, 1.503, 2.244]
    pos_right = [0.85, -0.26, 1.09]
    rot_right = [-0.837, -1.473, -1.011]
    mrp2a.dual_arm_pose_control(pos_left, rot_left, pos_right, rot_right)
    rospy.sleep(0.5)

    mrp2a.dual_gripper_joint_control("open", "open", 0, 0)

    mrp2a.Remove_Box(wood_box1.name, 4)
    mrp2a.Remove_Box(plate.name, 4)
    #mrp2a.Remove_Box(table_name, 4)
    rospy.sleep(0.5)

    # 靠近碗
    mrp2a.dual_Plan_Cartesian_Path_Inter([0, -0.14, 0], [0, 0.14, 0], 17)

    # 端碗
    mrp2a.dual_Plan_Cartesian_Path_Inter([0, 0, 0.2], [0, 0, 0.2], 22)

    # 移动
    MOVE()

    mrp2a.Remove_Box(table1_name, 4)
    mrp2a.Remove_Box(table2_name, 4)
    mrp2a.Add_Box(table3_name, table3_pose_stamped, table3_size)

    # 向前
    # mrp2a.dual_Plan_Cartesian_Path_Inter([0.2, 0, 0], [0.2, 0, 0], 22)

    # 放碗
    mrp2a.dual_Plan_Cartesian_Path_Inter([0, 0, -0.14], [0, 0, -0.14], 17)

    # 离开碗
    mrp2a.dual_Plan_Cartesian_Path_Inter([0, 0.14, 0], [0, -0.14, 0], 17)
    mrp2a.Remove_Box(table3_name, 4)

    # mrp2a.dual_arm_joint_control([0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0])



