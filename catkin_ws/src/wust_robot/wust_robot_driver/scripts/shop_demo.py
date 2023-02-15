#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import geometry_msgs.msg
import moveit_msgs.msg
from tf import transformations
from mrp2a_moveit_control_full import Mrp2a_Moveit_Control, Object_parameters

# 分拣demo
if __name__ == '__main__':
    mrp2a = Mrp2a_Moveit_Control()

    # 桌子1信息
    table1_name = "table1"
    table1_size = (0.5, 2, 1)
    table1_pose = (1.3, 0, 0.5, 0, 0, 0)

    # 桌子2信息
    table2_name = "table2"
    table2_size = (0.3, 1.5, 0.9)
    table2_pose = (0.9, 0, 0.45, 0, 0, 0)


    # 盘子信息
    plate = Object_parameters()
    plate.name = "plate"
    plate.size = (0.15, 0.15, 0.10)
    plate.pose = (0.85, 0, 1.21, 0, 0, 0)

    # 垫高物品信息
    wood_box1 = Object_parameters()
    wood_box1.name = "box1"
    wood_box1.size = (0.05, 0.05, 0.3)
    wood_box1.pose = (0.85, 0, 1.05, 0, 0, 0)

    # 物品信息
    box = Object_parameters()
    box.name = "box"
    box.size = (0.05, 0.05, 0.15)
    box.pose = (1.18, 0.28, 1.075, 0, 0, 0)

    # 物品2信息
    box2 = Object_parameters()
    box2.name = "box2"
    box2.size = (0.05, 0.05, 0.05)
    box2.pose = (1.18, -0.28, 1.025, 0, 0, 0)

    table1_pose_stamped = mrp2a.Pose_Transform(table1_pose)
    table2_pose_stamped = mrp2a.Pose_Transform(table2_pose)
    plate_pose_stamped = mrp2a.Pose_Transform(plate.pose)
    wood_box1_stamped = mrp2a.Pose_Transform(wood_box1.pose)
    box_pose_stamped = mrp2a.Pose_Transform(box.pose)
    box2_pose_stamped = mrp2a.Pose_Transform(box2.pose)

    mrp2a.Add_Box(table1_name, table1_pose_stamped, table1_size)
    mrp2a.Add_Box(table2_name, table2_pose_stamped, table2_size)
    mrp2a.Add_Box(box.name, box_pose_stamped, box.size)
    mrp2a.Add_Box(box2.name, box2_pose_stamped, box2.size)
    mrp2a.Add_Box(plate.name, plate_pose_stamped, plate.size)
    mrp2a.Add_Box(wood_box1.name, wood_box1_stamped, wood_box1.size)

    # # 左臂抓取物体动作分解
    # pos = [0.95, 0.28, 1.08]
    # rot = [-1.57, 1.57, -1.57]
    # mrp2a.arm_pose_control("left_arm", pos, rot)
    #
    # mrp2a.gripper_joint_control("left_gripper", "open", 0)
    #
    # mrp2a.Plan_Cartesian_Path_Inter("left_arm", [0.18, 0, 0], 20)
    #
    # mrp2a.gripper_joint_control("left_gripper", "close", 0.35)
    #
    # pos = [0.836, 0, 1.417]
    # rot = [3.040, 0.638, 3.082]
    # mrp2a.arm_pose_control("left_arm", pos, rot)
    #
    # mrp2a.gripper_joint_control("left_gripper", "open", 0)
    #
    # mrp2a.Detach_Box("left_arm", box.name, 4)
    # mrp2a.Remove_Box(box.name, 4)
    #
    # mrp2a.Plan_Cartesian_Path_Inter("left_arm", [0, 0.1, 0], 12)
    #
    # 右臂抓取物体动作分解
    pos = [0.974, -0.28, 1.149]
    rot = [-3.14, -1.1, 0]
    mrp2a.arm_pose_control("right_arm", pos, rot)

    mrp2a.gripper_joint_control("right_gripper", "open", 0)
    rospy.sleep(0.5)

    mrp2a.Plan_Cartesian_Path_Inter("right_arm", [0.16, 0, -0.09], 20)
    rospy.sleep(0.5)

    mrp2a.Attach_Box("right_arm", box2.name, 4)
    mrp2a.gripper_joint_control("right_gripper", "close", 0.35)
    rospy.sleep(0.5)

    pos = [0.837, 0, 1.298]
    rot = [-1.094, -1.514, -0.905]
    mrp2a.arm_pose_control("right_arm", pos, rot)

    mrp2a.gripper_joint_control("right_gripper", "open", 0)

    mrp2a.Detach_Box("right_arm", box2.name, 4)
    mrp2a.Remove_Box(box2.name, 4)

    joint_left = [0, 0, 0, 0, 0, 0, 0]
    joint_right = [0, 0, 0, 0, 0, 0, 0]
    mrp2a.dual_arm_joint_control(joint_left, joint_right)

    # 双臂端碗
    pos_left = [0.85, 0.26, 1.08]
    rot_left = [-2.713, 1.503, 2.244]
    pos_right = [0.85, -0.26, 1.08]
    rot_right = [-0.837, -1.473, -1.011]
    mrp2a.dual_arm_pose_control(pos_left, rot_left, pos_right, rot_right)
    rospy.sleep(1)

    mrp2a.dual_gripper_joint_control("open", "open", 0, 0)

    mrp2a.Remove_Box(plate.name, 4)
    mrp2a.Remove_Box(wood_box1.name, 4)

    # 靠近碗
    mrp2a.dual_Plan_Cartesian_Path_Inter([0, -0.14, 0], [0, 0.14, 0], 17)

    # 端碗
    mrp2a.dual_Plan_Cartesian_Path_Inter([0, 0, 0.2], [0, 0, 0.2], 22)

    # # 斜上
    # mrp2a.dual_Plan_Cartesian_Path_Inter([0, 0.1, 0.2], [0, 0.1, 0.2], 28)

    # 向前
    mrp2a.dual_Plan_Cartesian_Path_Inter([0.2, 0, 0], [0.2, 0, 0], 22)

    # 向后
    mrp2a.dual_Plan_Cartesian_Path_Inter([-0.2, 0, 0], [-0.2, 0, 0], 22)

    # 向左
    mrp2a.dual_Plan_Cartesian_Path_Inter([0, 0.1, 0], [0, 0.1, 0], 12)

    # 还原
    mrp2a.dual_Plan_Cartesian_Path_Inter([0, -0.1, 0], [0, -0.1, 0], 12)

    # 向右
    mrp2a.dual_Plan_Cartesian_Path_Inter([0, -0.1, 0], [0, -0.1, 0], 12)

    # 还原
    mrp2a.dual_Plan_Cartesian_Path_Inter([0, 0.1, 0], [0, 0.1, 0], 12)

    # 放碗
    mrp2a.dual_Plan_Cartesian_Path_Inter([0, 0, -0.2], [0, 0, -0.2], 22)

    # 离开碗
    mrp2a.dual_Plan_Cartesian_Path_Inter([0, 0.14, 0], [0, -0.14, 0], 17)

