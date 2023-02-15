#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import geometry_msgs.msg
from tf import transformations
from mrp2a_moveit_control_full import Mrp2a_Moveit_Control, Object_parameters

# 双臂端碗
if __name__ == '__main__':
    mrp2a = Mrp2a_Moveit_Control()

    # 桌子信息
    table_name = "table"
    table_size = (1, 2, 1)
    table_pose = (1.25, 0, 0.5, 0, 0, 0)

    # 物体1信息
    wood_box1 = Object_parameters()
    wood_box1.name = "box1"
    wood_box1.size = (0.1, 0.1, 0.2)
    wood_box1.pose = (0.85, 0, 1.1, 0, 0, 0)

    # 物体2信息
    wood_box2 = Object_parameters()
    wood_box2.name = "box2"
    wood_box2.size = (0.15, 0.15, 0.04)
    wood_box2.pose = (0.35, 0, 1.22, 0, 0, 0)

    table_pose_stamped = mrp2a.Pose_Transform(table_pose)

    box1_pose_stamped = mrp2a.Pose_Transform(wood_box1.pose)
    box2_pose_stamped = mrp2a.Pose_Transform(wood_box2.pose)

    mrp2a.Add_Box(table_name, table_pose_stamped, table_size)
    mrp2a.Add_Box(wood_box1.name, box1_pose_stamped, wood_box1.size)
    mrp2a.Add_Box(wood_box2.name, box2_pose_stamped, wood_box2.size)

    pos_left = [0.85, 0.26, 1.12]
    rot_left = [-2.713, 1.503, 2.244]
    pos_right = [0.85, -0.26, 1.13]
    rot_right = [-0.837, -1.473, -1.011]
    # pos_left = [0.95, 0.12, 1.18]
    # rot_left = [-2.713, 1.503, 2.244]
    # pos_right = [0.95, -0.12, 1.185]
    # rot_right = [-0.837, -1.473, -1.011]
    mrp2a.dual_arm_pose_control(pos_left, rot_left, pos_right, rot_right)
    rospy.sleep(0.5)

    # mrp2a.dual_gripper_joint_control("open", "open", 0, 0)
    #
    mrp2a.Remove_Box(wood_box1.name, 4)
    mrp2a.Remove_Box(wood_box2.name, 4)
    # #mrp2a.Remove_Box(table_name, 4)
    # rospy.sleep(0.5)
    #
    # 靠近碗
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
    #
    # 离开碗
    mrp2a.dual_Plan_Cartesian_Path_Inter([0, 0.14, 0], [0, -0.14, 0], 17)
