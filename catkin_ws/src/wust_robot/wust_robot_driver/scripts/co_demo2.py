#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import geometry_msgs.msg
from tf import transformations
from mrp2a_moveit_control_full import Mrp2a_Moveit_Control, Object_parameters
from workspace import WorkSpace


# 双臂端盘子
if __name__ == '__main__':
    mrp2a = Mrp2a_Moveit_Control()

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
    # plate.size = (0.15, 0.2, 0.04)
    # plate.pose = (1.03, 0.1, 1.02, 0, 0, 0)
    #
    # plate2 = Object_parameters()
    # plate2.name = "plate2"
    # plate2.size = (0.15, 0.2, 0.04)
    # plate2.pose = (1.03, -0.1, 1.02, 0, 0, 0)
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

    pos_left = [0.85, 0.12, 1.02]
    rot_left = [1.57, 0, 1.57]
    pos_right = [0.85, -0.12, 1.02]
    rot_right = [1.57, 0, 1.57]
    mrp2a.dual_arm_pose_control(pos_left, rot_left, pos_right, rot_right)
    rospy.sleep(0.5)

    mrp2a.dual_gripper_joint_control("open", "open", 0, 0)

    # mrp2a.Remove_Box(wood_box1.name, 4)
    # #mrp2a.Remove_Box(wood_box2.name, 4)
    # mrp2a.Remove_Box(table_name, 4)
    # rospy.sleep(0.5)

    # 靠近盘子
    mrp2a.dual_Plan_Cartesian_Path_Inter([0.1, 0, 0], [0.1, 0, 0], 12)

    # mrp2a.Attach_Box("left_arm", plate.name, 4)
    # mrp2a.Attach_Box("right_arm", plate2.name, 4)
    # #
    # mrp2a.Remove_Box(table_name, 4)
    # mrp2a.Remove_Box(wood_box1.name, 4)
    # #
    mrp2a.dual_gripper_joint_control("close", "close", 0.15, 0.15)

    pos_obj = [0.95, 0, 1.02]
    mrp2a.dual_arm_coordinate2(pos_obj, 16)

# pos_left = [0.95, 0.15, 1.17]
    # rot_left = [1.57, -0.5, 1.57]
    # pos_right = [0.95, -0.15, 1.17]
    # rot_right = [1.57, -0.5, 1.57]

    # pos_left = [0.95, 0.15, 1.17]
    # rot_left = [1.563, -0.5, 1.520]
    # pos_right = [0.95, -0.127, 1.100]
    # rot_right = [1.595, -0.460, 1.520]
    # mrp2a.dual_arm_pose_control_constraints(pos_left, rot_left, pos_right, rot_right)
    #
    #端盘
    #mrp2a.dual_Plan_Cartesian_Path_Inter([0, 0, 0.15], [0, 0, 0.15], 17)
    #
    # rospy.sleep(10)
    # mrp2a.Detach_Box("left_arm", plate.name, 4)
    # mrp2a.Detach_Box("right_arm", plate2.name, 4)
    # mrp2a.Remove_Box(plate.name, 4)
    # mrp2a.Remove_Box(plate2.name, 4)


    # # # 斜上
    # # mrp2a.dual_Plan_Cartesian_Path_Inter([0, 0.1, 0.2], [0, 0.1, 0.2], 28)
    # #
    # # 向前
    # mrp2a.dual_Plan_Cartesian_Path_Inter([0.2, 0, 0], [0.2, 0, 0], 22)
    #
    # # 向后
    # mrp2a.dual_Plan_Cartesian_Path_Inter([-0.2, 0, 0], [-0.2, 0, 0], 22)
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
    # mrp2a.dual_Plan_Cartesian_Path_Inter([0, 0, -0.2], [0, 0, -0.2], 22)
    #
    # # 离开碗
    # mrp2a.dual_Plan_Cartesian_Path_Inter([0, 0.14, 0], [0, -0.14, 0], 17)

    #mrp2a.dual_arm_coordinate2([1.03, 0 ,1.02],2)