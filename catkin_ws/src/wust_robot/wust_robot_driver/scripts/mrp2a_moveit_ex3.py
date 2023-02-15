#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import geometry_msgs.msg
from tf import transformations
from mrp2a_moveit_control_full import Mrp2a_Moveit_Control, Object_parameters

# 双臂抓杯
if __name__ == '__main__':
    mrp2a = Mrp2a_Moveit_Control()

    # 桌子信息
    table_name = "table"
    table_size = (1, 2, 1.15)
    table_pose = (1.508299, 0, 0.575, 0, 0, 0)
    #
    # # 物体1信息
    # wood_box1 = Object_parameters()
    # wood_box1.name = "box1"
    # wood_box1.size = (0.05, 0.05, 0.4)
    # wood_box1.pose = (1.15185, 0, 1.34983, 0, 0, 0.7854)
    #
    table_pose_stamped = mrp2a.Pose_Transform(table_pose)
    #
    # box1_pose_stamped = mrp2a.Pose_Transform(wood_box1.pose)
    #
    mrp2a.Add_Box(table_name, table_pose_stamped, table_size)
    # mrp2a.Add_Box(wood_box1.name, box1_pose_stamped, wood_box1.size)

    pos_left = [1.0518, 0.11, 1.42925]
    rot_left = [2.355, -1.57, 0]
    pos_right = [1.0518, -0.11, 1.26925]
    rot_right = [-0.785, -1.57, -1.57]
    mrp2a.dual_arm_pose_control(pos_left, rot_left, pos_right, rot_right)
    rospy.sleep(0.5)

    mrp2a.dual_gripper_joint_control("open", "open", 0, 0)

    mrp2a.Plan_Cartesian_Path_Inter("left_arm", [0.10, -0.10, 0], 15)
    mrp2a.Plan_Cartesian_Path_Inter("right_arm", [0.09, 0.09, 0], 15)
    rospy.sleep(0.5)

    mrp2a.dual_gripper_joint_control("close", "close", 0.14, 0.14)
    rospy.sleep(0.5)

    mrp2a.dual_Plan_Cartesian_Path_Inter([0, 0, 0.1], [0, 0, 0.1], 12)

    mrp2a.dual_Plan_Cartesian_Path_Inter([0, 0.1, 0], [0, 0.1, 0], 12)

    mrp2a.dual_Plan_Cartesian_Path_Inter([0, -0.15, 0], [0, -0.15, 0], 18)



