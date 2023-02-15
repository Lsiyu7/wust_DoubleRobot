#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import geometry_msgs.msg
from tf import transformations
from mrp2a_moveit_control_full import Mrp2a_Moveit_Control, Object_parameters
import visualization_msgs.msg
import numpy as np
import moveit_msgs.msg
from std_msgs.msg import String

if __name__ == '__main__':
    mrp2a = Mrp2a_Moveit_Control()

    # joint_left = [0.9883204526370744, -1.3333185841293336, -1.2503535647337758, 0.9343609983898471, 3.0499947065090165,
    #                -1.182644482979481, -1.5818374736780534]
    # joint_right = [-1.750, 0.511, 1.390, -1.799, 2.548, -1.938, -3.049]
    # mrp2a.dual_arm_joint_control(joint_left, joint_right)
    # ### head and gripper control
    # mrp2a.head_joint_control()
    # mrp2a.dual_gripper_joint_control("open", "open")
    # ### dual arm move
    # joint_left = [0.9883204526370744, -1.3333185841293336, -1.2503535647337758, 0.9343609983898471, 3.0499947065090165,
    #               -1.182644482979481, -1.5818374736780534]
    # joint_right = [-1.750, 0.511, 1.390, -1.799, 2.548, -1.938, -3.049]
    # mrp2a.dual_arm_joint_control(joint_left, joint_right)
    #
    # joint_left = [1.377524982289395, -1.2841467521271515, -1.3118280174040837, 0.8639703843718953, 2.955805426153786,
    #               -0.8811142812226889, -1.7446712180583015]
    # joint_right = [-1.8343902750279355, 0.7008803269013972, 1.5915867849141438, -1.000567151816611, 2.769612714252448,
    #                -1.3157834354666562, -2.8452852284467136]
    # mrp2a.dual_arm_joint_control(joint_left, joint_right)
    #
    # mrp2a.dual_gripper_joint_control("close", "close")
    #
    # joint_left = [1.3776585200213072, -1.2840220299007408, -1.311883015081472, 0.8640111049836081, 2.955891836759683,
    #               -0.8811221489338141, -1.744711724577975]
    # joint_right = [-1.2066298204777874, 0.6353936776839273, 1.2421693452061415, -1.38667851347311, 2.6280283558139352,
    #                -1.2885791378046232, -2.938055380422374]
    # mrp2a.dual_arm_joint_control(joint_left, joint_right)
    #
    # joint_left = [1.3775515592058083, -1.284047301734626, -1.3119623481726075, 0.8640255874551706, 2.9559026275996665,
    #               -0.8811576523091817, -1.7446737856302192]
    # joint_right = [-1.1398792639085977, 1.5687949803434345, 0.5045114446042618, -0.883077056223371, 0.95918833181614,
    #                -1.4667217132863204, -1.7556693885650017]
    # mrp2a.dual_arm_joint_control(joint_left, joint_right)




    # 小论文实验

    # joint_right = [-1.750, 0.511, 1.390, -1.799, 2.548, -1.938, -3.049]
    # mrp2a.arm_joint_control("right_arm", joint_right)
    #
    # rospy.sleep(10)
    #
    # mrp2a.Plan_Cartesian_Path("right_arm", [0.2, 0, 0])
    #
    # mrp2a.Plan_Cartesian_Path("right_arm", [0, 0, 0.15])

    # target_pose.position.x = 0
    # target_pose.position.y = 0.1
    # target_pose.position.z = 0
    #
    # mrp2a.Plan_Cartesian_Path("right_arm", target_pose)

    # # 桌子信息
    # table_name = "table"
    # table_size = (1, 0.8, 0.94)
    # table_pose = (1.508299, 0, 0.47, 0, 0, 0)
    #
    # # 物体1信息
    # wood_box1 = Object_parameters()
    # wood_box1.name = "box1"
    # wood_box1.size = (0.05, 0.05, 0.3)
    # wood_box1.pose = (1.12, 0.28, 1.095, 0, 0, 0)
    # wood_box1.place_pose = (1.12401, 0.28, 1.105, 0, 0, 0)
    # wood_box1.pick_rpy_left = [[-1.57], [1.57], [-1.57]]
    # wood_box1.approach_min = 0.15
    # wood_box1.approach_desire = 0.2
    # wood_box1.approach_axis = [0, 0, 1]
    # wood_box1.retreat_min = 0.15
    # wood_box1.retreat_desire = 0.2
    # wood_box1.retreat_axis = [-1.5, 0, 0]
    # wood_box1.compensate_left = [-0.012, 0, 0]
    #
    # table_pose_stamped = mrp2a.Pose_Transform(table_pose)
    #
    # box1_pose_stamped = mrp2a.Pose_Transform(wood_box1.pose)
    # box1_place_pose_stamped = mrp2a.Pose_Transform(wood_box1.place_pose)
    #
    # mrp2a.Add_Box(table_name, table_pose_stamped, table_size)
    # mrp2a.Add_Box(wood_box1.name, box1_pose_stamped, wood_box1.size)
    #
    # # 左臂抓取物体动作分解
    # target_pose0 = geometry_msgs.msg.PoseStamped()
    # target_pose = geometry_msgs.msg.Pose()
    #
    # target_pose0.header.frame_id = 'base_link'
    # target_pose0.pose.position.x = 0.87
    # target_pose0.pose.position.y = 0.28
    # target_pose0.pose.position.z = 1.09
    # quaternion = transformations.quaternion_from_euler(-1.57, 1.57, -1.57)
    # target_pose0.pose.orientation.x = quaternion[0]
    # target_pose0.pose.orientation.y = quaternion[1]
    # target_pose0.pose.orientation.z = quaternion[2]
    # target_pose0.pose.orientation.w = quaternion[3]
    # mrp2a.arm_pose_control("left_arm", target_pose0)
    #
    # mrp2a.gripper_joint_control("left_gripper", "open", 0)
    #
    # target_pose.position.x = 0.24
    # target_pose.position.y = 0.0
    # target_pose.position.z = 0.0
    # mrp2a.Plan_Cartesian_Path("left_arm", target_pose)
    # rospy.sleep(0.5)
    #
    # mrp2a.Attach_Box("left_arm", wood_box1.name, 4)
    # mrp2a.gripper_joint_control("left_gripper", "close", 0.14)
    # rospy.sleep(0.5)
    #
    # target_pose.position.x = 0.0
    # target_pose.position.y = 0.0
    # target_pose.position.z = 0.3
    # mrp2a.Plan_Cartesian_Path("left_arm", target_pose)

    #
    # target_pose1 = geometry_msgs.msg.PoseStamped()
    # target_pose2 = geometry_msgs.msg.PoseStamped()
    # target_pose3 = geometry_msgs.msg.Pose()
    # target_pose4 = geometry_msgs.msg.Pose()
    #
    # target_pose1.header.frame_id = 'base_link'
    # target_pose1.pose.position.x = 0.85
    # target_pose1.pose.position.y = 0.26
    # target_pose1.pose.position.z = 1.23
    # quaternion = transformations.quaternion_from_euler(-2.713, 1.503, 2.244)
    # #quaternion = transformations.quaternion_from_euler(-1.761, 1.542, -3.087)
    # target_pose1.pose.orientation.x = quaternion[0]
    # target_pose1.pose.orientation.y = quaternion[1]
    # target_pose1.pose.orientation.z = quaternion[2]
    # target_pose1.pose.orientation.w = quaternion[3]
    #
    # target_pose2.header.frame_id = 'base_link'
    # target_pose2.pose.position.x = 0.87
    # target_pose2.pose.position.y = -0.28
    # target_pose2.pose.position.z = 1.09
    # quaternion = transformations.quaternion_from_euler(-1.57, -1.57, -1.57)
    # target_pose2.pose.orientation.x = quaternion[0]
    # target_pose2.pose.orientation.y = quaternion[1]
    # target_pose2.pose.orientation.z = quaternion[2]
    # target_pose2.pose.orientation.w = quaternion[3]
    #
    # mrp2a.arm_pose_control("right_arm", target_pose2)
    #
    # target_pose3.position.x = 0.25
    # target_pose3.position.y = 0
    # target_pose3.position.z = 0
    #
    # mrp2a.Plan_Cartesian_Path("right_arm", target_pose3)
    #
    # target_pose3.position.x = 0
    # target_pose3.position.y = 0.25
    # target_pose3.position.z = 0
    #
    # mrp2a.Plan_Cartesian_Path("right_arm", target_pose3)

    # #自己设定轨迹
    # target_pose1 = geometry_msgs.msg.PoseStamped()
    # target_pose2 = geometry_msgs.msg.PoseStamped()
    # target_pose3 = geometry_msgs.msg.Pose()
    # target_pose4 = geometry_msgs.msg.Pose()
    #
    # target_pose1.header.frame_id = 'base_link'
    # target_pose1.pose.position.x = 0.85
    # target_pose1.pose.position.y = 0.26
    # target_pose1.pose.position.z = 1.23
    # quaternion = transformations.quaternion_from_euler(-2.713, 1.503, 2.244)
    # # quaternion = transformations.quaternion_from_euler(-1.761, 1.542, -3.087)
    # target_pose1.pose.orientation.x = quaternion[0]
    # target_pose1.pose.orientation.y = quaternion[1]
    # target_pose1.pose.orientation.z = quaternion[2]
    # target_pose1.pose.orientation.w = quaternion[3]
    # mrp2a.arm_pose_control("left_arm", target_pose1)
    #
    # rospy.sleep(1)
    #
    # # 靠近碗
    #
    # target_pose1 = geometry_msgs.msg.PoseStamped()
    # target_pose2 = geometry_msgs.msg.PoseStamped()
    # target_pose3 = geometry_msgs.msg.PoseStamped()
    # target_pose4 = geometry_msgs.msg.PoseStamped()
    #
    # target_pose1.pose.position.x = 0.85
    # target_pose1.pose.position.y = 0.27
    # target_pose1.pose.position.z = 1.23
    # quaternion = transformations.quaternion_from_euler(-2.713, 1.503, 2.244)
    # target_pose1.pose.orientation.x = quaternion[0]
    # target_pose1.pose.orientation.y = quaternion[1]
    # target_pose1.pose.orientation.z = quaternion[2]
    # target_pose1.pose.orientation.w = quaternion[3]
    #
    # target_pose2.pose.position.x = 0.85
    # target_pose2.pose.position.y = 0.28
    # target_pose2.pose.position.z = 1.23
    # quaternion = transformations.quaternion_from_euler(-2.713, 1.503, 2.244)
    # target_pose2.pose.orientation.x = quaternion[0]
    # target_pose2.pose.orientation.y = quaternion[1]
    # target_pose2.pose.orientation.z = quaternion[2]
    # target_pose2.pose.orientation.w = quaternion[3]
    #
    # target_pose3.pose.position.x = 0.85
    # target_pose3.pose.position.y = 0.30
    # target_pose3.pose.position.z = 1.23
    # quaternion = transformations.quaternion_from_euler(-2.713, 1.503, 2.244)
    # target_pose3.pose.orientation.x = quaternion[0]
    # target_pose3.pose.orientation.y = quaternion[1]
    # target_pose3.pose.orientation.z = quaternion[2]
    # target_pose3.pose.orientation.w = quaternion[3]
    #
    # target_pose4.pose.position.x = 0.85
    # target_pose4.pose.position.y = 0.32
    # target_pose4.pose.position.z = 1.23
    # quaternion = transformations.quaternion_from_euler(-2.713, 1.503, 2.244)
    # target_pose4.pose.orientation.x = quaternion[0]
    # target_pose4.pose.orientation.y = quaternion[1]
    # target_pose4.pose.orientation.z = quaternion[2]
    # target_pose4.pose.orientation.w = quaternion[3]
    #
    #
    # target_poses = [target_pose1]
    #
    #
    # mrp2a.arm_poses_control("left_arm", target_poses)

    #自己设定轨迹

    # mrp2a.arm_pose_control("left_arm", [0.85, 0.26, 1.23], [-2.713, 1.503, 2.244])
    # mrp2a.dual_arm_pose_control([0.85, 0.26, 1.23], [-2.713, 1.503, 2.244], [0.85, -0.26, 1.23],
    #                             [-0.837, -1.473, -1.011])
    # rospy.sleep(1)
    #
    # mrp2a.dual_gripper_joint_control("open", "open", 0, 0)
    #
    # mrp2a.dual_Plan_Cartesian_Path_Inter([0, -0.14, 0], [0, 0.14, 0], 17)

    # # # 靠近碗
    # mrp2a.dual_straight_Path(0, -0.14, 0, 0, 0.14, 0)
    # # # 斜上
    # mrp2a.dual_straight_Path(0, 0.1, 0.2, 0, 0.1, 0.2)


    #
    # target_pose3.position.x = 0
    # target_pose3.position.y = 0.1
    # target_pose3.position.z = 0.2
    #
    # target_pose4.position.x = 0
    # target_pose4.position.y = 0.1
    # target_pose4.position.z = 0.2
    # mrp2a.dual_Plan_Cartesian_Path_Inter(target_pose3, target_pose4, 25)

    # target_pose1.header.frame_id = 'base_link'
    # target_pose1.pose.position.x = 0.85
    # target_pose1.pose.position.y = 0.22
    # target_pose1.pose.position.z = 1.43
    # quaternion = transformations.quaternion_from_euler(-2.713, 1.503, 2.244)
    # #quaternion = transformations.quaternion_from_euler(-1.761, 1.542, -3.087)
    # target_pose1.pose.orientation.x = quaternion[0]
    # target_pose1.pose.orientation.y = quaternion[1]
    # target_pose1.pose.orientation.z = quaternion[2]
    # target_pose1.pose.orientation.w = quaternion[3]
    #
    # target_pose2.header.frame_id = 'base_link'
    # target_pose2.pose.position.x = 0.85
    # target_pose2.pose.position.y = -0.02
    # target_pose2.pose.position.z = 1.44
    # quaternion = transformations.quaternion_from_euler(-0.837, -1.473, -1.011)
    # target_pose2.pose.orientation.x = quaternion[0]
    # target_pose2.pose.orientation.y = quaternion[1]
    # target_pose2.pose.orientation.z = quaternion[2]
    # target_pose2.pose.orientation.w = quaternion[3]
    # #mrp2a.arm_pose_control("left_arm", target_pose1)
    # #mrp2a.arm_pose_control("right_arm", target_pose2)
    # mrp2a.dual_arm_pose_control(target_pose1, target_pose2)


    # 运动学插件实验
    # pos_left = [0.85, 0.26, 1.23]
    # rot_left = [-2.713, 1.503, 2.244]
    # pos_right = [0.85, -0.26, 1.24]
    # rot_right = [-0.837, -1.473, -1.011]
    # mrp2a.dual_arm_pose_control(pos_left, rot_left, pos_right, rot_right)
    #
    # # target_pose = [0, -0.14, 0]
    # # mrp2a.Plan_Cartesian_Path("left_arm", target_pose)
    #
    # target_pose1 = [0, -0.14, 0]
    # target_pose2 = [0, 0.14, 0]
    # mrp2a.dual_Plan_Cartesian_Path_Inter(target_pose1, target_pose2, 17)

    # pos = [1.038, -0.184, 1.286]
    # rot = [1.059, -1.150, 2.100]
    # mrp2a.arm_pose_control("right_arm", pos, rot)

    #pos = [0.87, -0.28, 1.09]
    #rot = [-1.57, -1.57, -1.57]
    #mrp2a.arm_pose_control("right_arm", pos, rot)
    #
    # target_pose1 = [0.26, 0, 0]
    # mrp2a.Plan_Cartesian_Path("right_arm", target_pose1)

    # pos = [1.054, -0.350, 1.071]
    # rot = [-2.843, -1.565, -1.347]
    # mrp2a.arm_pose_control("right_arm", pos, rot)

    # pos_left = [0.85, 0.26, 1.23]
    # rot_left = [-2.713, 1.503, 2.244]
    # pos_right = [0.85, -0.26, 1.24]
    # rot_right = [-0.837, -1.473, -1.011]
    # mrp2a.dual_arm_pose_control(pos_left, rot_left, pos_right, rot_right)
    # rospy.sleep(0.5)
    #
    # mrp2a.dual_gripper_joint_control("open", "open", 0, 0)
    #
    # # 靠近碗
    # mrp2a.dual_Plan_Cartesian_Path_Inter([0, -0.14, 0], [0, 0.14, 0], 17)
    #
    # # 端碗
    # mrp2a.dual_Plan_Cartesian_Path_Inter([0, 0, 0.1], [0, 0, 0.1], 12)
    #
    # # 斜上
    # mrp2a.dual_Plan_Cartesian_Path_Inter([0, 0.1, 0.2], [0, 0.1, 0.2], 28)
    #
    # # 还原
    # mrp2a.dual_Plan_Cartesian_Path_Inter([0, -0.1, -0.2], [0, -0.1, -0.2], 28)
    #
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
    # mrp2a.dual_Plan_Cartesian_Path_Inter([0, 0, -0.1], [0, 0, -0.1], 22)
    #
    # # 离开碗
    # mrp2a.dual_Plan_Cartesian_Path_Inter([0, 0.14, 0], [0, -0.14, 0], 17)

    # pos = [1.128, 0.058, 1.508]
    # rot = [-1.57, -1.57, -1.57]
    # mrp2a.arm_pose_control("right_arm", pos, rot)
    #
    # pos2 = [1.128, 0.058, 1.458]
    # print mrp2a.sample_Plan_Cartesian_Path("right_arm", pos2)


    # n = 0
    # m = 0
    # i = 1
    # pos[0] = 1.13
    # while i <= 3:
    #     pos[0] = pos[0] - 0.01
    #     j = 1
    #     pos[1] = -0.511
    #     while j <= 3:
    #         pos[1] = pos[1] - 0.01
    #         k = 1
    #         pos[2] = 1.260
    #         while k <= 3:
    #             pos[2] = pos[2] - 0.01
    #             print pos
    #             #mrp2a.sample_arm_pose_control("right_arm", pos, rot)
    #             if mrp2a.sample_arm_pose_control("right_arm", pos, rot) == 0:
    #                 n = n + 1
    #             elif mrp2a.sample_arm_pose_control("right_arm", pos, rot) == 1:
    #                 m = m + 1
    #             k = k + 1
    #         j = j + 1
    #     i = i + 1
    # print "m = ", m
    # print "n = ", n


    # pos = [0.928, -0.576, 1.388]
    # rot = [-3.017, -0.729, -0.093]
    # mrp2a.arm_pose_control("right_arm", pos, rot)
    # mrp2a.Plan_Cartesian_Path_Inter("right_arm", [-0.1, 0, 0], 12)


    # pos = [1.13, -0.51, 1.26]
    # rot = [-1.57, -1.57, -1.57]
    # mrp2a.arm_pose_control("right_arm", pos, rot)
    #
    # i = 1
    # while i <= 3:
    #     mrp2a.Plan_Cartesian_Path("right_arm", [0, 0, 0.01])
    #     j = 1
    #     while j <= 3:
    #         mrp2a.Plan_Cartesian_Path("right_arm", [0, 0.01, 0])
    #         j = j + 1
    #     # j = 1
    #     # while j <= 3:
    #     #     k = 1
    #     #     while k <= 3:
    #
    #         #     k = k + 1
    #         # j = j + 1
    #     i = i + 1

    #mrp2a.arm_joint_control("left_arm", [1.2, 0, 0, 1, -1, 0, 0])

    # pos_left = [0.85, 0.26, 1.08]
    # rot_left = [-2.713, 1.503, 2.244]
    # pos_right = [0.85, -0.26, 1.09]
    # rot_right = [-0.837, -1.473, -1.011]
    # mrp2a.dual_arm_pose_control(pos_left, rot_left, pos_right, rot_right)
    # rospy.sleep(1)
    #
    # mrp2a.dual_Plan_Cartesian_Path_Inter([0, -0.14, 0], [0, 0.14, 0], 17)
    # rospy.sleep(10)
    #
    # pos_left = [0.85, 0.12, 1.28]
    # pos_right = [0.85, -0.12, 1.29]
    # mrp2a.dual_arm_pose_control(pos_left, rot_left, pos_right, rot_right)

    # 速度控制
    # pos = [0.85, 0.26, 1.08]
    # rot = [-2.713, 1.503, 2.244]
    #
    # group = mrp2a.group_left_arm
    # target_pose = geometry_msgs.msg.PoseStamped()
    # target_pose.header.frame_id = 'base_link'
    # target_pose.pose.position.x = pos[0]
    # target_pose.pose.position.y = pos[1]
    # target_pose.pose.position.z = pos[2]
    # quaternion = transformations.quaternion_from_euler(rot[0], rot[1], rot[2])
    # target_pose.pose.orientation.x = quaternion[0]
    # target_pose.pose.orientation.y = quaternion[1]
    # target_pose.pose.orientation.z = quaternion[2]
    # target_pose.pose.orientation.w = quaternion[3]
    # group.set_pose_target(target_pose)
    # traj = group.plan()
    #
    # #print "traj1 = ", traj.joint_trajectory.points[5].velocities
    #
    # mrp2a.set_trajectory_speed(traj, 0.25)
    # print "traj2 = ", traj.joint_trajectory.points[5].velocities
    #
    # group.execute(traj)
    #
    # #group.execute(traj2)

    # pos_left = [0.95, 0.15, 1.02]
    # rot_left = [1.57, 1.57, 1.57]
    # pos_right = [0.95, -0.15, 1.02]
    # rot_right = [1.57, 0, 1.57]
    # mrp2a.dual_arm_pose_control(pos_left, rot_left, pos_right, rot_right)
    # rospy.sleep(0.5)
    #
    # # pos_left = [0.95, 0.15, 1.17]
    # # rot_left = [1.57, -0.5, 1.57]
    # # pos_right = [0.95, -0.15, 1.17]
    # # rot_right = [1.57, -0.5, 1.57]
    #
    # pos_left = [0.95, 0.15, 1.27]
    # rot_left = [1.57, 1.57, 1.57]
    # pos_right = [0.95, -0.15, 1.27]
    # rot_right = [1.57, 0, 1.57]
    #
    # pos_left = [0.95, 0.131, 1.206]
    # rot_left = [1.563, -0.5, 1.520]
    # pos_right = [0.95, -0.138, 1.095]
    # rot_right = [1.595, -0.460, 1.520]
    # mrp2a.dual_arm_pose_control(pos_left, rot_left, pos_right, rot_right)
    # mrp2a.dual_arm_pose_control_constraints(pos_left, rot_left, pos_right, rot_right)

    # pub = rospy.Publisher('traj_point', String, queue_size=10)
    # rospy.init_node('talker', anonymous=True)
    # i = 0
    # while (i<5):
    #     traj = "effector_link tf"
    #     rospy.loginfo(traj)
    #     pub.publish(traj)
    #     i = i+1

    #mrp2a.dual_arm_joint_control([0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 1.0])

    # pos_left = [0.85, 0.26, 1.08]
    # rot_left = [-2.713, 1.503, 2.244]
    # pos_right = [0.85, -0.26, 1.09]
    # rot_right = [-0.837, -1.473, -1.011]
    # mrp2a.dual_arm_pose_control(pos_left, rot_left, pos_right, rot_right)
    #
    # mrp2a.dual_arm_pose_control(pos_left, rot_left, pos_right, rot_right)
    #
    # mrp2a.dual_Plan_Cartesian_Path_Inter([0, -0.14, 0], [0, 0.14, 0], 17)
    # mrp2a.dual_Plan_Cartesian_Path_Inter([0, 0.1, 0.15], [0, 0.1, 0.15], 20)

    # pos_left = [0.850, 0.120, 1.080]
    # rot_left = [-2.713, 1.503, 2.244]
    # # mrp2a.arm_pose_control("left_arm", pos_left, rot_left)
    # mrp2a.dual_arm_coordinate(pos_left, rot_left)

    # pos_left = [0.85, 0.26, 1.08]
    # rot_left = [-2.713, 1.503, 2.244]
    # pos_right = [0.85, -0.26, 1.09]
    # rot_right = [-0.837, -1.473, -1.011]
    # mrp2a.dual_arm_pose_control(pos_left, rot_left, pos_right, rot_right)
    #
    # mrp2a.dual_Plan_Cartesian_Path_Inter([0, -0.14, 0], [0, 0.14, 0], 17)
    #
    # # pos_left = [0.85, 0.12, 1.25]
    # # rot_left = [-2.713, 1.503, 2.244]
    # pos_left = [0.85, 0.12, 1.15]
    # rot_left = [-2.950, 1.314, 2.032]
    # pos = [-0.023, -0.059, 0.232]
    # rot = [2.620, -0.032, 3.140]
    # mrp2a.dual_arm_coordinate(pos_left, rot_left, pos, rot)

    # pos_left = [0.85, 0.12, 1.25]
    # rot_left = [-2.713, 1.503, 2.244]
    # pos_right = [0.85, -0.12, 1.25]
    # rot_right = [-0.837, -1.473, -1.011]
    # mrp2a.dual_arm_pose_control(pos_left, rot_left, pos_right, rot_right)

    #mrp2a.dual_Plan_Cartesian_Path_Inter([0, 0.01, 0], [0, -0.01, 0], 2)

    #mrp2a.gripper_joint_control("right_gripper", "close", 0.35)

    # target_pose = geometry_msgs.msg.PoseStamped()
    # target_pose.header.frame_id = 'base_link'
    # target_pose.pose.position.x = 1
    # target_pose.pose.position.y = 1
    # target_pose.pose.position.z = 1
    # quaternion = transformations.quaternion_from_euler(rot[0], rot[1], rot[2])
    # target_pose.pose.orientation.x = quaternion[0]
    # target_pose.pose.orientation.y = quaternion[1]
    # target_pose.pose.orientation.z = quaternion[2]
    # target_pose.pose.orientation.w = quaternion[3]

    # print mrp2a.forward_kinematics([-0.05041055236691072, 1.0054135903828065, 1.319918501281517, -1.0600569052296844,
    # 0.9892630717069189, -1.3189748813750768, -1.32551432364785,])

    # pos_left = [0.85, 0.26, 1.08]
    # rot_left = [-2.713, 1.503, 2.244]
    # mrp2a.test("left_arm", pos_left, rot_left)

    # theta = [0.3537583652875469, 0.37652070777905156, 1.2290339385008124,
    #          -1.8111597325638433, 0.5092082403665401, -0.6737646309739027,
    #          -1.7933723110496507]
    # mrp2a.MoveJ("left_arm", theta)

    #mrp2a.dual_gripper_joint_control("close", "close", 0.15, 0.15)

    # pos_left = [0.95, 0.15, 1.02]
    # rot_left = [1.57, 0, 1.57]
    # pos_right = [0.95, -0.15, 1.02]
    # rot_right = [1.57, 0, 1.57]
    # # mrp2a.dual_arm_pose_control(pos_left, rot_left, pos_right, rot_right)
    # #
    # # pos = [0.1, 0, 0]
    # # #mrp2a.test(pos, pos)
    # # mrp2a.test(pos, pos, 17)
    #
    # joint_left = [-0.08650297458250034, 1.2727781639375984, -1.3009260606550033, 1.1520718711778377,
    #               -1.4356824556127217, 0.7415745731480765, -0.6039796775483115]
    # joint_right = [0.10429774001847303, -1.2805593428650264, 1.3217387173606268, -1.1297394680589363,
    #                1.4154110194771112, -0.7207960688389061, 0.6127798473117831]
    #
    # mrp2a.dual_arm_joint_control(joint_left, joint_right)
    #mrp2a.dual_arm_pose_control(pos_left, rot_left, pos_right, rot_right)
    #rospy.sleep(10)
    #mrp2a.dual_Plan_Cartesian_Path([0, 0, 0.25], [0, 0, 0.25])
    #mrp2a.Plan_Cartesian_Path("left_arm", [0, 0, 25])
    #mrp2a.Plan_Cartesian_Path("right_arm", [0, 0, 25])
    #mrp2a.test1()

    # Pose = geometry_msgs.msg.Pose
    # target_pose = []
    # pos_left = [0.85, 0.26, 1.08]
    # #rot_left = [-2.713, 1.503, 2.244]
    # rot_left = transformations.quaternion_from_euler(-2.713, 1.503, 2.244)
    # #rot_left = [0.441, 0.580, -0.429, 0.534]
    #
    # target_pose1 = geometry_msgs.msg.PoseStamped()
    # target_pose1.header.frame_id = 'base_link'
    # target_pose1.pose.position.x = pos_left[0]
    # target_pose1.pose.position.y = pos_left[1]
    # target_pose1.pose.position.z = pos_left[2]
    # #quaternion = transformations.quaternion_from_euler(rot_left[0], rot_left[1], rot_left[2])
    # target_pose1.pose.orientation.x = rot_left[0]
    # target_pose1.pose.orientation.y = rot_left[1]
    # target_pose1.pose.orientation.z = rot_left[2]
    # target_pose1.pose.orientation.w = rot_left[3]

    #
    # target_pose2 = geometry_msgs.msg.PoseStamped()
    # target_pose2.header.frame_id = 'base_link'
    # target_pose2.pose.position.x = pos_left2[0]
    # target_pose2.pose.position.y = pos_left2[1]
    # target_pose2.pose.position.z = pos_left2[2]
    # quaternion = transformations.quaternion_from_euler(rot_left2[0], rot_left2[1], rot_left2[2])
    # target_pose2.pose.orientation.x = quaternion[0]
    # target_pose2.pose.orientation.y = quaternion[1]
    # target_pose2.pose.orientation.z = quaternion[2]
    # target_pose2.pose.orientation.w = quaternion[3]
    #
    #
    # #target_pose.append(target_pose1.pose)
    #
    # # # target_pose.append(target_pose1)
    # # # target_pose.append(target_pose2)
    # # target_pose.append(Pose(pos_left, rot_left))
    # # target_pose.append(Pose(pos_left2, rot_left2))
    #
    # goal_point = geometry_msgs.msg.Point(0.85, 0.12, 1.08)
    # goal_ori = geometry_msgs.msg.Quaternion(rot_left[0], rot_left[1], rot_left[2], rot_left[3])
    # target_pose.append(Pose(goal_point, goal_ori))
    #
    # goal_point2 = geometry_msgs.msg.Point(0.863, 0.879, 1.390)
    # goal_ori2 = geometry_msgs.msg.Quaternion(rot_left2[0], rot_left2[1], rot_left2[2], rot_left2[3])
    # target_pose.append(Pose(goal_point2, goal_ori2))
    # print target_pose
    # mrp2a.arm_poses_control("left_arm", target_pose)

    #pos_left = [0.85, 0.26, 1.08]
    #rot_left = [-2.713, 1.503, 2.244]
    # print mrp2a.inverse_kinematics("left_arm", pos_left, rot_left)

    # joint1 = [0.0489146,0.702708,1.42906,-0.718713,0.617084,-1.56289,-1.13594]
    #
    # mrp2a.forward_kinematics(joint1)

    # pos_left = [0.85, 0.26, 1.08]
    # rot_left = [-2.713, 1.503, 2.244]

    # pos = [0.863, 0.879, 1.388]
    # rot = [1.78259, 1.57545, 0.735592]
    #
    # mrp2a.arm_pose_control("left_arm", pos, rot)

    # target_pose = geometry_msgs.msg.PoseStamped()
    # target_pose.header.frame_id = "base_link"
    # a = transformations.euler_matrix(-2.406, 1.566, -1.359, 'sxyz')
    # print a
    # b = transformations.euler_from_matrix(a, 'sxxy')

    # joint_left = [-0.8938021606637827, 0.6477844766050458, 2.733789242970616, -1.7156256211236638,
    # -2.7000346418110306, -1.2555944537843917, 3.049923976357862,]
    # joint_right = [-0.4853739151199443, -1.0242822820735453, 1.9481379075334981, 1.6877580834950354,
    #                0.6244975538631738, -0.9781498956736501, 0.43103792813840247]
    # mrp2a.dual_arm_joint_control(joint_left, joint_right)

    # pos_left = [0.30, 0.26, 1.12]
    # rot_left = [-2.713, 1.503, 2.244]
    # pos_right = [0.30, -0.26, 1.12]
    # rot_right = [-0.837, -1.473, -1.011]
    # mrp2a.dual_arm_pose_control(pos_left, rot_left, pos_right, rot_right)
    # rospy.sleep(0.5)

    # joint_left = [2.1688119870559923, -0.38854871092445187, -1.3825736780268603, -0.5555959631029586,
    # 0.8754500787822543, -1.6814338880570388, -1.7892983178713466]
    # joint_right = [1.9799830774432365, 0.22708586625711025, -0.3234329310594193, 0.6671038642725939,
    # 3.049044000356595, -1.4705617306816627, 1.6303825787598125]
    # mrp2a.dual_arm_joint_control(joint_left, joint_right)

    joint = [1.1, -0.9858628222597666, -1.9191075920735463, 2.004363402938921,
             -0.12494848443797386, -1.6251872184778957, 0.5778878846491576]
    mrp2a.arm_joint_control("left_arm", joint)
    # print transformations.quaternion_from_euler(-1.571, 1.571, -1.571)
    # pos_left = [0.293, 0.121, 1.057]
    # rot_left = [-1.571, 1.571, -1.571]
    # mrp2a.arm_pose_control("left_arm", pos_left, rot_left)

    # joints = [0.6974500167427566, 0.8049575780936343, -2.802697372213226, 2.07438019590088,
    #           0.28592038419928395, -0.40307762662023167, 1.8873323782453595]
    # #joints = [0, 0, 0, 0, 0, 0, 0]
    # mrp2a.arm_angle(joints)