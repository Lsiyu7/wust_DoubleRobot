#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import geometry_msgs.msg
from tf import transformations
from mrp2a_moveit_control_full import Mrp2a_Moveit_Control, Object_parameters

# 双臂同时运动倒水动作
if __name__ == '__main__':
    mrp2a = Mrp2a_Moveit_Control()

    # 桌子信息
    table_name = "table"
    table_size = (1, 0.8, 0.94)
    table_pose = (1.508299, 0, 0.47, 0, 0, 0)

    # 物体1信息
    wood_box1 = Object_parameters()
    wood_box1.name = "box1"
    wood_box1.size = (0.05, 0.05, 0.3)
    wood_box1.pose = (1.12, 0.28, 1.095, 0, 0, 0)
    wood_box1.place_pose = (1.12401, 0.28, 1.105, 0, 0, 0)
    wood_box1.pick_rpy_left = [[-1.57], [1.57], [-1.57]]
    wood_box1.approach_min = 0.15
    wood_box1.approach_desire = 0.2
    wood_box1.approach_axis = [0, 0, 1]
    wood_box1.retreat_min = 0.15
    wood_box1.retreat_desire = 0.2
    wood_box1.retreat_axis = [-1.5, 0, 0]
    wood_box1.compensate_left = [-0.012, 0, 0]

    # 物体2信息
    wood_box2 = Object_parameters()
    wood_box2.name = "box2"
    wood_box2.size = (0.05, 0.05, 0.3)
    wood_box2.pose = (1.12, -0.28, 1.095, 0, 0, 0)
    wood_box2.place_pose = (1.12, -0.28, 1.095, 0, 0, 0)
    wood_box2.pick_rpy_right = [[-1.57], [-1.57], [-1.57]]
    wood_box2.approach_min = 0.15
    wood_box2.approach_desire = 0.2
    wood_box2.approach_axis = [0, 0, 1]
    wood_box2.retreat_min = 0.15
    wood_box2.retreat_desire = 0.2
    wood_box2.retreat_axis = [0.5, 0, 0]
    wood_box2.compensate_right = [-0.015, 0, 0]

    table_pose_stamped = mrp2a.Pose_Transform(table_pose)

    box1_pose_stamped = mrp2a.Pose_Transform(wood_box1.pose)
    box1_place_pose_stamped = mrp2a.Pose_Transform(wood_box1.place_pose)

    box2_pose_stamped = mrp2a.Pose_Transform(wood_box2.pose)
    box2_place_pose_stamped = mrp2a.Pose_Transform(wood_box2.place_pose)

    mrp2a.Add_Box(table_name, table_pose_stamped, table_size)
    mrp2a.Add_Box(wood_box1.name, box1_pose_stamped, wood_box1.size)
    mrp2a.Add_Box(wood_box2.name, box2_pose_stamped, wood_box2.size)
    rospy.sleep(0.5)

    target_pose1 = geometry_msgs.msg.PoseStamped()
    target_pose2 = geometry_msgs.msg.PoseStamped()
    target_pose3 = geometry_msgs.msg.Pose()
    target_pose4 = geometry_msgs.msg.Pose()

    target_pose1.header.frame_id = 'base_link'
    target_pose1.pose.position.x = 0.87
    target_pose1.pose.position.y = 0.28
    target_pose1.pose.position.z = 1.09
    quaternion = transformations.quaternion_from_euler(-1.57, 1.57, -1.57)
    target_pose1.pose.orientation.x = quaternion[0]
    target_pose1.pose.orientation.y = quaternion[1]
    target_pose1.pose.orientation.z = quaternion[2]
    target_pose1.pose.orientation.w = quaternion[3]

    target_pose2.header.frame_id = 'base_link'
    target_pose2.pose.position.x = 0.87
    target_pose2.pose.position.y = -0.28
    target_pose2.pose.position.z = 1.09
    quaternion = transformations.quaternion_from_euler(-1.57, -1.57, -1.57)
    target_pose2.pose.orientation.x = quaternion[0]
    target_pose2.pose.orientation.y = quaternion[1]
    target_pose2.pose.orientation.z = quaternion[2]
    target_pose2.pose.orientation.w = quaternion[3]
    mrp2a.dual_arm_pose_control(target_pose1, target_pose2)
    rospy.sleep(0.5)

    mrp2a.dual_gripper_joint_control("open", "open", 0, 0)
    rospy.sleep(0.5)

    target_pose3.position.x = 0.24
    target_pose3.position.y = 0.0
    target_pose3.position.z = 0.0
    target_pose4.position.x = 0.24
    target_pose4.position.y = 0.0
    target_pose4.position.z = 0.0
    mrp2a.dual_Plan_Cartesian_Path(target_pose3, target_pose4)
    rospy.sleep(0.5)

    mrp2a.Attach_Box("left_arm", wood_box1.name, 4)
    mrp2a.Attach_Box("right_arm", wood_box2.name, 4)
    mrp2a.dual_gripper_joint_control("close", "close", 0.14, 0.14)
    rospy.sleep(0.5)

    target_pose3.position.x = 0.0
    target_pose3.position.y = 0.0
    target_pose3.position.z = 0.4
    target_pose4.position.x = 0.0
    target_pose4.position.y = 0.0
    target_pose4.position.z = 0.2
    mrp2a.dual_Plan_Cartesian_Path(target_pose3, target_pose4)
    rospy.sleep(1)

    target_pose3.position.x = 0.0
    target_pose3.position.y = -0.2
    target_pose3.position.z = 0.0
    target_pose4.position.x = 0.0
    target_pose4.position.y = 0.2
    target_pose4.position.z = 0.0
    mrp2a.dual_Plan_Cartesian_Path(target_pose3, target_pose4)
    rospy.sleep(1)

    mrp2a.arm_joint_control("left_arm", 0.8)
    rospy.sleep(4)
    mrp2a.arm_joint_control("left_arm", -0.8)
    rospy.sleep(0.5)

    target_pose3.position.x = 0.0
    target_pose3.position.y = 0.2
    target_pose3.position.z = 0.0
    target_pose4.position.x = 0.0
    target_pose4.position.y = -0.2
    target_pose4.position.z = 0.0
    mrp2a.dual_Plan_Cartesian_Path(target_pose3, target_pose4)
    rospy.sleep(1)

    target_pose3.position.x = 0.0
    target_pose3.position.y = 0.0
    target_pose3.position.z = -0.4
    target_pose4.position.x = 0.0
    target_pose4.position.y = 0.0
    target_pose4.position.z = -0.2
    mrp2a.dual_Plan_Cartesian_Path(target_pose3, target_pose4)
    rospy.sleep(1)

    mrp2a.dual_gripper_joint_control("open", "open", 0, 0)
    mrp2a.Detach_Box("left_arm", wood_box1.name, 4)
    mrp2a.Detach_Box("right_arm", wood_box2.name, 4)
    rospy.sleep(0.5)

    target_pose3.position.x = -0.2
    target_pose3.position.y = 0.0
    target_pose3.position.z = 0.0
    target_pose4.position.x = -0.2
    target_pose4.position.y = 0.0
    target_pose4.position.z = 0.0
    mrp2a.dual_Plan_Cartesian_Path(target_pose3, target_pose4)

