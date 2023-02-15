#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import numpy
import copy
import rospy
import actionlib

from tf import *
from geometry_msgs.msg import *
from moveit_msgs.msg import *
from moveit_msgs.srv import *
from actionlib_msgs.msg import *
from tf.listener import *

from ar_track_alvar_msgs.msg import AlvarMarkers

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from moveit_commander import PlanningSceneInterface, MoveGroupCommander
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from wust_robot_moveit_control_full import *
from move import MOVE



wust = Wust_Robot_Moveit_Control()

# box1 = Object_info()
# box2 = Object_info()
# box3 = Object_info()
# box4 = Object_info()
# box5 = Object_info()
# box6 = Object_info()
# bottle = Object_info()
# cup = Object_info()
# plate = Object_info()


# box1.name = "box1"
# box1.size = (0.05, 0.05, 0.2)
# box1.offset = [0.0, 0.0, 0]  # 二维码和物体质心坐标之间的补偿
# box1.id = 2
#
# box2.name = "box2"
# box2.size = (0.05, 0.05, 0.2)
# box2.offset = [0.0, 0.0, 0]
# box2.id = 3
#
# box3.name = "box3"
# box3.size = (0.05, 0.05, 0.2)
# box3.offset = [0.0, 0.0, 0]
# box3.id = 4
#
# box4.name = "box4"
# box4.size = (0.05, 0.05, 0.2)
# box4.offset = [0.0, 0.0, 0]
# box4.id = 5
#
# box5.name = "box5"
# box5.size = (0.05, 0.05, 0.2)
# box5.offset = [0.0, 0.0, 0]
# box5.id = 6
#
# box6.name = "box6"
# box6.size = (0.05, 0.05, 0.2)
# box6.offset = [0.0, 0.0, 0]
# box6.id = 7
#
# bottle.name = "bottle"
# bottle.size = (0.05, 0.05, 0.2)
# bottle.offset = [0.0, 0.0, 0]
# bottle.id = 8
#
# cup.name = "cup"
# cup.size = (0.05, 0.05, 0.2)
# cup.offset = [0.0, 0.0, 0]
# cup.id = 9
#
# plate.name = "plate"
# plate.size = (0.25, 0.32, 0.05)
# plate.offset = [0.0, 0.0, 0.0]
# plate.id = 11


table_name = "table"
table_pose = (0.55, 0, 0.45, 0, 0, 0)
table_size = (1.0, 1.0, 0.9)
table_pose_stamped = wust.Pose_Transform(table_pose)

box_offset = [0, 0, -0.025]
plate_offset = [0, 0, -0.015]
cup_offset = [0.025, 0, 0]

class Object_info():
    def __init__(self):
        self.name = ''
        self.size = [0, 0, 0]
        self.pose = [0, 0, 0, 0, 0, 0]
        self.id = None

class Wust_Robot_Grasp_Vision():
    def __init__(self):
        self.box_ids = []        #用于临时存放二维码id列表,每次用完清空
        self.target_poses = []      #用于临时存放二维码坐标列表,每次用完清空

        self.target_pose = None  # 单个二维码坐标
        self.Alvar_Markers = []   #二维码坐标列表
        self.Alvar_Markers_id = []  #二维码id列表
        rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.ar_pose_cb)

        ps_left = PoseStamped()
        ps_left.header.frame_id = "/base_link"
        ps_right = PoseStamped()
        ps_right.header.frame_id = "/base_link"
        self.ps_left = ps_left
        self.ps_right = ps_right

    def ar_pose_cb(self, msg):
        ar_poses = []
        if len(msg.markers)>0:
            for i in range(len(msg.markers)):            # 循环输出每一个识别到的二维码
                self.box_ids.append(msg.markers[i].id)   # 将识别到的二维码id添加到列表中
                pose_info = msg.markers[i].pose.pose     # 二维码的位姿
                ar_pose = [pose_info.position.x, pose_info.position.y, pose_info.position.z]   # 获取二维码坐标
                # 获取10次坐标求平均值
                for j in range(10):
                    #print ar_pose
                    ar_poses.append(ar_pose)
                px = []
                py = []
                pz = []
                for pose_list in ar_poses:
                    px.append(pose_list[0])
                    py.append(pose_list[1])
                    pz.append(pose_list[2])
                self.target_pose = [sum(px)/10, sum(py)/10, sum(pz)/10]
                ar_poses = []  # 求完平均值后清空列表
                self.target_poses.append(self.target_pose)    #将求完平均值的坐标添加到二维码坐标列表里
        if len(self.target_poses) == len(msg.markers):
            self.Alvar_Markers = self.target_poses
            self.Alvar_Markers_id = self.box_ids
            self.target_poses = []
            self.box_ids = []
        # if len(ar_poses) == 10:
        #     px = []
        #     py = []
        #     pz = []
        #     for pose_list in ar_poses:
        #         px.append(pose_list[0])
        #         py.append(pose_list[1])
        #         pz.append(pose_list[2])
        #     self.target_pose = [sum(px)/10, sum(py)/10, sum(pz)/10]


    def wust_robot_grasp_left_arm(self, box_id):
        ps = self.ps

        pos = []
        pos.append(ps.pose.position.x - 0.1)
        pos.append(ps.pose.position.y)
        pos.append(ps.pose.position.z)

        rot = [1.57, 1.57, 1.57]
        wust.arm_pose_control("left_arm", pos, rot)
        # wust.Plan_Cartesian_Path("left_arm", pos)
        # while True:
        #     result =
        #     if result == MoveItErrorCodes.SUCCESS:
        #         rospy.loginfo('Pick succeeded')
        #         break
        #     elif result == MoveItErrorCodes.PLANNING_FAILED:
        #         rospy.logerr('Pick failed in the planning stage, try again...')
        #         rospy.sleep(0.5)
        #         continue
        #     elif result == MoveItErrorCodes.CONTROL_FAILED:
        #         rospy.logerr('Pick failed CONTROL_FAILED')
        #         break
        #     elif result == MoveItErrorCodes.MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE:
        #         rospy.logerr('Pick failed MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE')
        #         break
        #     else:
        #         rospy.logerr('Pick failed with error code: %d PREEMPTED.' % result)
        #         continue
    # def wust_grasp_right_arm(self):

    # 双臂视觉分拣积木
    def wust_robot_dual_arm_grasp_block(self, box_id1, box_id2):
        ps_left = self.ps_left
        ps_right = self.ps_right

        while self.target_pose is None:
            rospy.sleep(0.02)

        wust.Add_Box(table_name, table_pose_stamped, table_size)
        print " ===== ", len(self.Alvar_Markers_id)
        print self.Alvar_Markers
        for i in range(len(self.Alvar_Markers_id)):
            if box_id1 == self.Alvar_Markers_id[i]:    #需要抓取的物品对应的二维码id
                target_pose = self.Alvar_Markers[i]
                target_x = target_pose[0] + box_offset[0]
                target_y = target_pose[1] + box_offset[1]
                target_z = target_pose[2] + box_offset[2]

                ps_left.pose = Pose(Point(target_x, target_y, target_z), Quaternion(0,0,0,1))
                box_left = Object_info()
                box_left.name = 'box_left'
                box_left.size = (0.05, 0.05, 0.05)
                box_left.id = box_id1
                wust.Add_Box(box_left.name, ps_left, box_left.size)

            if box_id2 == self.Alvar_Markers_id[i]:    #需要抓取的物品对应的二维码id
                target_pose = self.Alvar_Markers[i]
                target_x = target_pose[0] + box_offset[0]
                target_y = target_pose[1] + box_offset[1]
                target_z = target_pose[2] + box_offset[2]

                ps_right.pose = Pose(Point(target_x, target_y, target_z), Quaternion(0,0,0,1))
                box_right = Object_info()
                box_right.name = 'box_right'
                box_right.size = (0.05, 0.05, 0.05)
                box_right.id = box_id2
                wust.Add_Box(box_right.name, ps_right, box_right.size)

        pos_left = []
        pos_right = []
        pos_left.append(ps_left.pose.position.x)
        pos_left.append(ps_left.pose.position.y)
        pos_left.append(ps_left.pose.position.z + 0.1)

        pos_right.append(ps_right.pose.position.x)
        pos_right.append(ps_right.pose.position.y)
        pos_right.append(ps_right.pose.position.z + 0.1)

        rot_left = [-3.14, 0.79, -3.14]
        rot_right = [3.14, -0.96, 0]
        # rot_left = [1.57, 0, 0]
        # rot_right = [-1.57, 0, 0]
        wust.dual_arm_pose_control(pos_left, rot_left, pos_right, rot_right)
        wust.dual_gripper_joint_control("open", "open", 0, 0)
        wust.dual_Plan_Cartesian_Path_Inter([0, 0, -0.1], [0, 0, -0.1], 12)
        wust.dual_gripper_joint_control("close", "close", 0.15, 0.15)
        wust.dual_Plan_Cartesian_Path_Inter([0, 0, 0.12], [0, 0, 0.12], 14)
        wust.Plan_Cartesian_Path_Rot("left_arm", [0.35, 0.3, 1.08], [-3.14, 0.79, -3.14], 0)
        wust.Plan_Cartesian_Path_Rot("right_arm", [0.35, -0.3, 1.08], [3.14, -0.96, 0], 0)
        wust.dual_gripper_joint_control("open", "open", 0, 0)
        wust.dual_arm_joint_control([0,0,0,0,0,0,0],[0,0,0,0,0,0,0])
        # wust.Remove_Box(plate_name)
        #

        # wust.dual_Plan_Cartesian_Path_Inter([0, -0.07, 0], [0.0, 0.07, 0], 8)
        # wust.dual_gripper_joint_control("close", "close", 0.17, 0.17)
        # wust.dual_Plan_Cartesian_Path_Inter([0, 0, 0.1], [0.0, 0, 0.1], 12)

    def wust_robot_dual_arm_coordinate(self, plate_id):
        print "端盆"
        ps_left = self.ps_left
        ps_right = self.ps_right

        while self.target_pose is None:
            rospy.sleep(0.02)

        wust.Add_Box(table_name, table_pose_stamped, table_size)
        print " ===== ", len(self.Alvar_Markers_id)
        print self.Alvar_Markers
        for i in range(len(self.Alvar_Markers_id)):
            if plate_id == self.Alvar_Markers_id[i]:  # 需要抓取的物品对应的二维码id
                target_pose = self.Alvar_Markers[i]
                target_x = target_pose[0] + plate_offset[0]
                target_y = target_pose[1] + plate_offset[1]
                target_z = target_pose[2] + plate_offset[2]

                ps_left.pose = Pose(Point(target_x, target_y, target_z), Quaternion(0, 0, 0, 1))
                ps_right.pose = Pose(Point(target_x, target_y, target_z), Quaternion(0, 0, 0, 1))
                plate = Object_info()
                plate.name = 'plate'
                plate.size = (0.24, 0.36, 0.03)
                plate.id = plate_id
                wust.Add_Box(plate.name, ps_left, plate.size)

        pos_left = []
        pos_right = []
        pos_left.append(ps_left.pose.position.x)
        pos_left.append(ps_left.pose.position.y + 0.22)
        pos_left.append(ps_left.pose.position.z)

        pos_right.append(ps_right.pose.position.x)
        pos_right.append(ps_right.pose.position.y - 0.22)
        pos_right.append(ps_right.pose.position.z)

        rot_left = [1.57, 0, 0]
        rot_right = [-1.57, 0, 0]
        # rot_left = [1.57, 0, 0]
        # rot_right = [-1.57, 0, 0]
        wust.dual_arm_pose_control(pos_left, rot_left, pos_right, rot_right)
        wust.Add_Box(plate.name, ps_left, plate.size)
        wust.dual_gripper_joint_control("open", "open", 0, 0)
        wust.dual_Plan_Cartesian_Path_Inter([0, -0.07, 0], [0, 0.07, 0], 10)
        wust.dual_gripper_joint_control("close", "close", 0.15, 0.15)
        wust.dual_Plan_Cartesian_Path_Inter([0, 0, 0.1], [0, 0, 0.1], 12)
        #wust.dual_Plan_Cartesian_Path_Inter([0, 0, -0.1], [0, 0, -0.1], 12)

        # 向左
        wust.dual_Plan_Cartesian_Path_Inter([0, 0.1, 0], [0, 0.1, 0], 12)

        # 还原
        wust.dual_Plan_Cartesian_Path_Inter([0, -0.1, 0], [0, -0.1, 0], 12)

        # 向右
        wust.dual_Plan_Cartesian_Path_Inter([0, -0.1, 0], [0, -0.1, 0], 12)

        # 还原
        wust.dual_Plan_Cartesian_Path_Inter([0, 0.1, 0], [0, 0.1, 0], 12)

        # 放碗
        wust.dual_Plan_Cartesian_Path_Inter([0, 0, -0.1], [0, 0, -0.1], 12)
        #
        # 离开碗
        wust.dual_Plan_Cartesian_Path_Inter([0, 0.08, 0], [0, -0.08, 0], 10)


    def wust_robot_dual_arm_pour_water(self, cup_id, bottle_id):
        ps_left = self.ps_left
        ps_right = self.ps_right

        while self.target_pose is None:
            rospy.sleep(0.02)

        wust.Add_Box(table_name, table_pose_stamped, table_size)
        print " ===== ", len(self.Alvar_Markers_id)
        print self.Alvar_Markers
        for i in range(len(self.Alvar_Markers_id)):
            if cup_id == self.Alvar_Markers_id[i]:  # 需要抓取的物品对应的二维码id
                target_pose = self.Alvar_Markers[i]
                target_x = target_pose[0] + box_offset[0]
                target_y = target_pose[1] + box_offset[1]
                target_z = target_pose[2] + box_offset[2]

                ps_left.pose = Pose(Point(target_x, target_y, target_z), Quaternion(0, 0, 0, 1))
                cup = Object_info()
                cup.name = 'cup'
                cup.size = (0.05, 0.05, 0.12)
                cup.id = cup_id
                wust.Add_Box(cup.name, ps_left, cup.size)

            if bottle_id == self.Alvar_Markers_id[i]:  # 需要抓取的物品对应的二维码id
                target_pose = self.Alvar_Markers[i]
                target_x = target_pose[0] + box_offset[0]
                target_y = target_pose[1] + box_offset[1]
                target_z = target_pose[2] + box_offset[2]

                ps_right.pose = Pose(Point(target_x, target_y, target_z), Quaternion(0, 0, 0, 1))
                bottle = Object_info()
                bottle.name = 'bottle'
                bottle.size = (0.05, 0.05, 0.18)
                bottle.id = bottle_id
                wust.Add_Box(bottle.name, ps_right, bottle.size)

        pos_left = []
        pos_right = []
        pos_left.append(ps_left.pose.position.x - 0.1)
        pos_left.append(ps_left.pose.position.y)
        pos_left.append(ps_left.pose.position.z)

        pos_right.append(ps_right.pose.position.x - 0.1)
        pos_right.append(ps_right.pose.position.y)
        pos_right.append(ps_right.pose.position.z)
        wust.dual_arm_pose_control(pos_left, rot_left, pos_right, rot_right)

        wust.dual_gripper_joint_control("open", "open", 0, 0)
        wust.Remove_Box(bottle.name)
        wust.Remove_Box(cup.name)
        wust.dual_Plan_Cartesian_Path_Inter([0.1, 0, 0], [0.1, 0, 0], 12)
        # wust.Attach_Box("right_arm", bottle.name, 4)
        wust.dual_gripper_joint_control("close", "close", 0.15, 0.15)
        wust.Plan_Cartesian_Path_Inter("left_arm", [0, 0, 0.10], 12)
        wust.Plan_Cartesian_Path_Inter("right_arm", [0, 0, 0.18], 20)
        # wust.Plan_Cartesian_Path_Inter("right_arm", [0, 0.22, 0], 25)
        wust.dual_Plan_Cartesian_Path_Inter([0, -ps_left.pose.position.y + 0.06, 0],
                                            [0, -ps_right.pose.position.y - 0.06, 0], 30)
        rospy.sleep(1)
        wust.arm_joint_change("R_joint7", 1.5)
        rospy.sleep(5)

if __name__ == '__main__':
    robot = Wust_Robot_Grasp_Vision()

    # demo
    #robot.wust_robot_grasp_left_arm(box3.id)
    #robot.wust_robot_grasp_right_arm(box5.id)

    # 视觉分拣积木
    # def callback(data):
    #     rospy.loginfo(data.data)
    #    if data.data =="帮我端果盆":
    robot.wust_robot_dual_arm_coordinate(11)
    #     if data.data == "请把积木放到篮子里":
    #         robot.wust_robot_dual_arm__grasp_block(11)
    #
    # def listener():
    #
    #     rospy.Subscriber("/recognizer/output", String, callback)
    #
    #     # spin() simply keeps python from exiting until this node is stopped
    #     rospy.spin()
    #
    # listener()