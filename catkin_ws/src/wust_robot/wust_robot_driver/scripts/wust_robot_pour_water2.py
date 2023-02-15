#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
import geometry_msgs.msg
import moveit_msgs.msg
import tf
from tf import transformations
from wust_robot_moveit_control_full import Wust_Robot_Moveit_Control, Object_parameters
import numpy as np
import math
import matplotlib as mpl
import matplotlib.pyplot as plt
from math import pi
from std_msgs.msg import String

# 移动抓取

if __name__ == '__main__':
    wust = Wust_Robot_Moveit_Control()
    # pos_left = [0.22, 0.25, 0.95]
    # rot_left = [1.57, 0, 0]
    # pos_right = [0.22, -0.25, 0.95]
    # rot_right = [-1.57, 0, 0]
    # wust.dual_arm_pose_control(pos_left, rot_left, pos_right, rot_right)

    # 桌子信息


    # table = Object_parameters()
    # table.name = "table"
    # table.size = (1, 2, 0.9)
    # table.pose = (0.8, 0, 0.45, 0, 0, 0)
    #
    # table_pose_stamped = wust.Pose_Transform(table.pose)
    #
    # wust.Add_Box(table.name, table_pose_stamped, table.size)

    rospy.Subscriber("/wust_robot/camera/image_raw", 1)

    obj_camera_frame.setZ(-obj_y)
    obj_camera_frame.setY(-obj_x)
    obj_camera_frame.setX(0.45)
