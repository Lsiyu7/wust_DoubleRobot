#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import numpy
import copy
import rospy
import moveit_commander
import geometry_msgs.msg
import math
from wust_robot_moveit_control_full import Wust_Robot_Moveit_Control

if __name__ == '__main__':
    wust = Wust_Robot_Moveit_Control()

    wust.Plan_Cartesian_Path_Circle( "left_arm", 1)

    #wust.Plan_Cartesian_Path_Circle( "left_arm", 2)
    #wust.Plan_Cartesian_Path_Circle( "left_arm", 3)

    #wust.Plan_Cartesian_Path_Circle2( "left_arm", 1)

    #wust.Plan_Cartesian_Path_Circle( "right_arm", 1)
