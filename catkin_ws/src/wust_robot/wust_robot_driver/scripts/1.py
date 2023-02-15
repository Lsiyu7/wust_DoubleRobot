#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import geometry_msgs.msg
from tf import transformations
from wust_robot_moveit_control_full import Wust_Robot_Moveit_Control, Object_parameters
import visualization_msgs.msg
import numpy as np
import moveit_msgs.msg
from std_msgs.msg import String
import commands
import os

if __name__ == '__main__':
    wust = Wust_Robot_Moveit_Control()

    # 向上后还原
    #wust.dual_Plan_Cartesian_Path_Inter([0, 0, 0.1], [0, 0.0, 0.1], 12)
    #rospy.sleep(1)
    #wust.dual_Plan_Cartesian_Path_Inter([0, 0, -0.1], [0, 0.0, -0.1], 12)

    # 向下后还原
    wust.dual_Plan_Cartesian_Path_Inter([0, 0, -0.1], [0, 0.0, -0.1], 12)
    rospy.sleep(1)
    wust.dual_Plan_Cartesian_Path_Inter([0, 0, 0.1], [0, 0.0, 0.1], 12)

    # # 向左后还原
    wust.dual_Plan_Cartesian_Path_Inter([0, 0.1, 0], [0, 0.1, 0], 12)
    rospy.sleep(1)
    wust.dual_Plan_Cartesian_Path_Inter([0, -0.1, 0], [0, -0.1, 0], 12)
    #
    # # 向右后还原
    wust.dual_Plan_Cartesian_Path_Inter([0, -0.1, 0], [0, -0.1, 0], 12)
    rospy.sleep(1)
    wust.dual_Plan_Cartesian_Path_Inter([0, 0.1, 0], [0, 0.1, 0], 12)
    #

