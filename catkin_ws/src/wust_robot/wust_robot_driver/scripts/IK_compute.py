#!/usr/bin/env python
#coding:utf-8

import rospy
import geometry_msgs.msg
import visualization_msgs.msg
from mrp2a_moveit_control_full import Mrp2a_Moveit_Control, Object_parameters
import numpy as np
from numpy import dot
from math import sin, cos


# 机械臂逆解测试程序
class IK_Compute:
    def __init__(self):
        #rospy.init_node('IK_Sample_Points', anonymous=False)
        marker_pub = rospy.Publisher('/IK_Sample_Points', visualization_msgs.msg.Marker, queue_size=1)

        points = visualization_msgs.msg.Marker()
        points.ns = "IK_Sample_Points"
        points.header.stamp = rospy.Time.now()
        points.type = visualization_msgs.msg.Marker.POINTS
        points.action = visualization_msgs.msg.Marker.ADD
        points.header.frame_id = "/base_link"

        points.id = 0
        points.scale.x = 0.005
        points.scale.y = 0.005
        points.scale.z = 0.005

        points2 = visualization_msgs.msg.Marker()
        points2.ns = "IK_Sample_Points"
        points2.header.stamp = rospy.Time.now()
        points2.type = visualization_msgs.msg.Marker.POINTS
        points2.action = visualization_msgs.msg.Marker.ADD
        points2.header.frame_id = "/base_link"

        points2.id = 1
        points2.scale.x = 0.005
        points2.scale.y = 0.005
        points2.scale.z = 0.005

        self.marker_pub = marker_pub
        self.points = points
        self.points2 = points2

    def Display_Point(self, x, y, z, result):
        marker_pub = self.marker_pub
        points = self.points
        points2 = self.points2
        p = geometry_msgs.msg.Point()
        p.x = x
        p.y = y
        p.z = z
        if result == "success":
            # points.color.r = 1.0
            points.color.g = 1.0
            # points.color.b = 1.0
            points.color.a = 1.0  # 透明度

            points.points.append(p)
            marker_pub.publish(points)
        elif result == "failure":

            points2.color.r = 1.0
            # points2.color.g = 1.0
            # points2.color.b = 1.0
            points2.color.a = 1.0  # 透明度

            points2.points.append(p)
            marker_pub.publish(points2)

if __name__ == '__main__':
    mrp2a = Mrp2a_Moveit_Control()
    ik_test = IK_Compute()

    n = 0
    m = 0
    pos = [1.15, 0, 1.508]
    rot = [-1.57, -1.57, -1.57]

    pos2 = [1.2, 0, 1.538]
    rot2 = [-1.57, 1.57, -1.57]

    mrp2a.arm_pose_control("right_arm", pos, rot)
    rospy.sleep(5)

    i = 1
    pos[0] = 1.2
    while i <= 10:
        pos[0] = pos[0] - 0.01
        j = 1
        pos[1] = -0.05
        while j <= 35:
            pos[1] = pos[1] - 0.01
            k = 1
            pos[2] = 1.558
            while k <= 30:
                pos[2] = pos[2] - 0.01
                print pos
                if mrp2a.sample_Plan_Cartesian_Path("right_arm", pos) == 0:
                    ik_test.Display_Point(pos[0], pos[1], pos[2], "failure")
                    n = n + 1
                elif mrp2a.sample_Plan_Cartesian_Path("right_arm", pos) == 1:
                    ik_test.Display_Point(pos[0], pos[1], pos[2], "success")
                    m = m + 1
                k = k + 1
            j = j + 1
        i = i + 1

    # x = 1
    # pos2[0] = 1.2
    # while x <= 2:
    #     pos2[0] = pos2[0] - 0.01
    #     y = 1
    #     pos2[1] = 0
    #     while y <= 45:
    #         pos2[1] = pos2[1] + 0.01
    #         z = 1
    #         pos2[2] = 1.558
    #         while z <= 25:
    #             pos2[2] = pos2[2] - 0.01
    #             print pos2
    #             if mrp2a.sample_arm_pose_control("left_arm", pos2, rot2) == 0:
    #                 ik_test.Display_Point(pos2[0], pos2[1], pos2[2], "failure")
    #                 n = n + 1
    #             elif mrp2a.sample_arm_pose_control("left_arm", pos2, rot2) == 1:
    #                 ik_test.Display_Point(pos2[0], pos2[1], pos2[2], "success")
    #                 m = m + 1
    #             z = z + 1
    #         y = y + 1
    #     x = x + 1

    print m
    print n