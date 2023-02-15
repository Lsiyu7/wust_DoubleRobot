#!/usr/bin/env python
#coding:utf-8

import rospy
import geometry_msgs.msg
import visualization_msgs.msg
from mrp2a_moveit_control_full import Mrp2a_Moveit_Control, Object_parameters
import numpy as np
from numpy import dot
from math import sin, cos


# 机械臂工作空间
class WorkSpace:
    def __init__(self):
        #rospy.init_node('visualization_marker', anonymous=False)
        marker_pub = rospy.Publisher('/visualization_marker', visualization_msgs.msg.Marker, queue_size=1)

        points = visualization_msgs.msg.Marker()
        points.ns = "visualization_marker"
        points.header.stamp = rospy.Time.now()
        points.type = visualization_msgs.msg.Marker.POINTS
        points.action = visualization_msgs.msg.Marker.ADD
        points.header.frame_id = "/R_Link0"

        points.id = 0
        points.scale.x = 0.005
        points.scale.y = 0.005
        points.scale.z = 0.005

        points2 = visualization_msgs.msg.Marker()
        points2.ns = "visualization_marker"
        points2.header.stamp = rospy.Time.now()
        points2.type = visualization_msgs.msg.Marker.POINTS
        points2.action = visualization_msgs.msg.Marker.ADD
        points2.header.frame_id = "/R_Link0"

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

    def transform_array(self, a, b, c, d):
        return np.array([[cos(d), -sin(d) * cos(b), sin(d) * sin(b), a * cos(d)],
                            [sin(d), cos(d) * cos(b), -cos(d) * sin(b), a * sin(d)],
                            [0, sin(b), cos(b), c],
                            [0, 0, 0, 1]])


if __name__ == '__main__':
    mrp2a = Mrp2a_Moveit_Control()
    workspace = WorkSpace()
    a1 = 0
    a2 = 0
    a3 = 0
    a4 = 0
    a5 = 0
    a6 = 0
    a7 = 0

    alpha1 = -1.57
    alpha2 = 1.57
    alpha3 = -1.57
    alpha4 = 1.57
    alpha5 = -1.57
    alpha6 = 1.57
    alpha7 = 0

    d1 = 0.15
    d2 = 0
    d3 = 0.30
    d4 = 0
    d5 = 0.25
    d6 = 0
    d7 = 0.22

    t1_min = -3.0
    t2_min = -2.0
    t3_min = -3.0
    # t4_min = -2.0
    t4_min = 0
    t5_min = -3.0
    t6_min = -2.0
    t7_min = -3.0
    t1_max = 3.0
    t2_max = 2.0
    t3_max = 3.0
    t4_max = 2.0
    t5_max = 3.0
    t6_max = 2.0
    t7_max = 3.0

    N = 40000
    t1 = []
    t2 = []
    t3 = []
    t4 = []
    t5 = []
    t6 = []
    t7 = []
    for i in range(0, N, 1):
        # t1 = t1_min + (t1_max-t1_min)*np.random.random()
        # t2 = t2_min + (t2_max-t2_min)*np.random.random()
        # t3 = t3_min + (t3_max-t3_min)*np.random.random()
        # t4 = t4_min + (t4_max-t4_min)*np.random.random()
        # t5 = t5_min + (t5_max-t5_min)*np.random.random()
        # t6 = t6_min + (t6_max-t6_min)*np.random.random()
        # t7 = t7_min + (t7_max-t7_min)*np.random.random()
        t1.append(t1_min + (t1_max - t1_min) * np.random.random())
        t2.append(t2_min + (t2_max - t2_min) * np.random.random())
        t3.append(t3_min + (t3_max - t3_min) * np.random.random())
        t4.append(t4_min + (t4_max - t4_min) * np.random.random())
        t5.append(t5_min + (t5_max - t5_min) * np.random.random())
        t6.append(t6_min + (t6_max - t6_min) * np.random.random())
        t7.append(t7_min + (t7_max - t7_min) * np.random.random())

    for i in range(0, N, 1):
        A1 = workspace.transform_array(a1, alpha1, d1, t1[i])
        A2 = workspace.transform_array(a2, alpha2, d2, t2[i])
        A3 = workspace.transform_array(a3, alpha3, d3, t3[i])
        A4 = workspace.transform_array(a4, alpha4, d4, t4[i])
        A5 = workspace.transform_array(a5, alpha5, d5, t5[i])
        A6 = workspace.transform_array(a6, alpha6, d6, t6[i])
        A7 = workspace.transform_array(a7, alpha7, d7, t7[i])
        T2 = dot(A1, A2)
        T3 = dot(T2, A3)
        T4 = dot(T3, A4)
        T5 = dot(T4, A5)
        T6 = dot(T5, A6)
        T7 = dot(T6, A7)
        point_x = T7[0, 3]
        point_y = T7[1, 3]
        point_z = T7[2, 3]
        pos = [point_x, point_y, point_z]
        if mrp2a.sample_Plan_Cartesian_Path("right_arm", pos) == 0:
            workspace.Display_Point(pos[0], pos[1], pos[2], "failure")
        elif mrp2a.sample_Plan_Cartesian_Path("right_arm", pos) == 1:
            workspace.Display_Point(pos[0], pos[1], pos[2], "success")
        # self.Display_Point(point_x, point_y, point_z, group_name)

    # angle = 57.296   # 角度
    # rad = 0.0174   # 弧度
    #
    # q1_s = -150
    # q1_end = 150
    # q2_s = -100
    # q2_end = 100
    # q3_s = -150
    # q3_end = 150
    # q4_s = -100
    # q4_end = 100
    # q5_s = -150
    # q5_end = 150
    # q6_s = -100
    # q6_end = 100
    # q7_s = -150
    # q7_end = 150
    #
    # step = 60
    #
    # i = 1
    # T_cell = {((q1_end-q1_s)/step)*((q2_end-q2_s)/step)*((q3_end-q3_s)/step)*((q4_end-q4_s)/step)* \
    #           ((q5_end-q5_s)/step)*((q6_end-q6_s)/step)*((q7_end-q7_s)/step)}
    #
    # # for  q1=q1_s:step:q1_end
    # #     for  q2=q2_s:step:q2_end
    # #           for  q3=q3_s:step:q3_end
    # #               for  q4=q4_s:step:q4_end
    # #                   for q5=q5_s:step:q5_end
    # #                       for q6=q6_s:step:q6_end
    # #                               q =[q1*du q2*du q3*du...
    # #                                   q4*du q5*du q6*du 0*du];
    # for q1 in range(q1_s, q1_end, step):
    #     for q2 in range(q2_s, q2_end, step):
    #         for q3 in range(q3_s, q3_end, step):
    #             for q4 in range(q4_s, q4_end, step):
    #                 for q5 in range(q5_s, q5_end, step):
    #                     for q6 in range(q6_s, q6_end, step):
    #                         for q7 in range(q7_s, q7_end, step):
    #                             q = [q1*rad, q2*rad, q3*rad, q4*rad, q5*rad, q6*rad, q7*rad]
    #                             print q
    #                             mrp2a.arm_joint_control("right_arm", q)
    #                             i = i + 1
    #
    # print "i = ", i
    # rospy.sleep(1)









