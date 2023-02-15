#!/usr/bin/env python
#coding:utf-8

import rospy
import geometry_msgs.msg
import visualization_msgs.msg
from wust_robot_moveit_control_full import *
import numpy as np
from numpy import dot
from math import sin, cos


# 机械臂工作空间
class WorkSpace:
    def __init__(self):
        rospy.init_node('visualization_marker', anonymous=False)
        marker_pub = rospy.Publisher('/visualization_marker', visualization_msgs.msg.Marker, queue_size=1)

        points = visualization_msgs.msg.Marker()
        points.ns = "visualization_marker"
        points.header.stamp = rospy.Time.now()
        points.type = visualization_msgs.msg.Marker.POINTS
        points.action = visualization_msgs.msg.Marker.ADD

        points.id = 0
        points.scale.x = 0.01
        points.scale.y = 0.01
        points.scale.z = 0.01

        points2 = visualization_msgs.msg.Marker()
        points2.ns = "visualization_marker"
        points2.header.stamp = rospy.Time.now()
        points2.type = visualization_msgs.msg.Marker.POINTS
        points2.action = visualization_msgs.msg.Marker.ADD

        points2.id = 1
        points2.scale.x = 0.01
        points2.scale.y = 0.01
        points2.scale.z = 0.01

        self.marker_pub = marker_pub
        self.points = points
        self.points2 = points2

    # 使用Marker画点
    def Display_Point(self, x, y, z, group_name):
        marker_pub = self.marker_pub
        points = self.points
        points2 = self.points2
        p = geometry_msgs.msg.Point()
        p.x = x
        p.y = y
        p.z = z
        if group_name == "left_arm":
            points.header.frame_id = "/L_Link0"
            points.color.r = 0.0
            points.color.g = 1.0
            points.color.b = 0.0
            points.color.a = 1.0  # 透明度

            points.points.append(p)
            marker_pub.publish(points)
        elif group_name == "right_arm":
            points2.header.frame_id = "/R_Link0"
            # points2.color.r = 1.0
            # points2.color.g = 1.0
            points2.color.b = 0.0
            points2.color.a = 1.0  # 透明度

            points2.points.append(p)
            marker_pub.publish(points2)
        #rospy.sleep(0.01)

    # 变换矩阵
    def transform_array(self, a, b, c, d):
        return np.array([[cos(d), -sin(d) * cos(b), sin(d) * sin(b), a * cos(d)],
                            [sin(d), cos(d) * cos(b), -cos(d) * sin(b), a * sin(d)],
                            [0, sin(b), cos(b), c],
                            [0, 0, 0, 1]])

    # 蒙特卡洛方法计算工作空间
    def Monte_carlo_Workspace(self, group_name):
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
            A1 = self.transform_array(a1, alpha1, d1, t1[i])
            A2 = self.transform_array(a2, alpha2, d2, t2[i])
            A3 = self.transform_array(a3, alpha3, d3, t3[i])
            A4 = self.transform_array(a4, alpha4, d4, t4[i])
            A5 = self.transform_array(a5, alpha5, d5, t5[i])
            A6 = self.transform_array(a6, alpha6, d6, t6[i])
            A7 = self.transform_array(a7, alpha7, d7, t7[i])
            T2 = dot(A1, A2)
            print "T2 = ", T2
            T3 = dot(T2, A3)
            T4 = dot(T3, A4)
            T5 = dot(T4, A5)
            T6 = dot(T5, A6)
            T7 = dot(T6, A7)
            point_x = T7[0, 3]
            point_y = T7[1, 3]
            point_z = T7[2, 3]
            pos = [point_x, point_y, point_z]
            self.Display_Point(point_x, point_y, point_z, group_name)


    def transform_array2(self, a, alpha, d, theta):
        return np.array([[cos(theta), -sin(theta), 0, a],
                  [sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                  [sin(theta)*sin(alpha), cos(theta)*sin(alpha), cos(alpha), cos(alpha)*d],
                  [0, 0, 0, 1]])

  # 正运动学方程
    def forward_kinematics(self, theta1, theta2, theta3, theta4, theta5, theta6, theta7, group_name):
      # Mod_DH参数
      a1 = 0
      a2 = 0
      a3 = 0
      a4 = 0
      a5 = 0
      a6 = 0
      a7 = 0

      alpha1 = 0
      alpha2 = -1.5708
      alpha3 = 1.5708
      alpha4 = -1.5708
      alpha5 = 1.5708
      alpha6 = -1.5708
      alpha7 = 1.5708

      d1 = 0.164
      d2 = 0
      d3 = 0.292
      d4 = 0
      d5 = 0.242
      d6 = 0
      d7 = 0.234

      T1 = self.transform_array2(a1, alpha1, d1, theta1)
      T2 = self.transform_array2(a2, alpha2, d2, theta2)
      #print "======STOP======", T2
      T3 = self.transform_array2(a3, alpha3, d3, theta3)
      T4 = self.transform_array2(a4, alpha4, d4, theta4)
      T5 = self.transform_array2(a5, alpha5, d5, theta5)
      T6 = self.transform_array2(a6, alpha6, d6, theta6)
      T7 = self.transform_array2(a7, alpha7, d7, theta7)

      T = T7
      #print "1.T = ", T
      T = dot(T6,T)
      #print "2.T = ", T
      T = dot(T5,T)
      #print "3.T = ", T
      T = dot(T4,T)
      #print "4.T = ", T
      T = dot(T3,T)
      #print "5.T = ", T
      T = dot(T2,T)
      #print "6.T = ", T
      T = dot(T1,T)
      #print "7.T = ", T

      point_x = T[0, 3]
      point_y = T[1, 3]
      point_z = T[2, 3]
      pos = [point_x, point_y, point_z]
      print pos
      while True:
        self.Display_Point(point_x, point_y, point_z, group_name)
        #self.Display_Point(108, -0.364, 0.260, "right_arm")

      return T

    def Vector_shouler_wrist(self, theta, group_name):
        # Mod_DH参数

        a1 = 0
        a2 = 0
        a3 = 0
        a4 = 0
        a5 = 0
        a6 = 0

        alpha1 = 0
        alpha2 = -1.57
        alpha3 = 1.57
        alpha4 = -1.57
        alpha5 = 1.57
        alpha6 = -1.57

        # d1 = 0.164
        # d2 = 0
        d3 = 0.292
        d4 = 0
        d5 = 0.242
        d6 = 0


        T3 = self.transform_array2(a3, alpha3, d3, theta[2])
        T4 = self.transform_array2(a4, alpha4, d4, theta[3])
        T5 = self.transform_array2(a5, alpha5, d5, theta[4])
        T6 = self.transform_array2(a6, alpha6, d6, theta[5])

        T = T6
        T = dot(T5, T)
        T = dot(T4, T)
        T = dot(T3, T)

        point_x = T[0, 3]
        point_y = T[1, 3]
        point_z = T[2, 3]
        pos = [point_x, point_y, point_z]
        print pos
        while True:
            self.Display_Point(point_x, point_y, point_z, group_name)

    # 球关节正运动学方程
    def Spherical_forward_kinematics(self, theta1, theta2, theta3, group_name):
        # Mod_DH参数
        a1 = 0
        a2 = 0
        a3 = 0


        alpha1 = 0
        alpha2 = -1.5708
        alpha3 = 0

        d1 = 0.164
        d2 = 0
        d3 = 0.292

        T1 = self.transform_array2(a1, alpha1, d1, theta1)
        T2 = self.transform_array2(a2, alpha2, d2, theta2)
        # print "======STOP======", T2
        T3 = self.transform_array2(a3, alpha3, d3, theta3)

        T = T3
        T = dot(T2, T)
        # print "6.T = ", T
        T = dot(T1, T)
        # print "7.T = ", T

        point_x = T[0, 3]
        point_y = T[1, 3]
        point_z = T[2, 3]
        pos = [point_x, point_y, point_z]
        print pos
        while True:
            self.Display_Point(point_x, point_y, point_z, group_name)
            # self.Display_Point(108, -0.364, 0.260, "right_arm")

        return T


if __name__ == '__main__':
    workspace = WorkSpace()
    # pos_obj = [0.95, 0, 1.02]
    # tcp = [0.95, 0, 1.02]
    # while True:
    #     tcp[2] = 1.02
    #     for i in range(15):
    #         tcp[2] = pos_obj[2] + 0.01*i
    #         print tcp[2]
    #         workspace.Display_Point(tcp[0], tcp[1], tcp[2], "left_arm")

    #TCP =  workspace.forward_kinematics(0, 0, 0, 0, 0, 0, 0, "left_arm")
    # TCP =  workspace.forward_kinematics(1.2930468619576105, -0.2343073379017514,
    #                                         1.0644097599465199, -1.942919398150267,
    #           0.5363087564734575, 1.3122057222899124, -3.025596302054855, "left_arm")
    # print TCP
    #workspace.Monte_carlo_Workspace("right_arm")

    # TCP2 =  workspace.Vector_shouler_wrist([1.2930468619576105, -0.2343073379017514,
    #                                         1.0644097599465199, -1.942919398150267,
    #           0.5363087564734575, 1.3122057222899124, -3.025596302054855], "left_arm")

    #TCP2 =  workspace.Vector_shouler_wrist([0, 0, 0, 0, 0, 0, 0], "left_arm")

    TCP3 = workspace.Spherical_forward_kinematics(0, 0, 0, "left_arm")



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








