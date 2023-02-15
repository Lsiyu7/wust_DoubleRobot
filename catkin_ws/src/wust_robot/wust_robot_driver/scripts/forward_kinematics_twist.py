#!/usr/bin/env python
#coding:utf-8

import rospy
import geometry_msgs.msg
import visualization_msgs.msg
from mrp2a_moveit_control_full import Mrp2a_Moveit_Control, Object_parameters
from tf import transformations
import numpy as np
import math
from math import sin, cos


# 机械臂运动学(旋量法)
class Kinematics_Twist:
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

        # 机械臂参数
        L1 = 0.164
        L2 = 0.292
        L3 = 0.242
        L4 = 0.234

        pai = 3.14159

        # 单位矩阵
        unit_matrix = np.array([[1, 0, 0],
                                [0, 1, 0],
                                [0, 0, 1]])

        # 初始姿态矩阵
        g0 = np.array([[1, 0, 0, 0],
                       [0, 1, 0, 0],
                       [0, 0, 1, L1 + L2 + L3 + L4],
                       [0, 0, 0, 1]])

        # T1 = np.array([[0], [0], [0], [0], [0], [1]])
        # T2 = np.array([[-L1], [0], [0], [0], [1], [0]])
        # T3 = np.array([[0], [0], [0], [0], [0], [1]])
        # T4 = np.array([[-(L1 + L2)], [0], [0], [0], [1], [0]])
        # T5 = np.array([[0], [0], [0], [0], [0], [1]])
        # T6 = np.array([[-(L1 + L2 + L3)], [0], [0], [0], [1], [0]])
        # T7 = np.array([[0], [0], [0], [0], [0], [1]])

        w1 = np.array([[0], [0], [1]])
        w2 = np.array([[0], [1], [0]])
        w3 = np.array([[0], [0], [1]])
        w4 = np.array([[0], [1], [0]])
        w5 = np.array([[0], [0], [1]])
        w6 = np.array([[0], [1], [0]])
        w7 = np.array([[0], [0], [1]])

        q1 = np.array([[0], [0], [0]])
        q2 = np.array([[0], [0], [L1]])
        q3 = np.array([[0], [0], [L1]])
        q4 = np.array([[0], [0], [L1 + L2]])
        q5 = np.array([[0], [0], [L1 + L2]])
        q6 = np.array([[0], [0], [L1 + L2 + L3]])
        q7 = np.array([[0], [0], [L1 + L2 + L3 + L4]])

        # q1 = [0, 0, 0]
        # q2 = [0, 0, L1]
        # q3 = [0, 0, L1]
        # q4 = [0, 0, L1 + L2]
        # q5 = [0, 0, L1 + L2]
        # q6 = [0, 0, L1 + L2 + L3]
        # q7 = [0, 0, L1 + L2 + L3 + L4]

        self.marker_pub = marker_pub
        self.points = points
        self.points2 = points2

        self.unit_matrix = unit_matrix
        self.g0 = g0
        self.pai = pai

        self.w1 = w1
        self.w2 = w2
        self.w3 = w3
        self.w4 = w4
        self.w5 = w5
        self.w6 = w6
        self.w7 = w7

        self.q1 = q1
        self.q2 = q2
        self.q3 = q3
        self.q4 = q4
        self.q5 = q5
        self.q6 = q6
        self.q7 = q7

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
            points.color.r = 1.0
            points.color.g = 0.0
            points.color.b = 0.0
            points.color.a = 1.0  # 透明度

            points.points.append(p)
            marker_pub.publish(points)
        elif group_name == "right_arm":
            points2.header.frame_id = "/L_Link0"
            points2.color.r = 1.0
            points2.color.g = 0.0
            points2.color.b = 0.0
            points2.color.a = 1.0  # 透明度

            points2.points.append(p)
            marker_pub.publish(points2)
        # rospy.sleep(0.01)

    # 旋转角
    def Rotation_axis(self,w,theta):
        # 反对称矩阵
        skew_symmetric_matrix = np.array([[0,-w[2],w[1]],
                                          [w[2],0,-w[0]],
                                          [-w[1],w[0],0]])
        unit_matrix = self.unit_matrix

        # 罗德格里斯公式
        Rodrigucs = unit_matrix + skew_symmetric_matrix*sin(theta) + np.dot(skew_symmetric_matrix,
                                                                            skew_symmetric_matrix)*(1-cos(theta))

        return Rodrigucs

    # 运动旋量
    def Exp_twist(self,w,q,theta):
        unit_matrix = self.unit_matrix

        # 转化为行向量
        w_vector = w.reshape((-1))
        q_vector = q.reshape((-1))

        # 线速度
        v = -np.cross(w_vector,q_vector)

        rotation_axis = self.Rotation_axis(w,theta)

        # 矩阵转置
        w_trans = np.transpose(w)

        # 位置
        P1 = unit_matrix - rotation_axis
        P2 = np.cross(w_vector,v)
        P3 = np.dot(P1,P2)

        P4 = np.dot(w,w_trans)
        P5 = np.dot(P4,v)
        P6 = P5*theta

        position = P3 + P6

        #rotation_axis[0][3] = P7[0]
        # 旋量公式
        exp_twist = np.array([[rotation_axis[0][0], rotation_axis[0][1], rotation_axis[0][2], position[0]],
                              [rotation_axis[1][0], rotation_axis[1][1], rotation_axis[1][2], position[1]],
                              [rotation_axis[2][0], rotation_axis[2][1], rotation_axis[2][2], position[2]],
                              [0, 0, 0, 1]])

        return exp_twist

    # 臂形角方法求逆解
    def arm_angel_inverse(self, group_name, pos, rot, arm_angel):

        pai = self.pai
        d3 = np.linalg.norm([0.004, -0.292, -0.002])
        d5 = np.linalg.norm([0.003, -0.242, -0.015])

        T = transformations.euler_matrix(rot[0], rot[1], rot[2])

        if group_name == "left_arm":


            T_s_from_b = np.array([[1, 0, 0, -0.240],
                                   [0, 1, 0, 0.245],
                                   [0, 0, 1, 1.155],
                                   [0, 0, 0, 1]])        # 设定一个左臂肩坐标系,位置在肩关节中心,方向与base_link相同

            T_b_from_s = np.array([[1, 0, 0, 0.240],     # base_link相对于s坐标系的变换矩阵
                                   [0, 1, 0, -0.245],
                                   [0, 0, 1, -1.155],
                                   [0, 0, 0, 1]])

            T_t_from_b = np.array([[T[0][0], T[0][1], T[0][2], pos[0]],
                                   [T[1][0], T[1][1], T[1][2], pos[1]],
                                   [T[2][0], T[2][1], T[2][2], pos[2]],
                                   [0, 0, 0, 1]])

            T_t_from_s = np.dot(T_b_from_s, T_t_from_b)


            R_t_from_s = np.array([[T_t_from_s[0][0], T_t_from_s[0][1], T_t_from_s[0][2]],
                                   [T_t_from_s[1][0], T_t_from_s[1][1], T_t_from_s[1][2]],
                                   [T_t_from_s[2][0], T_t_from_s[2][1], T_t_from_s[2][2]]])


            # 求肘关节角
            st_from_s = np.array([[T_t_from_s[0][3]], [T_t_from_s[1][3]], [T_t_from_s[2][3]]])  #向量st

            wt = np.array([[0.003], [-0.234], [-0.035]])  #机械臂w坐标系到末端坐标系的向量(相对于w坐标系)
            tw = np.array([[0.00], [0.035], [-0.234]])
            tw_from_s = np.dot(R_t_from_s, tw)
            wt_from_s = -tw_from_s   #向量wt在基标系中的值
            sw_from_s = st_from_s - wt_from_s  #机械臂基坐标系到s坐标系的向量(相对于基坐标系)
            sw_length = np.linalg.norm(sw_from_s)  #向量sw的模
            ctheta4 = (math.pow(d3,2) + math.pow(d5,2) - math.pow(sw_length, 2))/(2*d3*d5)
                                                    #余弦定理求cos theta4
            print sw_from_s
            theta4 = pai - math.acos(ctheta4)
            print theta4

            # x = np.linalg.norm([[1], [1], [2]])
            # print "x = " ,x

            # 求肩关节
            sw_unit_from_s = sw_from_s/sw_length

            calpha = (math.pow(d3,2) + math.pow(sw_length,2) - math.pow(d5, 2))/(2*d3*sw_length)
            alpha = math.acos(calpha)

            w_sw = sw_from_s/sw_length    #旋量参数w
            q_sw = sw_from_s              #旋量参数q

            v = np.array([[0], [0], [1]])
            w_ref = np.cross(sw_unit_from_s.reshape((-1)), v.reshape((-1)))    #参考平面旋量参数
            q_ref = np.array([[0], [0], [0]])

            Exp_sw = self.Exp_twist(w_sw, q_sw, arm_angel)
            Exp_ref = self.Exp_twist(w_ref, q_ref, alpha)     #旋量公式

            P_e_from_s0 = np.dot(Exp_sw, Exp_ref)
            d3_sw = np.array([[d3 * sw_unit_from_s[0][0]],
                              [d3 * sw_unit_from_s[1][0]],
                              [d3 * sw_unit_from_s[2][0]],
                              [1]])                           #d3乘以向量sw
            P_e_from_s = np.dot(P_e_from_s0, d3_sw)           #求出肘关节中心e的坐标
            print P_e_from_s

            theta1 = -math.atan2(P_e_from_s[2][0], P_e_from_s[0][0])
            print theta1

        # if group_name == "right_arm":
        #     bs_from_b = np.array([0.000, 0.052, 0.175])  # 机械臂基坐标系到s坐标系的向量(向量相对于基坐标系)
        #     wt = np.array([0.003, -0.234, -0.035])  # 机械臂w坐标系到末端坐标系的向量(相对于w坐标系)
        #     tw = np.array([0.00, 0.035, -0.234])
        #     tw_from_b = np.dot(R, tw)
        #     wt_from_b = -tw_from_b  # 向量wt在基标系中的值
        #     sw_from_b = bt_from_b - bs_from_b - wt_from_b  # 机械臂基坐标系到s坐标系的向量(相对于基坐标系)
        #     sw_length = np.linalg.norm(sw_from_b)  # 向量sw的模
        #
        #     ctheta4 = (math.pow(d3, 2) + math.pow(d5, 2) - math.pow(sw_length, 2)) / (2 * d3 * d5)
        #     # 余弦定理求cos theta4
        #     print ctheta4



    def test(self):
        # w2 = self.w2
        # w1 = w2.reshape((-1))
        # w3 = w2.reshape((-1,1))
        # print w1
        # print w3

        w2 = self.w2
        w1 = np.array([[0,1,0]])
        w3 = np.transpose(w1)
        print np.dot(w3,w1)
        print np.dot(w2,np.transpose(w2))

if __name__ == '__main__':
    mrp2a = Kinematics_Twist()

    # pos = [0.080, 0.690, 0.746]
    # rot = [-3.139, 0.561, -2.098]
    #
    # mrp2a.arm_angel_inverse("left_arm", pos,rot, 0)

    #mrp2a.arm_angel_inverse("right_arm", pos_right,rot_right, 0)

    # print"===="
    # print cos(1.0)
    # print cos(-0.5)

    g0 = np.array([[1, 0, 0, 0],
                   [0, 1, 0, 0],
                   [0, 0, 1, 0.164+0.292],
                   [0, 0, 0, 1]])
    pai = mrp2a.pai
    #
    w1 = mrp2a.w1
    w2 = mrp2a.w2
    w3 = mrp2a.w3
    w4 = mrp2a.w4
    # w5 = mrp2a.w5
    # w6 = mrp2a.w6
    # w7 = mrp2a.w7
    #
    q1 = mrp2a.q1
    q2 = mrp2a.q2
    q3 = mrp2a.q3
    q4 = mrp2a.q4
    # q5 = mrp2a.q5
    # q6 = mrp2a.q6
    # q7 = mrp2a.q7
    #
    # # theta1 = 60*pai/180
    # # theta2 = 60*pai/180
    # # theta3 = 60*pai/180
    # # theta4 = 60*pai/180
    # # theta5 = 60*pai/180
    # # theta6 = 60*pai/180
    # # theta7 = 60*pai/180
    theta1 = 0
    theta2 = 0
    theta3 = 0
    theta4 = 0
    # theta5 = 0
    # theta6 = 0
    # theta7 = 0
    #
    E1 = mrp2a.Exp_twist(w1,q1,theta1)
    E2 = mrp2a.Exp_twist(w2,q2,theta2)
    E3 = mrp2a.Exp_twist(w3,q3,theta3)
    E4 = mrp2a.Exp_twist(w4,q4,theta4)
    # E5 = mrp2a.Exp_twist(w5,q5,theta5)
    # E6 = mrp2a.Exp_twist(w6,q6,theta6)
    # E7 = mrp2a.Exp_twist(w7,q7,theta7)
    #
    T1 = E1
    T2 = np.dot(T1,E2)
    T3 = np.dot(T2,E3)
    T4 = np.dot(T3,E4)
    # T5 = np.dot(T4,E5)
    # T6 = np.dot(T5,E6)
    # T7 = np.dot(T6,E7)
    # T = np.dot(T7,g0)
    T = np.dot(T4, g0)
    print T
    print T[1,3]
    while True:
        mrp2a.Display_Point(T[0,3], T[1,3], T[2,3], "left_arm")
    mrp2a.test()