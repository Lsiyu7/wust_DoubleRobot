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
import math
import matplotlib as mpl
import matplotlib.pyplot as plt

if __name__ == '__main__':
    wust = Wust_Robot_Moveit_Control()

    # joint = [1.140624068766987, -0.8748328661695439, 1.8315749077618269, -1.9407375568590506,
    #          0.20792902059600826, 1.6116328183868582, 0.6358516729085432]
    # arm_angle = wust.arm_angle("left_arm", joint)
    joint = [0, 0,0.0, 1.57, 0, -0.0, 0.0]
    # joint = [1.1221012789541036, -0.8191751726582512, 1.7929376782175561, -1.940609457252452,
    #          0.2459484491416406, 1.595634676615595, 0.7018777900290394]
    wust.arm_joint_control("right_arm", joint)

    #wust.arm_joint_control("left_arm", joints[4])
    # rospy.sleep(2)

    # # # 关节角
    # # pos = [0.299, 0.270, 1.002]
    # # rot = [-1.575, -0.976, -1.565]
    # # arm_angle = -3.10449203651
    # #
    # q1 = []
    # q2 = []
    # q3 = []
    # q4 = []
    # q5 = []
    # q6 = []
    # q7 = []
    # # for i in range(20):
    # #     joints = wust.arm_angle_inverse("left_arm", pos, rot, arm_angle)
    # #     q1.append(joints[5][0])
    # #     q2.append(joints[5][1])
    # #     q3.append(joints[5][2])
    # #     q4.append(joints[5][3])
    # #     q5.append(joints[5][4])
    # #     q6.append(joints[5][5])
    # #     q7.append(joints[5][6])
    # #
    # #     arm_angle = arm_angle + 0.0954
    # for i in range(5):
    #     # # #     wust.Plan_Cartesian_Path_Rot("left_arm", [0.30, 0.270, 1.003], [-1.572, -0.975, -1.570])
    #     # # #     #wust.Plan_Cartesian_Path_Rot("left_arm", [0.30, 0.230, 1.303], [-1.572, -0.975, -1.570])
    #     # wust.Plan_Cartesian_Path_Rot("left_arm", [0.30, 0, 1.303], [-0.517, -1.380, 2.390], 0)
    #     q = wust.Plan_Cartesian_Path_Rot("left_arm", [0.30, 0, 1.303], [-0.517, -1.380, 2.390], 0)
    #     wust.Plan_Cartesian_Path_Rot("left_arm", [0.30, 0, 1.003], [1.573, -1.435, 1.581], 0)
    #     wust.Plan_Cartesian_Path_Rot("left_arm", [0.30, 0.270, 1.003], [-1.572, -0.975, -1.570], 0)
    #     for j in range(len(q)):
    #         print q[j][0]
    #         q1.append(q[j][0])
    #         q2.append(q[j][1])
    #         q3.append(q[j][2])
    #         q4.append(q[j][3])
    #         q5.append(q[j][4])
    #         q6.append(q[j][5])
    #         q7.append(q[j][6])
    # q0 = []
    # for i in range(len(q1)):
    #     q0.append(i + 1)
    #
    # x = np.array(q0)
    #
    # plt.plot(q1)
    # plt.plot(q2)
    # plt.plot(q3)
    # plt.plot(q4)
    # plt.plot(q5)
    # plt.plot(q6)
    # plt.plot(q7)
    # plt.grid(True)
    # plt.axis('tight')
    # plt.xlabel('points')
    # plt.ylabel('joints')
    # plt.ylim(-3.5, 3.5)
    # #plt.xlim(0, 250)
    # plt.show()
    #
    # fileObject = open('ikfast_joints.txt', 'w')
    # fileObject.write('joint1')
    # fileObject.write('\n')
    # for i in range(len(q1)):
    #     fileObject.write(str(q1[i]))
    #     fileObject.write(', ')
    #
    # fileObject.write('\n')
    # fileObject.write('joint2')
    # fileObject.write('\n')
    # for i in range(len(q2)):
    #     fileObject.write(str(q2[i]))
    #     fileObject.write(', ')
    #
    # fileObject.write('\n')
    # fileObject.write('joint3')
    # fileObject.write('\n')
    # for i in range(len(q3)):
    #     fileObject.write(str(q3[i]))
    #     fileObject.write(', ')
    #
    # fileObject.write('\n')
    # fileObject.write('joint4')
    # fileObject.write('\n')
    # for i in range(len(q4)):
    #     fileObject.write(str(q4[i]))
    #     fileObject.write(', ')
    #
    # fileObject.write('\n')
    # fileObject.write('joint5')
    # fileObject.write('\n')
    # for i in range(len(q5)):
    #     fileObject.write(str(q5[i]))
    #     fileObject.write(', ')
    #
    # fileObject.write('\n')
    # fileObject.write('joint6')
    # fileObject.write('\n')
    # for i in range(len(q6)):
    #     fileObject.write(str(q6[i]))
    #     fileObject.write(', ')
    #
    # fileObject.write('\n')
    # fileObject.write('joint7')
    # fileObject.write('\n')
    # for i in range(len(q7)):
    #     fileObject.write(str(q7[i]))
    #     fileObject.write(', ')
    #
    # fileObject.close()

    # #臂形角
    # x0 = []
    # y0 = []
    # # for i in range(20):
    # #     y0.append(arm_angle)
    # #     arm_angle = arm_angle + 0.0954
    # for i in range(5):
    # # # #     wust.Plan_Cartesian_Path_Rot("left_arm", [0.30, 0.270, 1.003], [-1.572, -0.975, -1.570])
    # # # #     #wust.Plan_Cartesian_Path_Rot("left_arm", [0.30, 0.230, 1.303], [-1.572, -0.975, -1.570])
    #     y1 = wust.Plan_Cartesian_Path_Rot("left_arm", [0.30, 0, 1.303], [-0.517, -1.380, 2.390], 1)
    #     wust.Plan_Cartesian_Path_Rot("left_arm", [0.30, 0, 1.003], [1.573, -1.435, 1.581], 0)
    #     wust.Plan_Cartesian_Path_Rot("left_arm", [0.30, 0.270, 1.003], [-1.572, -0.975, -1.570], 0)
    #     for j in range(len(y1)):
    #         y0.append(y1[j])
    #
    # y = np.array(y0)
    #
    # for i in range(len(y)):
    #     x0.append(i+1)
    #
    # x = np.array(x0)
    # print y
    # plt.plot(y)
    # plt.grid(True)
    # plt.axis('tight')
    # plt.xlabel('points')
    # plt.ylabel('arm_angle')
    # plt.title('arm_angle')
    # plt.ylim(-3.5, 3.5)
    # plt.xlim(0, 210)
    # plt.show()
    #
    # fileObject = open('MC_Inverse_arm_angle.txt', 'w')
    # fileObject.write('arm_angle')
    # fileObject.write('\n')
    # for i in range(len(y0)):
    #     fileObject.write(str(y0[i]))
    #     fileObject.write(', ')
    #
    # fileObject.close()

    # #肘关节位置
    # x0 = []
    # y0 = []
    # # for i in range(20):
    # #     y0.append(arm_angle)
    # #     arm_angle = arm_angle + 0.0954
    # for i in range(5):
    # # # #     wust.Plan_Cartesian_Path_Rot("left_arm", [0.30, 0.270, 1.003], [-1.572, -0.975, -1.570])
    # # # #     #wust.Plan_Cartesian_Path_Rot("left_arm", [0.30, 0.230, 1.303], [-1.572, -0.975, -1.570])
    #     y1 = wust.Plan_Cartesian_Path_Rot("left_arm", [0.30, 0, 1.303], [-0.517, -1.380, 2.390], 1)
    #     wust.Plan_Cartesian_Path_Rot("left_arm", [0.30, 0, 1.003], [1.573, -1.435, 1.581], 0)
    #     wust.Plan_Cartesian_Path_Rot("left_arm", [0.30, 0.270, 1.003], [-1.572, -0.975, -1.570], 0)
    #     for j in range(len(y1)):
    #         y0.append(y1[j])