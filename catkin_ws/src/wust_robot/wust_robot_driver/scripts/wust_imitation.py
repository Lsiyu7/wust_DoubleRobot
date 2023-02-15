#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
import geometry_msgs.msg
from tf import transformations
from wust_robot_moveit_control_full import Wust_Robot_Moveit_Control, Object_parameters
import visualization_msgs.msg
import numpy
from scipy.spatial.transform import Rotation
import moveit_msgs.msg
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import commands
import os
import csv
import time
   
"""    
def q2rot(w,x,y,z):
    T00 = 1 - 2 * pow(y, 2) - 2 * pow(z, 2)
    T01 = 2 * (x * y - w * z)
    T02 = 2 * (x * z + w * y)

    T10 = 2 * (x * y + w * z)
    T11 = 1 - 2 * pow(x, 2) - 2 * pow(z, 2)
    T12= 2 * (y * z - w * x)

    T20 = 2 * (x * z - w * y)
    T21= 2 * (y * z + w * x)
    T22 = 1 - 2 * pow(x, 2) - 2 * pow(y, 2)
    T = numpy.array([[T00, T01, T02],
                    [T10, T11, T12],
                    [T20, T21, T22]])
    return T
"""
def read_Skeleton_Quaternion(csv_reader):
    Time = []
    q_rs_w = []
    q_rs_x = []
    q_rs_y = []
    q_rs_z = []
    q_ru_w = []
    q_ru_x = []
    q_ru_y = []
    q_ru_z = []
    q_rf_w = []
    q_rf_x = []
    q_rf_y = []
    q_rf_z = []
    q_rh_w = []
    q_rh_x = []
    q_rh_y = []
    q_rh_z = []

    q_ls_w = []
    q_ls_x = []
    q_ls_y = []
    q_ls_z = []
    q_lu_w = []
    q_lu_x = []
    q_lu_y = []
    q_lu_z = []
    q_lf_w = []
    q_lf_x = []
    q_lf_y = []
    q_lf_z = []
    q_lh_w = []
    q_lh_x = []
    q_lh_y = []
    q_lh_z = []

    q_rs = []
    q_ru = []
    q_rf = []
    q_rh = []
    q_r = []

    q_ls = []
    q_l = []

    for i in csv_reader:
        t = float(i['Time(Second)'])
        RightShoulder_w = float(i['Skeleton Quaternion:RightShoulder-W'])
        RightShoulder_x = float(i['Skeleton Quaternion:RightShoulder-X'])
        RightShoulder_y = float(i['Skeleton Quaternion:RightShoulder-Y'])
        RightShoulder_z = float(i['Skeleton Quaternion:RightShoulder-Z'])
        RightUpperArm_w = float(i['Skeleton Quaternion:RightUpperArm-W'])
        RightUpperArm_x = float(i['Skeleton Quaternion:RightUpperArm-X'])
        RightUpperArm_y = float(i['Skeleton Quaternion:RightUpperArm-Y'])
        RightUpperArm_z = float(i['Skeleton Quaternion:RightUpperArm-Z'])
        RightForeArm_w = float(i['Skeleton Quaternion:RightForeArm-W'])
        RightForeArm_x = float(i['Skeleton Quaternion:RightForeArm-X'])
        RightForeArm_y = float(i['Skeleton Quaternion:RightForeArm-Y'])
        RightForeArm_z = float(i['Skeleton Quaternion:RightForeArm-Z'])
        RightHand_w = float(i['Skeleton Quaternion:RightHand-W'])
        RightHand_x = float(i['Skeleton Quaternion:RightHand-X'])
        RightHand_y = float(i['Skeleton Quaternion:RightHand-Y'])
        RightHand_z = float(i['Skeleton Quaternion:RightHand-Z'])

        LeftShoulder_w = float(i['Skeleton Quaternion:LeftShoulder-W'])
        LeftShoulder_x = float(i['Skeleton Quaternion:LeftShoulder-X'])
        LeftShoulder_y = float(i['Skeleton Quaternion:LeftShoulder-Y'])
        LeftShoulder_z = float(i['Skeleton Quaternion:LeftShoulder-Z'])
        LeftUpperArm_w = float(i['Skeleton Quaternion:LeftUpperArm-W'])
        LeftUpperArm_x = float(i['Skeleton Quaternion:LeftUpperArm-X'])
        LeftUpperArm_y = float(i['Skeleton Quaternion:LeftUpperArm-Y'])
        LeftUpperArm_z = float(i['Skeleton Quaternion:LeftUpperArm-Z'])
        LeftForeArm_w = float(i['Skeleton Quaternion:LeftForeArm-W'])
        LeftForeArm_x = float(i['Skeleton Quaternion:LeftForeArm-X'])
        LeftForeArm_y = float(i['Skeleton Quaternion:LeftForeArm-Y'])
        LeftForeArm_z = float(i['Skeleton Quaternion:LeftForeArm-Z'])
        LeftHand_w = float(i['Skeleton Quaternion:LeftHand-W'])
        LeftHand_x = float(i['Skeleton Quaternion:LeftHand-X'])
        LeftHand_y = float(i['Skeleton Quaternion:LeftHand-Y'])
        LeftHand_z = float(i['Skeleton Quaternion:LeftHand-Z'])

        Time.append(t)
        q_rs_w.append(RightShoulder_w)
        q_rs_x.append(RightShoulder_x)
        q_rs_y.append(RightShoulder_y)
        q_rs_z.append(RightShoulder_z)
        q_ru_w.append(RightUpperArm_w)
        q_ru_x.append(RightUpperArm_x)
        q_ru_y.append(RightUpperArm_y)
        q_ru_z.append(RightUpperArm_z)
        q_rf_w.append(RightForeArm_w)
        q_rf_x.append(RightForeArm_x)
        q_rf_y.append(RightForeArm_y)
        q_rf_z.append(RightForeArm_z)
        q_rh_w.append(RightHand_w)
        q_rh_x.append(RightHand_x)
        q_rh_y.append(RightHand_y)
        q_rh_z.append(RightHand_z)

        q_ls_w.append(LeftShoulder_w)
        q_ls_x.append(LeftShoulder_x)
        q_ls_y.append(LeftShoulder_y)
        q_ls_z.append(LeftShoulder_z)
        q_lu_w.append(LeftUpperArm_w)
        q_lu_x.append(LeftUpperArm_x)
        q_lu_y.append(LeftUpperArm_y)
        q_lu_z.append(LeftUpperArm_z)
        q_lf_w.append(LeftForeArm_w)
        q_lf_x.append(LeftForeArm_x)
        q_lf_y.append(LeftForeArm_y)
        q_lf_z.append(LeftForeArm_z)
        q_lh_w.append(LeftHand_w)
        q_lh_x.append(LeftHand_x)
        q_lh_y.append(LeftHand_y)
        q_lh_z.append(LeftHand_z)
        """
        q_r.append(numpy.array([[RightShoulder_w, RightShoulder_x, RightShoulder_y, RightShoulder_z],
                                [RightUpperArm_w, RightUpperArm_x, RightUpperArm_y, RightUpperArm_z],
                                [RightForeArm_w, RightForeArm_x, RightForeArm_y, RightForeArm_z],
                                [RightHand_w, RightHand_x, RightHand_y, RightHand_z]]))
        q_l.append(numpy.array([[LeftShoulder_w, LeftShoulder_x, LeftShoulder_y, LeftShoulder_z],
                                [LeftUpperArm_w, LeftUpperArm_x, LeftUpperArm_y, LeftUpperArm_z],
                                [LeftForeArm_w, LeftForeArm_x, LeftForeArm_y, LeftForeArm_z],
                                [LeftHand_w, LeftHand_x, LeftHand_y, LeftHand_z]]))
        """

    return (Time, q_rs_w, q_rs_x, q_rs_y, q_rs_z, q_ru_w, q_ru_x, q_ru_y, q_ru_z, q_rf_w, q_rf_x, q_rf_y, q_rf_z, q_rh_w, q_rh_x, q_rh_y, q_rh_z,
            q_ls_w, q_ls_x, q_ls_y, q_ls_z, q_lu_w, q_lu_x, q_lu_y, q_lu_z, q_lf_w, q_lf_x, q_lf_y, q_lf_z, q_lh_w, q_lh_x, q_lh_y, q_lh_z)
    #return (Time, q_r, q_l)

# 四元数转换为球关节求逆解
  # 球关节逆解
def SphericalJoint_inverse(R):
      # theta1 = []
      # theta2 = []
      # theta3 = []

      # theta2 > 0
      theta1_1 = math.atan2(R[1][2], R[0][2])
      theta2_1 = math.atan2(math.sqrt(math.pow(R[0][2], 2) + math.pow(R[1][2], 2)), R[2][2])
      theta3_1 = math.atan2(R[2][1], -R[2][0])

      # theta2 < 0
      theta1_2 = math.atan2(-R[1][2], -R[0][2])
      theta2_2 = math.atan2(-math.sqrt(math.pow(R[0][2], 2) + math.pow(R[1][2], 2)), R[2][2])
      theta3_2 = math.atan2(-R[2][1], R[2][0])

      thetas = []
      thetas1 = []
      thetas2 = []
      thetas1.append(theta1_1)
      thetas1.append(theta2_1)
      thetas1.append(theta3_1)
      thetas2.append(theta1_2)
      thetas2.append(theta2_2)
      thetas2.append(theta3_2)
      thetas.append(thetas1)
      thetas.append(thetas2)
      #print "SphericalJoint_inverse", thetas
      return thetas2
def inverse(group_name,Q):
    #print(Q)
    a=Q[0]
    b=Q[1]
    c=Q[2]
    d=Q[3]
    R1=Rotation(Q)
    ro = R1.as_matrix()
    R=numpy.array([[1-2*math.pow(c,2)-2*math.pow(d,2), 2*b*c-2*a*d, 2*a*c+2*b*d],
                    [2*b*c+2*a*d, 1-2*math.pow(b,2)-2*math.pow(d,2), 2*c*d-2*a*b],
                    [2*b*d-2*a*c, 2*a*b+2*c*d, 1-2*math.pow(b,2)-2*math.pow(c,2)]])
    
    #human to T_B
    H_to_B=numpy.array([[0,1,0],[1,0,0],[0,0,1]])
    if group_name == "left_arm":
        T_base = transformations.euler_matrix(1.583, -0.525, -1.577)
        T_base = numpy.array([[T_base[0][0], T_base[0][1], T_base[0][2]],
                             [T_base[1][0], T_base[1][1], T_base[1][2]],
                             [T_base[2][0], T_base[2][1], T_base[2][2]]])

          #末端相对于双臂基坐标系的位姿
        T_from_b =numpy.dot(H_to_B,R)

          #末端相对于手臂基座的位姿
        T = numpy.dot(T_base, T_from_b)
    elif group_name == "right_arm":
        T_base = transformations.euler_matrix(-1.561, -0.522, -1.576)
        T_base = numpy.array([[T_base[0][0], T_base[0][1], T_base[0][2]],
                             [T_base[1][0], T_base[1][1], T_base[1][2]],
                             [T_base[2][0], T_base[2][1], T_base[2][2]]])

          #末端相对于双臂基坐标系的位姿
        T_from_b =numpy.dot(H_to_B,R)

          #末端相对于手臂基座的位姿
        T = numpy.dot(T_base, T_from_b)

    theta = SphericalJoint_inverse(T)
    return theta

def publish(JS):
    pub = rospy.Publisher('joint_states', JointState, queue_size=30)
    rate = rospy.Rate(50)
    joint_states = JointState()
    joint_states.header = Header()
    joint_states.header.stamp = rospy.Time.now()
    joint_states.name = ['wheel_joint1', 'wheel_joint2', 'wheel_joint3', 'L_joint1', 'L_joint2', 'L_joint3', 'L_joint4',
    'L_joint5', 'L_joint6', 'L_joint7', 'L_left_joint', 'L_right_joint', 'R_joint1', 'R_joint2', 'R_joint3',
    'R_joint4', 'R_joint5', 'R_joint6', 'R_joint7', 'R_left_joint', 'R_right_joint', 'H_joint1', 'H_joint2']
    joint_states.position =JS 
    joint_states.velocity = []
    joint_states.effort = []
    pub.publish(joint_states)
    #rospy.loginfo(joint_states)
    rate.sleep()

if __name__ == '__main__': 
    wust = Wust_Robot_Moveit_Control()
    path = "/home/yumenghui/test/arm_simple1.csv"
    csv_reader = csv.DictReader(open(path))
    Data = read_Skeleton_Quaternion(csv_reader)

    for i in range(len(Data[0])):
        q_ru = [Data[5][i],Data[6][i],Data[7][i],Data[8][i]]
        q_rf=[Data[9][i],Data[10][i],Data[11][i],Data[12][i]]
        shoulder_theta = inverse("right_arm",q_ru)
        #print (shoulder_theta)
        elbow_theta =0
        #wrist_theta = inverse("right_arm",q_rf)
        J_S = [0.0, 0.0, 0.0,0.0, 0.0, 0.0, 0.0, 
                0.0, 0.0, 0.0,0.0, 0.0,
                shoulder_theta[0], shoulder_theta[1], shoulder_theta[2], elbow_theta, 
                0, 0, 0, 0.0, 0.0, 0.0, 0.0]
        publish(J_S)
        """
        J_S = [0.0, 0.0, 0.0,shoulder_theta[0], shoulder_theta[1], shoulder_theta[2], elbow_theta, 
                wrist_theta[0], wrist_theta[1], wrist_theta[2],0.0, 0.0,
                JR_shoulder[1][0], JR_shoulder[1][1], JR_shoulder[1][2], JRE, 
                JR_wrist[1][0],JR_wrist[1][1],JR_wrist[1][2], 0.0, 0.0, 0.0, 0.0]
        """
        
