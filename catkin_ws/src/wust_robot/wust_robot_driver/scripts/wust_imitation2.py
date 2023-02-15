#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
import geometry_msgs.msg
from tf import transformations
from wust_robot_moveit_control_full import Wust_Robot_Moveit_Control, Object_parameters
import visualization_msgs.msg
import numpy
import numpy as np
import moveit_msgs.msg
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import commands
import os
import csv
import time
   
def wust_read():
    csv_reader = csv.DictReader(open("/home/yumenghui/test/arm_complex1.csv"))
    Time=[]
    p1=[]
    p2=[]
    p3=[]
    p4=[]
    p5=[]
    p6=[]
    p7=[]
    p8=[]
    p9=[]
    p10=[]
    p11=[]
    p12=[]
    for i in csv_reader:
        t = float(i['Time(Second)'])
        RightShoulder_x=float(i['Displacement(m):RightShoulder-X'])
        RightShoulder_y=float(i['Displacement(m):RightShoulder-Y'])
        RightShoulder_z=float(i['Displacement(m):RightShoulder-Z'])
        RightUpperArm_x=float(i['Displacement(m):RightUpperArm-X'])
        RightUpperArm_y=float(i['Displacement(m):RightUpperArm-Y'])
        RightUpperArm_z=float(i['Displacement(m):RightUpperArm-Z'])
        RightForeArm_x=float(i['Displacement(m):RightForeArm-X'])
        RightForeArm_y=float(i['Displacement(m):RightForeArm-Y'])
        RightForeArm_z=float(i['Displacement(m):RightForeArm-Z'])
        RightHand_x=float(i['Displacement(m):RightHand-X'])
        RightHand_y=float(i['Displacement(m):RightHand-Y'])
        RightHand_z=float(i['Displacement(m):RightHand-Z'])
        Time.append(t)
        p1.append(RightUpperArm_x)
        p2.append(RightUpperArm_y)
        p3.append(RightUpperArm_z)
        p4.append(RightForeArm_x)
        p5.append(RightForeArm_y)
        p6.append(RightForeArm_z)
        p7.append(RightHand_x)
        p8.append(RightHand_y)
        p9.append(RightHand_z)
        p10.append(RightShoulder_x)
        p11.append(RightShoulder_y)
        p12.append(RightShoulder_z)    
    return (Time,p1,p2,p3,p4,p5,p6,p7,p8,p9,p10,p11,p12)
    
def cos_angle(List1,List2):
    V1=numpy.array([List1[0],List1[1],List1[2]])
    V2=numpy.array([List2[0],List2[1],List2[2]])
    angle=np.arccos(V1.dot(V2))
    return angle

#def V_mul(V1,V2):
#	x = V1[1]*V2[2] - V1[2]*V2[1]
#	y = V1[2]*V2[0] - V1[0]*V2[2]
#	z = V1[0]*V2[1] - V1[1]*V2[0]
#	V_n = np.array([x,y,z])
#	return V_n
	    
def imitation_inverse(P):
    V1 =P[3]/np.linalg.norm(P[3])    #单位化
    V2 =P[0]-P[3]
    V2 =V2/np.linalg.norm(V2)
    V3 = P[1] - P[0]
    V3 = V3/np.linalg.norm(V3)
    V4 = P[2] - P[1]
    V4 = V4/np.linalg.norm(V4)
    theta2 = cos_angle(V2,V3)
    theta4 = cos_angle(V3,V4)
    n1 = np.cross(V1,V2)
    n2 = np.cross(V2,V3)
    n3 = np.cross(V3,V4)
    theta1 = cos_angle(n1,n2)
    theta3 = cos_angle(n1,n3)
    return (theta1,theta2,theta3,theta4)
		
def publish(JS):
    pub = rospy.Publisher('joint_states', JointState, queue_size=30)
    rate = rospy.Rate(100)
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
    readcopy = wust_read()
    for i in range(len(readcopy[0])):
            P=numpy.array([[readcopy[1][i],readcopy[2][i],readcopy[3][i]],
                            [readcopy[4][i],readcopy[5][i],readcopy[6][i]],
                            [readcopy[7][i],readcopy[8][i],readcopy[9][i]],
                            [readcopy[10][i],readcopy[11][i],readcopy[12][i]]])
            Theta = imitation_inverse(P)
            print(Theta)
            J_S = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                    Theta[0]-math.pi, Theta[1], Theta[2],Theta[3],0.0,0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            publish(J_S)
