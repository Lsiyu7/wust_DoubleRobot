#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
import geometry_msgs.msg
from tf import transformations
from wust_robot_moveit_control_full import Wust_Robot_Moveit_Control, Object_parameters
import visualization_msgs.msg
import numpy as np
import moveit_msgs.msg
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import commands
import os
import csv
import time
   
def read():
    csv_reader = csv.DictReader(open("/home/yumenghui/test/test1.csv"))
    Time=[]
    JR1=[]
    JR2=[]
    JR3=[]
    JR4=[]
    JR5=[]
    JR6=[]
    JR7=[]
    for i in csv_reader:
        time = float(i['Time(Second)'])
        jr1 =(180 - (float(i['Joint Euler Angle(Degree):RightShoulder-X'])) * (math.pi/180))
        jr2 =((float(i['Joint Euler Angle(Degree):RightShoulder-Y'])) * (math.pi/180))
        jr3 =((float(i['Joint Euler Angle(Degree):RightShoulder-Z']) + 180) * (math.pi/180))
        jr4 = (-(180 - float(i['Joint Vector Angle(Degree):RightElbow'])) * (math.pi/180))
        jr5 =(float(i['Joint Euler Angle(Degree):RightElbow-X']) * (math.pi/180))
        jr6 = (float(i['Joint Euler Angle(Degree):RightElbow-Y']) * (math.pi/180))
        jr7 =(float(i['Joint Euler Angle(Degree):RightElbow-Z']) * (math.pi/180))
        Time.append(time)
        JR1.append(jr1)
        JR2.append(jr2)
        JR3.append(jr3)
        JR4.append(jr4)
        JR5.append(jr5)
        JR6.append(jr6)
        JR7.append(jr7)
    return (Time,JR1,JR2,JR3,JR4,JR5,JR6,JR7)

def smooth(a,WSZ):
	out0 = np.convolve(a,np.ones(WSZ,dtype=int),'valid')/WSZ    
	r = np.arange(1,WSZ-1,2)
	start = np.cumsum(a[:WSZ-1])[::2]/r
	stop = (np.cumsum(a[:-WSZ:-1])[::2]/r)[::-1]
	return np.concatenate((  start , out0, stop  ))

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
    readcopy = read()
    JR1_S = smooth(readcopy[1],29)
    JR2_S = smooth(readcopy[2],29)
    JR3_S = smooth(readcopy[3],29)
    JR4_S = smooth(readcopy[4],29)
    JR5_S = smooth(readcopy[5],29)
    JR6_S = smooth(readcopy[6],29)
    JR7_S = smooth(readcopy[7],29)
    for i in range(len(readcopy[0])):
        j_r1 = JR1_S[i]
        j_r2 = JR2_S[i]
        j_r3 = JR3_S[i]
        j_r4 = JR4_S[i]
        j_r5 = JR5_S[i]
        j_r6 = JR6_S[i]
        j_r7 = JR7_S[i]
        #J_S = [j_r4, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0, 0, 0.0,
        #                0, 0, 0, 0.0, 0.0, 0.0, 0.0]
        J_S = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, j_r1, j_r2, j_r3, j_r4,
                        j_r5, j_r6, j_r7, 0.0, 0.0, 0.0, 0.0]
        publish(J_S)
