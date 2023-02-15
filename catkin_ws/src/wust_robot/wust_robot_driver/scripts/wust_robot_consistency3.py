#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys
import geometry_msgs.msg
from tf import transformations
from wust_robot_moveit_control_full import Wust_Robot_Moveit_Control, Object_parameters
import visualization_msgs.msg
import numpy as np
import moveit_msgs.msg
import moveit_commander
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import Pose, PoseStamped
from copy import deepcopy
from tf import transformations
import math
from std_msgs.msg import String
import commands
import os
import math
import matplotlib as mpl
import matplotlib.pyplot as plt

if __name__ == '__main__':

    wust = Wust_Robot_Moveit_Control()

    # joints = [-0.7589887631976004, 1.1816009121835886, -0.7778825300814339, 1.7246689402469935,
    #           0.674630790683862, -1.41401442293726, 1.9804284685614664]
    joints = [-1.0070961525168305, 0.46307186836913666, -0.23017442101115995, 2.044801786670023,
              0.36714645729335316, -1.458566036262238, 1.4444510882040236]

    # pos = [0.293, 0.15, 1.05]
    # rot = [-1.571, 1.571, -1.571]
    # wust.arm_pose_control("left_arm", pos, rot)

    # joints = [2.7764033131528554, -1.6262055960709954, -1.9463358455113213, -2.040482063429635,
    #           0.26606841789009233, 0.7723619831395908, 0.0005423436043680984]
    #
    wust.arm_joint_control("left_arm", joints)
    # wust.Plan_Cartesian_Path("left_arm", [0, 0.15, 0])

    # # 关节角
    # q1 = []
    # q2 = []
    # q3 = []
    # q4 = []
    # q5 = []
    # q6 = []
    # q7 = []
    #
    # x0 = []
    # y0 = []
    #
    # for i in range(5):
    #     q = wust.Plan_Cartesian_Path_Circle("left_arm", 0)
    #     for j in range(len(q)):
    #         # q1.append(q[j][0])
    #         # q2.append(q[j][1])
    #         # q3.append(q[j][2])
    #         # q4.append(q[j][3])
    #         # q5.append(q[j][4])
    #         # q6.append(q[j][5])
    #         # q7.append(q[j][6])
    #         q_m = []
    #         q1.append(q[j][0]-0.02*i)
    #         q2.append(q[j][1]-0.02*i)
    #         q3.append(q[j][2]+0.02*i)
    #         q4.append(q[j][3]-0.02*i)
    #         q5.append(q[j][4]-0.02*i)
    #         q6.append(q[j][5]-0.02*i)
    #         q7.append(q[j][6]-0.02*i)
    #         q_m.append(q1[j])
    #         q_m.append(q2[j])
    #         q_m.append(q3[j])
    #         q_m.append(q4[j])
    #         q_m.append(q5[j])
    #         q_m.append(q6[j])
    #         q_m.append(q7[j])
    #         y0.append(wust.arm_angle("left_arm", q_m))
    #
    # y = np.array(y0)
    # for i in range(len(y0)):
    #     x0.append(i+1)
    #
    # x = np.array(x0)
    # plt.plot(y)
    # plt.grid(True)
    # plt.axis('tight')
    # plt.xlabel('points')
    # plt.ylabel('arm_angle')
    # plt.title('arm_angle')
    # plt.ylim(-4, 4)
    # plt.show()
    # fileObject = open('Trac-IK_arm_angle.txt', 'w')
    # fileObject.write('arm_angle')
    # fileObject.write('\n')
    # for i in range(len(y0)):
    #     fileObject.write(str(y0[i]))
    #     fileObject.write(', ')
    #
    # fileObject.close()
    # for i in range(3):
    #     q = wust.Plan_Cartesian_Path_Circle2("left_arm", 0)
    #     for j in range(len(q)):
    #         print q[j][0]
    #         q1.append(q[j][0])
    #         q2.append(q[j][1])
    #         q3.append(q[j][2])
    #         q4.append(q[j][3])
    #         q5.append(q[j][4])
    #         q6.append(q[j][5])
    #         q7.append(q[j][6])
    #         # q1.append(q[j][0] - 0.02 * i)
    #         # q2.append(q[j][1] - 0.02 * i)
    #         # q3.append(q[j][2] + 0.02 * i)
    #         # q4.append(q[j][3] - 0.02 * i)
    #         # q5.append(q[j][4] - 0.02 * i)
    #         # q6.append(q[j][5] - 0.02 * i)
    #         # q7.append(q[j][6] - 0.02 * i)
    # q0 = []
    # for i in range(len(q1)):
    #     q0.append(i + 1)
    #
    # x = np.array(q0)
    #
    # plt.plot(q1, label='joint1')
    # plt.plot(q2, label='joint2')
    # plt.plot(q3, label='joint3')
    # plt.plot(q4, label='joint4')
    # plt.plot(q5, label='joint5')
    # plt.plot(q6, label='joint6')
    # plt.plot(q7, label='joint7')
    #
    # plt.title("Joint Angle Curve of Anthropomorphic Manipulator Repeated Drawing Circle(ikfast)")
    #
    # plt.legend(loc='upper right',ncol=3)
    # plt.grid(True)
    # plt.axis('tight')
    # plt.xlabel('points')
    # plt.ylabel('joints(rad)')
    # plt.ylim(-4, 4)
    # plt.show()

    # fileObject = open('Trac_ik_joints.txt', 'w')
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

    # 臂形角

    x0 = []
    y0 = []

    for i in range(2):
        y1 = wust.Plan_Cartesian_Path_Circle("left_arm", 1)
        for j in range(len(y1)):
            y0.append(y1[j])
    for i in range(3):
        y1 = wust.Plan_Cartesian_Path_Circle2("left_arm", 1)
        for j in range(len(y1)):
            y0.append(y1[j])

    y = np.array(y0)

    for i in range(len(y)):
        x0.append(i+1)

    x = np.array(x0)


    print y
    plt.plot(y)
    plt.grid(True)
    plt.axis('tight')
    plt.xlabel('points')
    plt.ylabel('arm_angle')
    plt.title('arm_angle')
    plt.ylim(-4, 4)
    plt.show()
    fileObject = open('Trac-IK_arm_angle.txt', 'w')
    fileObject.write('arm_angle')
    fileObject.write('\n')
    for i in range(len(y0)):
        fileObject.write(str(y0[i]))
        fileObject.write(', ')

    fileObject.close()





    #wust = Wust_Robot_Moveit_Control()

    # joint = [1.140624068766987, -0.8748328661695439, 1.8315749077618269, -1.9407375568590506,
    #          0.20792902059600826, 1.6116328183868582, 0.6358516729085432]
    # arm_angle = wust.arm_angle("left_arm", joint)
    # joint = [0.2003907411596762, 0.604972346514992, -1.6360275371292667, 1.9404937877720707,
    #          0.5529451075387826, -0.7931420239081193, -0.8789243628927844]
    # # joint = [1.1221012789541036, -0.8191751726582512, 1.7929376782175561, -1.940609457252452,
    # #          0.2459484491416406, 1.595634676615595, 0.7018777900290394]
    # wust.arm_joint_control("left_arm", joint)

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

    #臂形角
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



# class MoveItCircleDemo:
#     def __init__(self):
#         # 初始化move_group的API
#         moveit_commander.roscpp_initialize(sys.argv)
#
#         # 初始化ROS节点
#         rospy.init_node('moveit_clrcle_demo', anonymous=True)
#
#         # 初始化需要使用move group控制的机械臂中的arm group
#         arm = MoveGroupCommander('left_arm')
#
#         # 设置目标位置所使用的参考坐标系
#         reference_frame = 'base_link'
#         arm.set_pose_reference_frame('base_link')
#
#         joints = [-0.7589887631976004, 1.1816009121835886, -0.7778825300814339, 1.7246689402469935,
#                   0.674630790683862, -1.41401442293726, 1.9804284685614664]
#
#
#         target_pose = PoseStamped()
#         target_pose.header.frame_id = reference_frame
#         target_pose.pose.position.x = 0.293
#         target_pose.pose.position.y = 0
#         target_pose.pose.position.z = 1.05
#         rot = [-1.571, 1.571, -1.571]
#         quaternion = transformations.quaternion_from_euler(rot[0], rot[1], rot[2])
#         target_pose.pose.orientation.x = quaternion[0]
#         target_pose.pose.orientation.y = quaternion[1]
#         target_pose.pose.orientation.z = quaternion[2]
#         target_pose.pose.orientation.w = quaternion[3]
#
#         # 设置机械臂终端运动的目标位姿
#         arm.set_pose_target(target_pose)
#         plan = arm.plan()
#         arm.execute(plan)

        # # 初始化路点列表
        # waypoints = []
        #
        # # 将圆弧上的路径点加入列表
        # waypoints.append(target_pose.pose)
        #
        # centerA = target_pose.pose.position.y
        # centerB = target_pose.pose.position.z
        # radius = 0.15
        #
        # target_pose2 = PoseStamped()
        # target_pose2 = target_pose
        # target_pose2.pose.position.y = 0.15
        # waypoints.append(target_pose2.pose)
        #
        # fraction = 0.0  # 路径规划覆盖率
        # maxtries = 100  # 最大尝试规划次数
        # attempts = 0  # 已经尝试规划次数
        #
        # # 设置机器臂当前的状态作为运动初始状态
        # arm.set_start_state_to_current_state()
        #
        # # 尝试规划一条笛卡尔空间下的路径，依次通过所有路点，完成圆弧轨迹
        # while fraction < 1.0 and attempts < maxtries:
        #     (plan, fraction) = arm.compute_cartesian_path(
        #         waypoints,  # waypoint poses，路点列表
        #         0.01,  # eef_step，终端步进值
        #         0.0,  # jump_threshold，跳跃阈值
        #         True)  # avoid_collisions，避障规划
        #
        #     # 尝试次数累加
        #     attempts += 1
        #
        #     # 打印运动规划进程
        #     if attempts % 10 == 0:
        #         rospy.loginfo("Still trying after " + str(attempts) + " attempts...")
        #
        # # 如果路径规划成功（覆盖率100%）,则开始控制机械臂运动
        # if fraction == 1.0:
        #     rospy.loginfo("Path computed successfully. Moving the arm.")
        #     arm.execute(plan)
        #     rospy.loginfo("Path execution complete.")
        #
        # rospy.sleep(10)
        #
        # for th in numpy.arange(0, 6.30, 0.01):
        #     target_pose.pose.position.y = centerA + radius * math.cos(th)
        #     target_pose.pose.position.z = centerB + radius * math.sin(th)
        #     wpose = deepcopy(target_pose.pose)
        #     waypoints.append(deepcopy(wpose))
        #
        #     # print('%f, %f' % (Y, Z))
        #
        # fraction = 0.0  # 路径规划覆盖率
        # maxtries = 100  # 最大尝试规划次数
        # attempts = 0  # 已经尝试规划次数
        #
        # # 设置机器臂当前的状态作为运动初始状态
        # arm.set_start_state_to_current_state()
        #
        # # 尝试规划一条笛卡尔空间下的路径，依次通过所有路点，完成圆弧轨迹
        # while fraction < 1.0 and attempts < maxtries:
        #     (plan, fraction) = arm.compute_cartesian_path(
        #         waypoints,  # waypoint poses，路点列表
        #         0.01,  # eef_step，终端步进值
        #         0.0,  # jump_threshold，跳跃阈值
        #         True)  # avoid_collisions，避障规划
        #
        #     # 尝试次数累加
        #     attempts += 1
        #
        #     # 打印运动规划进程
        #     if attempts % 10 == 0:
        #         rospy.loginfo("Still trying after " + str(attempts) + " attempts...")
        #
        # # 如果路径规划成功（覆盖率100%）,则开始控制机械臂运动
        # if fraction == 1.0:
        #     rospy.loginfo("Path computed successfully. Moving the arm.")
        #     arm.execute(plan)
        #     rospy.loginfo("Path execution complete.")
        # # 如果路径规划失败，则打印失败信息
        # else:
        #     rospy.loginfo("Path planning failed with only " + str(fraction) + " success after " + str(
        #         maxtries) + " attempts.")
        #
        # rospy.sleep(1)
        #
        # # 关闭并退出moveit
        # moveit_commander.roscpp_shutdown()
        # moveit_commander.os._exit(0)
