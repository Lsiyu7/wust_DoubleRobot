#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
import geometry_msgs.msg
import moveit_msgs.msg
from tf import transformations
from wust_robot_moveit_control_full import Wust_Robot_Moveit_Control, Object_parameters
from move import MOVE
import numpy as np
import math
import matplotlib as mpl
import matplotlib.pyplot as plt

# 移动抓取
if __name__ == '__main__':
    wust = Wust_Robot_Moveit_Control()

    # pos = [0.386, 0.350, 0.820]
    # rot = [-1.57, 1.57, -1.57]
    # wust.arm_pose_control("left_arm", pos, rot)
    # joints = [3.0333413373989626, -0.806849362358476, -2.4508069396511996, 0.2468500363708622,
    #           -2.793787280682091, 1.0783991106562099, -0.37181151214557573]
    #
    # wust.arm_joint_control("left_arm", joints)
    # rospy.sleep(10)
    #
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, -0.4, 0], 45)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, 0, 0.03], 7)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, 0.4, 0], 45)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, 0, 0.03], 7)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, -0.4, 0], 45)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, 0, 0.03], 7)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, 0.4, 0], 45)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, 0, 0.03], 7)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, -0.4, 0], 45)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, 0, 0.03], 7)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, 0.4, 0], 45)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, 0, 0.03], 7)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, -0.4, 0], 45)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, 0, 0.03], 7)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, 0.4, 0], 45)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, 0, 0.03], 7)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, -0.4, 0], 45)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, 0, 0.03], 7)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, 0.4, 0], 45)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, 0, 0.03], 7)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, -0.4, 0], 45)

    # joints = [-2.771765292209274, -0.6247561654915313, -0.8324859413850061, -0.3629988617883714,
    #           2.0044848842427903, 1.1963804000415543, -0.6512191566433152]
    # wust.arm_joint_control("left_arm", joints)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, 0, 0.3], 35)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, -0.04, 0], 7)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, 0, -0.3], 35)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, -0.04, 0], 7)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, 0, 0.3], 35)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, -0.04, 0], 7)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, 0, -0.3], 35)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, -0.04, 0], 7)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, 0, 0.3], 35)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, -0.04, 0], 7)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, 0, -0.3], 35)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, -0.04, 0], 7)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, 0, 0.3], 35)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, -0.04, 0], 7)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, 0, -0.3], 35)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, -0.04, 0], 7)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, 0, 0.3], 35)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, -0.04, 0], 7)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, 0, -0.3], 35)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, -0.04, 0], 7)

    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, -0.4, 0], 45)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, 0, 0.03], 7)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, 0.4, 0], 45)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, 0, 0.03], 7)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, -0.4, 0], 45)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, 0, 0.03], 7)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, 0.4, 0], 45)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, 0, 0.03], 7)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, -0.4, 0], 45)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, 0, 0.03], 7)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, 0.4, 0], 45)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, 0, 0.03], 7)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, -0.4, 0], 45)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, 0, 0.03], 7)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, 0.4, 0], 45)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, 0, 0.03], 7)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, -0.4, 0], 45)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, 0, 0.03], 7)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, 0.4, 0], 45)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, 0, 0.03], 7)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, -0.4, 0], 45)

    # joints = [2.9953404367914604, -0.5983987591795765, -2.9536319043607975, 0.26815438820561927,
    #           -1.5262224320343702, 1.3151176505417803, -0.7740736547259993]
    # wust.arm_joint_control("left_arm", joints)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, 0, 0.4], 45)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, -0.04, 0], 7)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, 0, -0.4], 45)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, -0.04, 0], 7)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, 0, 0.4], 45)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, -0.04, 0], 7)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, 0, -0.4], 45)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, -0.04, 0], 7)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, 0, 0.4], 45)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, -0.04, 0], 7)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, 0, -0.4], 45)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, -0.04, 0], 7)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, 0, 0.4], 45)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, -0.04, 0], 7)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, 0, -0.4], 45)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, -0.04, 0], 7)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, 0, 0.4], 45)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, -0.04, 0], 7)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, 0, -0.4], 45)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, -0.04, 0], 7)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, 0, 0.4], 45)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, -0.04, 0], 7)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, 0, -0.4], 45)

    # joint_left = [0.006521063539536489, 0.5346040940527549, 2.5465647316325684, -0.5880149782496275,
    #               -0.3461919803592281, -1.6443646271650074, -0.6181066849914771]
    # joint_left = [0.0012952749055768298, 0.5896052447078024, 2.523691313494691, -0.3477433372213094,
    # -0.31635701416070033, -1.8316952058492857, -0.6691991389773809]

    # joint = [1.683384898006957, -1.592855985746594, 2.6140045981721256, -1.731929435631908,
    #          2.7728080968175024, -1.30935810893412, -2.11642509426235]

    # joint = [1.3396496418560027, -1.2763198660666113, 2.139505258957186, -1.9415267869152149,
    #          3.0472312158977997, -1.6428274737070985, -1.560646623757475]

    # for i in range(10):
    #     wust.Plan_Cartesian_Path_Rot("left_arm", [0.3, 0.507, 1.017], [-0.381, 1.555, -0.475])
    #     wust.Plan_Cartesian_Path_Rot("left_arm", [0.3, 0.507, 1.017], [-1.556, 0.111, -1.663])
        # wust.Plan_Cartesian_Path_Rot("left_arm", [0.3, 0.507, 1.319], [-1.556, 0.111, -1.663])
        # wust.Plan_Cartesian_Path_Rot("left_arm", [0.3, -0.016, 1.319], [-1.855, 0.979, -2.012])
        # wust.Plan_Cartesian_Path_Rot("left_arm", [0.3, -0.016, 1.017], [-2.511, -0.382, -1.711])
        # wust.Plan_Cartesian_Path_Rot("left_arm", [0.35, -0.025, 1.452], [-1.294, -1.519, -1.939])
        # wust.Plan_Cartesian_Path_Rot("left_arm", [0.35, -0.025, 0.9], [-1.557, 0.003, -1.663])

    # wust.Plan_Cartesian_Path_Rot("left_arm", [0.028, 0.203, 0.944], [2.942, 1.502, 2.244])
    # for i in range(100):
    #     wust.Plan_Cartesian_Path_Inter("left_arm", [0, 0.10, 0.70], 75)
    #     wust.Plan_Cartesian_Path_Inter("left_arm", [0, -0.10, -0.70], 75)

    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, -0.03, 0.0],5)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, 0.10, 0.70],75)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, -0.03, 0.0],5)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, -0.10, -0.70],75)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, -0.03, 0.0],5)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, 0.10, 0.70],75)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, -0.03, 0.0],5)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, -0.10, -0.70],75)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, -0.03, 0.0],5)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, 0.10, 0.70],75)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, -0.03, 0.0],5)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, -0.10, -0.70],75)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, -0.03, 0.0],5)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, 0.10, 0.70],75)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, -0.03, 0.0],5)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, -0.10, -0.70],75)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, -0.03, 0.0],5)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, 0.10, 0.70],75)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, -0.03, 0.0],5)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, -0.10, -0.70],75)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, -0.03, 0.0],5)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, 0.10, 0.70],75)

    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, 0.10, 0.40],45)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, -0.03, 0.0],5)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, -0.10, -0.40],45)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, -0.03, 0.0],5)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, 0.10, 0.40],45)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, -0.03, 0.0],5)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, -0.10, -0.40],45)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, -0.03, 0.0],5)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, 0.10, 0.40],45)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, -0.03, 0.0],5)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, -0.10, -0.40],45)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, -0.03, 0.0],5)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, 0.10, 0.40],45)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, -0.03, 0.0],5)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, -0.10, -0.40],45)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, -0.03, 0.0],5)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, 0.10, 0.40],45)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, -0.03, 0.0],5)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, -0.10, -0.40],45)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, -0.03, 0.0],5)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, 0.10, 0.40],45)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, -0.03, 0.0],5)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, -0.10, -0.40],45)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, -0.03, 0.0],5)
    # wust.Plan_Cartesian_Path_Inter("left_arm", [0, 0.10, 0.40],45)

    # pos_left = [0.300, 0.120, 0.940]
    # rot_left = [-2.711, 1.502, 2.247]
    # pos_right = [0.300, -0.120, 0.940]
    # rot_right = [-0.843, -1.474, -1.006]
    # wust.dual_arm_pose_control(pos_left, rot_left, pos_right, rot_right)


    #wust.arm_pose_control("left_arm", [0.3, 0.270, 1.003], [-1.57, -0.975, -1.57])

    # joint = [-0.7677920038312553, 0.1216774096449057, -0.1794680138210823, 1.9392615240336333,
    #          0.596854879755905, -1.172877778402431, -1.5541123170677527]
    joint = [1.140624068766987, -0.8748328661695439, 1.8315749077618269, -1.9407375568590506,
             0.20792902059600826, 1.6116328183868582, 0.6358516729085432]
    # joint = [1.1221012789541036, -0.8191751726582512, 1.7929376782175561, -1.940609457252452,
    #          0.2459484491416406, 1.595634676615595, 0.7018777900290394]
    wust.arm_joint_control("left_arm", joint)

    # 关节角臂形角
    q1 = []
    q2 = []
    q3 = []
    q4 = []
    q5 = []
    q6 = []
    q7 = []
    for i in range(5):
    # # #     wust.Plan_Cartesian_Path_Rot("left_arm", [0.30, 0.270, 1.003], [-1.572, -0.975, -1.570])
    # # #     #wust.Plan_Cartesian_Path_Rot("left_arm", [0.30, 0.230, 1.303], [-1.572, -0.975, -1.570])
        #wust.Plan_Cartesian_Path_Rot("left_arm", [0.30, 0, 1.303], [-0.517, -1.380, 2.390], 0)
        q = wust.Plan_Cartesian_Path_Rot("left_arm", [0.30, 0, 1.303], [-0.517, -1.380, 2.790], 0)
        wust.Plan_Cartesian_Path_Rot("left_arm", [0.30, 0, 1.003], [1.573, -1.435, 1.581], 0)
        wust.Plan_Cartesian_Path_Rot("left_arm", [0.30, 0.27, 1.003], [-1.572, -0.975, -1.570], 0)
        for j in range(len(q)):
            q1.append(q[j][0])
            q2.append(q[j][1])
            q3.append(q[j][2])
            q4.append(q[j][3])
            q5.append(q[j][4])
            q6.append(q[j][5])
            q7.append(q[j][6])
    q0 = []
    for i in range(len(q1)):
        q0.append(i+1)

    x = np.array(q0)

    plt.plot(q1)
    plt.plot(q2)
    plt.plot(q3)
    plt.plot(q4)
    plt.plot(q5)
    plt.plot(q6)
    plt.plot(q7)
    plt.grid(True)
    plt.axis('tight')
    plt.xlabel('points')
    plt.ylabel('joints')
    # plt.q1label('joint1')
    # plt.q2label('joint2')
    # plt.q3label('joint3')
    # plt.q4label('joint4')
    # plt.q5label('joint5')
    # plt.q6label('joint6')
    # plt.q7label('joint7')
    plt.show()

    # 臂形角
    x0 = []
    y0 = []
    for i in range(1):
         wust.Plan_Cartesian_Path_Rot("left_arm", [0.30, 0.270, 1.003], [-1.572, -0.975, -1.570])
         wust.Plan_Cartesian_Path_Rot("left_arm", [0.30, 0.230, 1.303], [-1.572, -0.975, -1.570])
         y1 = wust.Plan_Cartesian_Path_Rot("left_arm", [0.30, 0, 1.303], [-0.517, -1.380, 2.390], 1)
         wust.Plan_Cartesian_Path_Rot("left_arm", [0.30, 0, 1.003], [1.573, -1.435, 1.581], 0)
         wust.Plan_Cartesian_Path_Rot("left_arm", [0.30, 0.270, 1.003], [-1.572, -0.975, -1.570], 0)
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
    plt.show()

