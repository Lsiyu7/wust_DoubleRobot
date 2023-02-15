#!/usr/bin/env python
#coding:utf-8

import rospy

# 导入最主要的Python for ROS库
from geometry_msgs.msg import Twist
from math import pi

# 导入geometry_msgs包中的Twist消息类型
class MOVE():
    def __init__(self):
        # 节点名称
     #rospy.init_node('out_and_back', anonymous=False)
        # 当终端按下Ctrl＋C之后可以终止节点
        rospy.on_shutdown(self.shutdown)
        # 定义在/cmd_vel Topic中发布Twist消息，控制机器人速度
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rate = 50
        # 设置更新频率为50HZ
        r = rospy.Rate(rate)
        # 线速度
        linear_speed = 0.4
        angular_speed = 1.0
        self.linear_speed = linear_speed
        self.angular_speed = angular_speed
        self.rate = rate
        self.r = r
    def pos_move(self, speed, goal_distance):
        rate = self.rate
        linear_speed = self.linear_speed
        r = self.r
        # 到达目标的时间
        linear_duration = goal_distance / linear_speed
        for i in range(1):
            # 初始化移动命名
            move_cmd = Twist()
            #  # 设置前进速度
            move_cmd.linear.x = speed[0]
            move_cmd.linear.y = speed[1]
            move_cmd.linear.z = speed[2]
            # 机器人向前运动，延时一定时间
            ticks = int(linear_duration * rate)
            for t in range(ticks):
                self.cmd_vel.publish(move_cmd)
                r.sleep()
        self.cmd_vel.publish(Twist())

    def rot_move(self, speed, goal_angle):
        rate = self.rate
        angular_speed = self.angular_speed
        r = self.r
        # 到达目标的时间
        angular_duration = goal_angle / angular_speed
        for i in range(1):
            # 初始化移动命名
            move_cmd = Twist()
            #  # 设置前进速度
            move_cmd.angular.x = speed[0]
            move_cmd.angular.y = speed[1]
            move_cmd.angular.z = speed[2]
            # 机器人向前运动，延时一定时间
            ticks = int(angular_duration * rate)
            for t in range(ticks):
                self.cmd_vel.publish(move_cmd)
                r.sleep()
        self.cmd_vel.publish(Twist())
    def shutdown(self):
        # Always stop the robot when shutting down the node.
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)

# class MOVE():
#     def __init__(self):
#     # 节点名称
#      #rospy.init_node('out_and_back', anonymous=False)
#     # 当终端按下Ctrl＋C之后可以终止节点
#       rospy.on_shutdown(self.shutdown)
#     # 定义在/cmd_vel Topic中发布Twist消息，控制机器人速度
#       self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
#       rate = 50
#      # 设置更新频率为50HZ
#       r = rospy.Rate(rate)
#     # 线速度
#       linear_speed = 0.4
#
#    def pos_move(self, linear):
#         # 目标距离
#      goal_distance1 = 1.5
#
#      # 目标距离
#      goal_distance2 = 2.0
#         # 到达目标的时间
#      linear_duration1 = goal_distance1 / linear_speed
#
#      linear_duration2 = goal_distance2 / linear_speed
#      #    # 角速度 1.0rad/s
#      angular_speed = 1.0
#         # 转角为Pi(180 degrees)
#      print pi
#      goal_angle = 0.575*pi
# 	# How long should it take to rotate?
#      angular_duration = goal_angle / angular_speed
#
# 	# Loop through the two legs of the trip
#      for i in range(1):
#          # 初始化移动命名
#        move_cmd = Twist()
#        #  # 设置前进速度
#        move_cmd.linear.y = linear_speed
#             # 机器人向前运动，延时一定时间
#        ticks = int(linear_duration1 * rate)
#        for t in range(ticks):
#           self.cmd_vel.publish(move_cmd)
#           r.sleep()

	# # 发送一个空的Twist消息是机器人停止
    #    move_cmd = Twist()
    #    self.cmd_vel.publish(move_cmd)
    #    rospy.sleep(1)
    #    move_cmd.angular.z = -angular_speed
    #         #机器人开始旋转，延时一定时间使机器人转180度
    #    ticks = int(angular_duration* rate)
    #    print ticks
    #    for t in range(ticks):
    #         self.cmd_vel.publish(move_cmd)
    #         r.sleep()

       # move_cmd = Twist()
       #  #  # 设置前进速度
       # move_cmd.linear.x = linear_speed
       #  # 机器人向前运动，延时一定时间
       # ticks = int(linear_duration2 * rate)
       # for t in range(ticks):
       #      self.cmd_vel.publish(move_cmd)
       #      r.sleep()

            # 停下来

        # 循环两次之后停止
        #self.cmd_vel.publish(Twist())

   # 定义 shutdown(self)可以手动停止机器人



