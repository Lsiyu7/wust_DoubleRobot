#!/usr/bin/env python

import rospy
import actionlib
from mr_msgs.msg import GripperCommand
from control_msgs.msg import GripperCommandAction, FollowJointTrajectoryAction
class wust_robot_Gripper_Control():
    def __init__(self):
        self.gripper_pub = rospy.Publisher('/wust_robot/left_gripper/gripper_cmd', GripperCommand, queue_size=5)
        self.gripper_cmd = GripperCommand()
        # self.left_gripper_server = actionlib.SimpleActionServer(
        #     '/wust_robot/left_gripper_controller/gripper_cmd',
        #     GripperCommandAction,
        #     execute_cb=self.wust_robot_Left_Gripper_Command,
        #     auto_start=False)
        self.left_gripper_server = actionlib.SimpleActionServer(
            '/wust_robot/left_gripper_controller/follow_joint_trajectory',
            FollowJointTrajectoryAction,
            execute_cb=self.wust_robot_Left_Gripper_Command,
            auto_start=False)
        self.left_gripper_server.start()

    def wust_robot_Left_Gripper_Command(self, goal):
        # position = goal.command.position
        # print " ====", goal.trajectory, " ==== "
        goal.trajectory.points.reverse()
        position =goal.trajectory.points[0].positions
        self.gripper_cmd.name = "Gripper1"
        self.gripper_cmd.positionL = 400*((0.36-position[0])/0.72)
        self.gripper_cmd.positionR = 400*((0.36-position[0])/0.72)
        # print " ====", self.gripper_cmd, " ==== "
        self.gripper_pub.publish(self.gripper_cmd)
        rospy.sleep(2)
        self.left_gripper_server.set_succeeded()

if __name__ == '__main__':

    rospy.init_node('wust_robot_Left_Gripper', anonymous=True)
    wust_robot_Gripper_Control()
    rospy.spin()
