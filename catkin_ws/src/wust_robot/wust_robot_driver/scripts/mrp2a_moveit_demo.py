#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from tf import transformations

from std_msgs.msg import String

class mrp2a_moveit_control:
  def __init__(self):
    print "==== Starting mrp2a_moveit ===="
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    self.moveit_commander = moveit_commander
    self.robot = robot
    self.scence = scene


  def gripper_joint_control(self, group_name, control):
    group = self.moveit_commander.MoveGroupCommander(group_name)
    group.set_planner_id("RRTkConfigDefault")
    group.set_pose_reference_frame("/base_link")

    ### Then, we will get the current set of joint values for the group
    group_variable_values = group.get_current_joint_values()

    # print "======== Current Joint values: ", group_variable_values

    ### modify joints according to the control type
    if control == "open":
      group_variable_values[0] = 0.36
      group_variable_values[1] = -0.36
      group.set_joint_value_target(group_variable_values)
    elif control == "close":
      group_variable_values[0] = -0.13
      group_variable_values[1] = 0.13
      group.set_joint_value_target(group_variable_values)

    group.plan()
    group.go(wait=True)

    # print "==== Waiting while RVIZ displays plan2 ===="
    # rospy.sleep(0.5)
    # moveit_commander.roscpp_shutdown()
    print "==== STOPPING ===="

  def dual_gripper_joint_control(self, control_left, control_right):
    group = self.moveit_commander.MoveGroupCommander("dual_gripper")
    group.set_planner_id("RRTkConfigDefault")
    group.set_pose_reference_frame("/base_link")

    ### Then, we will get the current set of joint values for the group
    group_variable_values = group.get_current_joint_values()

    ### modify joints according to the control type
    if control_left == "open":
      group_variable_values[0] = 0.36
      group_variable_values[1] = -0.36
    elif control_left == "close":
      group_variable_values[0] = -0.08
      group_variable_values[1] = 0.08
    if control_right == "open":
      group_variable_values[2] = 0.36
      group_variable_values[3] = -0.36
    elif control_right == "close":
      group_variable_values[2] = -0.22
      group_variable_values[3] = 0.22
    group.set_joint_value_target(group_variable_values)
    group.plan()
    group.go(wait=True)

    # print "==== Waiting while RVIZ displays plan2 ===="
    # rospy.sleep(0.5)
    print "==== STOPPING ===="

  def head_joint_control(self):
    group = self.moveit_commander.MoveGroupCommander("head")
    group.set_planner_id("RRTkConfigDefault")
    group.set_pose_reference_frame("/base_link")
    ### Then, we will get the current set of joint values for the group
    group_variable_values = group.get_current_joint_values()
    # print "======== Current Joint values: ", group_variable_values
    ### modify joints according to the control type
    group_variable_values[1] = 0.72
    group.set_joint_value_target(group_variable_values)

    group.plan()
    group.go(wait=True)
    # rospy.sleep(0.5)
    print "==== STOPPING ===="

  def arm_joint_control(self, group_name, joint):
    group = self.moveit_commander.MoveGroupCommander(group_name)
    group.set_planner_id("RRTConnectkConfigDefault")
    group.set_pose_reference_frame("/base_link")

    ### Then, we will get the current set of joint values for the group
    group_variable_values = group.get_current_joint_values()
    ### modify joints according to the control type
    group_variable_values[0] = joint[0]
    group_variable_values[1] = joint[1]
    group_variable_values[2] = joint[2]
    group_variable_values[3] = joint[3]
    group_variable_values[4] = joint[4]
    group_variable_values[5] = joint[5]
    group_variable_values[6] = joint[6]
    group.set_joint_value_target(group_variable_values)

    group.plan()
    group.go(wait=True)
    # rospy.sleep(1)
    print "============ STOPPING"

  def dual_arm_joint_control(self, joint_left, joint_right):
    group = self.moveit_commander.MoveGroupCommander("dual_arm")
    group.set_planner_id("RRTkConfigDefault")
    group.set_pose_reference_frame("/base_link")
    group_variable_values = group.get_current_joint_values()
    # print "======== Current Joint values: ", group_variable_values, " ===="

    ### modify joints according to the control type
    group_variable_values[0] = joint_left[0]
    group_variable_values[1] = joint_left[1]
    group_variable_values[2] = joint_left[2]
    group_variable_values[3] = joint_left[3]
    group_variable_values[4] = joint_left[4]
    group_variable_values[5] = joint_left[5]
    group_variable_values[6] = joint_left[6]
    group_variable_values[7] = joint_right[0]
    group_variable_values[8] = joint_right[1]
    group_variable_values[9] = joint_right[2]
    group_variable_values[10] = joint_right[3]
    group_variable_values[11] = joint_right[4]
    group_variable_values[12] = joint_right[5]
    group_variable_values[13] = joint_right[6]

    group.set_joint_value_target(group_variable_values)
    # group.plan()
    group.go(wait=True)
    print "============ STOPPING"

  def arm_pose_control(self, group_name, pose):
    display_trajectory_publisher = rospy.Publisher(
      '/move_group/display_planned_path',
      moveit_msgs.msg.DisplayTrajectory)

    group = self.moveit_commander.MoveGroupCommander(group_name)
    group.set_planner_id("RRTkConfigDefault")
    group.set_pose_reference_frame("/base_link")

    ### Then, we will get the current set of joint values for the group
    group_variable_values = group.get_current_joint_values()
    group_end_effector_link = group.get_end_effector_link()
    # print "======== Current Joint values: ", group_variable_values, "===="
    print "======== End Effector Link:", group_end_effector_link, "===="

    group.set_pose_target(pose)
    plan = group.plan()

    ### visualize in RVIZ
    print "==== Visualizing plan ===="
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = self.robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    display_trajectory_publisher.publish(display_trajectory)
    # rospy.sleep(1)

    ### Uncomment below line when working with a real robot
    group.go(wait=True)
    print "==== STOPPING ===="

  def dual_arm_pose_control(self, pose_left, pose_right):
    display_trajectory_publisher = rospy.Publisher(
      '/move_group/display_planned_path',
      moveit_msgs.msg.DisplayTrajectory)

    group = self.moveit_commander.MoveGroupCommander("dual_arm")
    group.set_planner_id("RRTkConfigDefault")
    group.set_pose_reference_frame("/base_link")
    group.set_pose_target(pose_left,"L_ee")
    group.set_pose_target(pose_right, "R_ee")

    scene = self.scence
    pose_Woodbox = geometry_msgs.msg.PoseStamped()
    pose_Woodbox.header.frame_id = "base_link"
    pose_Woodbox.pose.position.x = 1.01958
    pose_Woodbox.pose.position.y = 0.505413
    pose_Woodbox.pose.position.z = 1.16481
    quaternion = transformations.quaternion_from_euler(0.000456, -0.000142, -0.023473)
    pose_Woodbox.pose.orientation.x = quaternion[0]
    pose_Woodbox.pose.orientation.y = quaternion[1]
    pose_Woodbox.pose.orientation.z = quaternion[2]
    pose_Woodbox.pose.orientation.w = quaternion[3]
    scene.add_box("Woodbox", pose_Woodbox, (0.05, 0.05, 0.3))

    plan = group.plan()

    ### visualize in RVIZ
    print "==== Visualizing plan ===="
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = self.robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    display_trajectory_publisher.publish(display_trajectory)
    # rospy.sleep(1)

    ### Uncomment below line when working with a real robot
    group.go(wait=True)
    group.clear_pose_target("L_ee")
    group.clear_pose_target("R_ee")
    print "==== STOPPING ===="

  def data_to_pose(self, poses, orientations):
    pose = geometry_msgs.msg.Pose()
    pose.position.x = poses[0]
    pose.position.y = poses[1]
    pose.position.z = poses[2]
    pose.orientation.x = orientations[0]
    pose.orientation.y = orientations[1]
    pose.orientation.z = orientations[2]
    pose.orientation.w = orientations[3]
    return pose


if __name__ == '__main__':
  mrp2a = mrp2a_moveit_control()
  joint_left = [-0.6755863082045508, 1.3019358796609195, 2.760390101648138, 1.584628971690658, -0.8461974745458996, -1.5096349377470357, -1.5446433193905955] 
  joint_right = [-1.987835237660107e-05, -5.858705833361455e-05, -3.202692945780683e-05, 2.3819171068417688e-05, 3.912254331606135e-05, -5.986390414047804e-05, -2.7210040732583707e-05]
  mrp2a.dual_arm_joint_control(joint_left, joint_right)
  ### head and gripper control
  mrp2a.head_joint_control()
  mrp2a.dual_gripper_joint_control("open", "open")
  ### dual arm move
  joint_left = [0.9883204526370744, -1.3333185841293336, -1.2503535647337758, 0.9343609983898471, 3.0499947065090165, -1.182644482979481, -1.5818374736780534]
  joint_right = [-1.750, 0.511, 1.390, -1.799, 2.548, -1.938, -3.049]
  mrp2a.dual_arm_joint_control(joint_left, joint_right)

  joint_left = [1.377524982289395, -1.2841467521271515, -1.3118280174040837, 0.8639703843718953, 2.955805426153786, -0.8811142812226889, -1.7446712180583015]
  joint_right = [-1.8343902750279355, 0.7008803269013972, 1.5915867849141438, -1.000567151816611, 2.769612714252448, -1.3157834354666562, -2.8452852284467136]
  mrp2a.dual_arm_joint_control(joint_left, joint_right)

  mrp2a.dual_gripper_joint_control("close", "close")

  joint_left = [1.3776585200213072, -1.2840220299007408, -1.311883015081472, 0.8640111049836081, 2.955891836759683, -0.8811221489338141, -1.744711724577975]
  joint_right = [-1.2066298204777874, 0.6353936776839273, 1.2421693452061415, -1.38667851347311, 2.6280283558139352, -1.2885791378046232, -2.938055380422374]
  mrp2a.dual_arm_joint_control(joint_left, joint_right)

  joint_left = [1.3775515592058083, -1.284047301734626, -1.3119623481726075, 0.8640255874551706, 2.9559026275996665, -0.8811576523091817, -1.7446737856302192]
  joint_right = [-1.1398792639085977, 1.5687949803434345, 0.5045114446042618, -0.883077056223371, 0.95918833181614, -1.4667217132863204, -1.7556693885650017]
  mrp2a.dual_arm_joint_control(joint_left, joint_right)
