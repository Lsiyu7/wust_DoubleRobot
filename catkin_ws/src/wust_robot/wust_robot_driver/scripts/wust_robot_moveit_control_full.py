#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import numpy
import copy
import rospy
import actionlib
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import sensor_msgs.msg
from moveit_msgs.msg import RobotState, ObjectColor, PlanningScene
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
import tf
from tf import transformations
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import String
from mr_msgs.msg import GripperCommand
import math
from math import pi, sin, cos
#from ar_track_alvar_msgs.msg import  AlvarMarkers
import matplotlib as mpl
import matplotlib.pyplot as plt


class Pick_Place_Interface:
  # 初始化抓取和放置
  def __init__(self, group = 'arm', ee_group = 'gripper', plan_only = False):
    self._group = group
    self._effector = ee_group
    rospy.loginfo("Manipulator connection success!")
    self._pick_action = actionlib.SimpleActionClient('pickup', moveit_msgs.msg.PickupAction)
    self._pick_action.wait_for_server()
    self._place_action = actionlib.SimpleActionClient('place', moveit_msgs.msg.PlaceAction)
    self._place_action.wait_for_server()
    self._plan_only = plan_only

  # 抓取
  def Pickup(self, name, grasps, support_name = 'table',
             allow_gripper_support_collision = True,
             allowed_touch_objects = list()):

    g = moveit_msgs.msg.PickupGoal()

    # 配置抓取相关信息和参数
    g.target_name = name
    g.group_name = self._group
    g.end_effector = self._effector
    g.possible_grasps = grasps
    g.support_surface_name = support_name
    g.allow_gripper_support_collision = allow_gripper_support_collision
    g.attached_object_touch_links = list()
    g.planner_id = "RRTConnectkConfigDefault"
    g.allowed_touch_objects = allowed_touch_objects
    g.allowed_planning_time = 5.0
    g.planning_options.planning_scene_diff.is_diff = True
    g.planning_options.planning_scene_diff.robot_state.is_diff = True
    g.planning_options.plan_only = self._plan_only
    self._pick_action.send_goal(g)
    self._pick_action.wait_for_result()
    return self._pick_action.get_result().error_code.val

  # 放置
  def Place(self, name, locations, support_name='table',
            allow_gripper_support_collision=True,
            allowed_touch_objects=list(),
            goal_is_eef=True):
    g = moveit_msgs.msg.PlaceGoal()
    g.group_name = self._group
    g.attached_object_name = name
    g.place_locations = locations
    g.place_eef = goal_is_eef
    g.support_surface_name = support_name
    g.allow_gripper_support_collision = allow_gripper_support_collision
    g.planner_id = "RRTConnectkConfigDefault"
    g.allowed_touch_objects = allowed_touch_objects
    g.allowed_planning_time = 5.0
    g.planning_options.plan_only = self._plan_only
    self._place_action.send_goal(g)
    self._place_action.wait_for_result()
    return self._place_action.get_result().error_code.val

# 需要抓取的物体的信息
class Object_parameters:
  def __init__(self):
    self.name = ''
    self.size = [0, 0, 0]
    self.pose = [0, 0, 0, 0, 0, 0]
    self.place_pose = [0, 0, 0, 0, 0, 0]
    self.pick_rpy_left = [[0], [0], [0]]      # 抓取时夹爪姿态
    self.pick_rpy_right = [[0], [0], [0]]
    self.approach_min = 0                     # 夹爪靠近目标，开始改变姿态时与目标最小距离
    self.approach_desire = 0                  # 夹爪靠近目标，开始改变姿态时与目标期望距离
    self.approach_axis = [0, 0, 0]            # 夹爪靠近目标时切入方向
    self.retreat_min = 0                      # 夹爪撤离目标，开始改变姿态时与目标最小距离
    self.retreat_desire = 0                   # 夹爪撤离目标，开始改变姿态时与目标期望距离
    self.retreat_axis = [0, 0, 0]             # 夹爪撤离目标时切出方向
    self.compensate_left = [0, 0, 0]          # 抓取补偿（左夹爪）
    self.compensate_right = [0, 0, 0]         # 抓取补偿（右夹爪）
    self.place_rpy_left = [[0], [0], [0]]     # 放置时夹爪姿态
    self.place_rpy_right = [[0], [0], [0]]

class Wust_Robot_Moveit_Control:
  def __init__(self):
    # 初始化move_group的API
    moveit_commander.roscpp_initialize(sys.argv)
    #初始化ROS节点
    rospy.init_node('Wust_Robot_Moveit_Control', anonymous=True)
    # 初始化需要使用move group控制的机械臂中的group
    group_left_arm = moveit_commander.MoveGroupCommander("left_arm")
    group_right_arm = moveit_commander.MoveGroupCommander("right_arm")
    group_dual_gripper = moveit_commander.MoveGroupCommander("dual_gripper")
    group_dual_arm = moveit_commander.MoveGroupCommander("dual_arm")
    group_head = moveit_commander.MoveGroupCommander("head")
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    new_traj = moveit_commander.RobotTrajectory()
    cout = 99

    unit_matrix = numpy.array([[1, 0, 0],
                            [0, 1, 0],
                            [0, 0, 1]])
    #g.path_constraints = constraints
    
    
    self.unit_matrix = unit_matrix
    self.robot = robot
    self.scene = scene
    self.moveit_commander = moveit_commander
    self.group_left_arm = group_left_arm
    self.group_right_arm = group_right_arm
    self.group_dual_arm = group_dual_arm
    self.group_dual_gripper = group_dual_gripper
    self.group_head = group_head
    self.new_traj = new_traj
    self.cout = cout
    self.colors = dict()  # 创建一个存储物体颜色的字典对象
    self.scene_pub = rospy.Publisher('planning_scene', PlanningScene, queue_size=10) #创建一个发布场景变化信息的发布者

    left_pick_and_place = Pick_Place_Interface("left_arm", "left_gripper")
    right_pick_and_place = Pick_Place_Interface("right_arm", "right_gripper")
    self.left_pick_and_place = left_pick_and_place
    self.right_pick_and_place = right_pick_and_place

    # 发布一个在rviz中显示轨迹的话题
    display_trajectory_publisher = rospy.Publisher(
      '/move_group/display_planned_path',
      moveit_msgs.msg.DisplayTrajectory, queue_size=20)
    self.display_trajectory_publisher = display_trajectory_publisher

    # 实际平台夹爪姿态发布者
    gripper_pub_left = rospy.Publisher('/wust_robot/left_gripper/gripper_cmd', GripperCommand, queue_size=5)
    gripper_pub_right = rospy.Publisher('/wust_robot/right_gripper/gripper_cmd', GripperCommand, queue_size=5)
    self.girpper_pub_left = gripper_pub_left
    self.gripper_pub_right = gripper_pub_right

  # 订阅二维码识别话题
  # rospy.Subscriber('ar_pose_marker', '/ar_pose_marker', AlvarMarkers, self.ar_pose_cb)

  # def ar_pose_cb(self, msg):
  #   if len(msg.markers) > 0:
  #       pose_info = msg.markers[0].pose.pose

  # 在rviz中显示
  def Display_Trajectory(self, plan):
    robot = self.robot
    display_trajectory_publisher = self.display_trajectory_publisher
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    # ==== Publish ====
    display_trajectory_publisher.publish(display_trajectory)

  def Wait_For_Update(self, name, box_is_known=False, box_is_attached=False, timeout=4):
    scene = self.scene
    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
        # ==== Test if the box is in attached objects ====
        attached_objects = scene.get_attached_objects([name])
        is_attached = len(attached_objects.keys()) > 0
        # ==== Test if the box is in the scene. ====
        is_known = name in scene.get_known_object_names()

        # ==== Test if we are in the expected state ====
        if (box_is_attached == is_attached) and (box_is_known == is_known):
            return True
        # ==== Sleep so that we give other threads time on the processor ====
        rospy.sleep(1)
        seconds = rospy.get_time()

    # ==== If we exited the while loop without returning then we timed out ====
    return False

  #def forward_kinematic(self, group_name, joint):

  # 抓取物品（左臂、右臂）
  def Gripper_Pick(self, group_name, pose_stamped, object_parameters):
    pick_name = object_parameters
    if group_name == "left_arm":
        left_pick = self.left_pick_and_place
        grasp = self.Make_Grasps(group_name, pose_stamped, object_parameters)
        result = left_pick.Pickup(pick_name, grasp)
    elif group_name == "right_arm":
        right_pick = self.right_pick_and_place
        grasp = self.Make_Grasps(group_name, pose_stamped, object_parameters)
        result = right_pick.Pickup(pick_name, grasp)

  # 抓取参数
  def Make_Grasps(self, group_name, pose_stamped, object_parameters, mega_angle=False):
    if group_name == "left_arm":
        rpy = object_parameters.pick_rpy_left
        compensate = object_parameters.compensate_left
    elif group_name == "right_arm":
        rpy = object_parameters.pick_rpy_right
        compensate = object_parameters.compensate_right
    g = moveit_msgs.msg.Grasp()
    # 创建夹爪张开、闭合的姿态
    g.pre_grasp_posture = self.Make_Gripper_Posture(group_name, "open", object_parameters)
    g.grasp_posture = self.Make_Gripper_Posture(group_name, "close", object_parameters)
    # 设置夹爪靠近、撤离目标的参数
    g.pre_grasp_approach = self.Make_Gripper_Translation(group_name,
                                                         object_parameters.approach_min,
                                                         object_parameters.approach_desire,
                                                         object_parameters.approach_axis,
                                                         object_parameters)
    g.post_grasp_retreat = self.Make_Gripper_Translation(group_name,
                                                         object_parameters.retreat_min,
                                                         object_parameters.retreat_desire,
                                                         object_parameters.retreat_axis,
                                                         object_parameters)
    # 设置抓取补偿
    g.grasp_pose = pose_stamped
    g.grasp_pose.pose.position.x += compensate[0]
    g.grasp_pose.pose.position.y += compensate[1]
    g.grasp_pose.pose.position.z += compensate[2]

    rolls = rpy[0]
    pitchs = rpy[1]
    yaws = rpy[2]

    # 抓取姿态的列表
    grasps = []

    # 改变姿态，生成动作
    for roll in rolls:
      for pitch in pitchs:
        for yaw in yaws:
          # 欧拉角到四元数的转换
          q = transformations.quaternion_from_euler(roll, pitch, yaw)

          # 设置抓取的姿态
          g.grasp_pose.pose.orientation.x = q[0]
          g.grasp_pose.pose.orientation.y = q[1]
          g.grasp_pose.pose.orientation.z = q[2]
          g.grasp_pose.pose.orientation.w = q[3]
          g.id = str(len(grasps)) # 设置抓取的唯一id号
          # 抓取评分
          g.grasp_quality = 1.0 - abs(pitch)
          # 将本次规划的抓取放入抓取列表中
          grasps.append(copy.deepcopy(g))
    rospy.ROSException
    # 返回抓取列表
    return grasps

  # 抓取和放置时夹爪姿态数据JointTrajectory（闭合、张开）
  def Make_Gripper_Posture(self, group_name, control, object_parameters):
    # 初始化夹爪的关节运动轨迹
    t = JointTrajectory()
    t.header.frame_id = "base_link"
    t.header.stamp = rospy.Time.now()

    # 初始化关节轨迹点
    tp = JointTrajectoryPoint()
    if group_name == "left_arm":
        t.joint_names = ["L_left_joint", "L_right_joint"]
    elif group_name == "right_arm":
        t.joint_names = ["R_left_joint", "R_right_joint"]
    if control == "open":
        tp.positions = (0.36, -0.36)
    elif control == "close":
        tp.positions = (-0.14, 0.14)
    # ==== 仿真时需要设置 ====
    tp.time_from_start.secs = 1
    t.points.append(tp) # 将目标轨迹点加入到运动轨迹中
    # print " ==== ", t, " ==== " # 返回夹爪的关节运动轨迹
    return t

  # 使用给定向量创建夹爪的translation结构
  def Make_Gripper_Translation(self, group_name, min_dist, desired, axis, object_parameters):
    # 初始化translation对象
    g = moveit_msgs.msg.GripperTranslation()

    # 设置方向向量
    g.direction.vector.x = axis[0]
    g.direction.vector.y = axis[1]
    g.direction.vector.z = axis[2]

    # 设置参考坐标系
    if group_name == "left_arm":
        g.direction.header.frame_id = "L_ee"
    if group_name == "right_arm":
        g.direction.header.frame_id = "R_ee"

    # 设置最小和期望距离
    g.min_distance = min_dist
    g.desired_distance = desired

    return g

  # 放置物品（左臂、右臂）
  def Gripper_Place(self, group_name, pose_stamped, object_parameters):
    place_name = object_parameters.name
    if group_name == "left_arm":
        left_place = self.left_pick_and_place
        place = self.Make_Places(pose_stamped)
        result = left_place.Place(place_name, place)
    elif group_name == "right_arm":
        right_place = self.left_pick_and_place
        place = self.Make_Places(pose_stamped)
        result = right_place.Place(place_name, place)

  # 放置参数
  def Make_Places(self, group_name, pose_stamped, object_parameters, mega_angle=False):
    if group_name =="left_arm":
          rpy = object_parameters.place_rpy_left
          componsate = object_parameters.compensate_left

    elif group_name == "right_arm":
          rpy = object_parameters.place_rpy_right
          componsate = object_parameters.compensate_right
    # 设置夹爪靠近、撤离目标的参数
    g = moveit_msgs.msg.PlaceLocation()
    g.post_place_posture = self.Make_Gripper_Posture(group_name, "open", object_parameters)
    g.pre_place_approach = self.Make_Gripper_Translation(group_name,
                                                         object_parameters.approach_min,
                                                         object_parameters.approach_desire,
                                                         object_parameters.approach_axis,
                                                         object_parameters)
    g.post_place_retreat = self.Make_Gripper_Translation(group_name,
                                                         object_parameters.retreat_min,
                                                         object_parameters.retreat_desire,
                                                         object_parameters.retreat_axis,
                                                         object_parameters)
    # 设置放置姿态
    g.place_pose = pose_stamped
    rolls = rpy[0]
    pitchs = rpy[1]
    yaws = rpy[2]

    places = []
    # 改变姿态，生成动作
    for roll in rolls:
      for pitch in pitchs:
        for yaw in yaws:
          # 欧拉角到四元数的转换
          q = transformations.quaternion_from_euler(roll, pitch, yaw)
          # 设置放置的姿态
          g.place_pose.pose.orientation.x = q[0]
          g.place_pose.pose.orientation.y = q[1]
          g.place_pose.pose.orientation.z = q[2]
          g.place_pose.pose.orientation.w = q[3]
          g.id = str(len(places)) # 设置放置的唯一id号
          # 将该放置姿态加入列表
          places.append(copy.deepcopy(g))
    return places

  def head_joint_control(self, joint):
      group = self.group_head
      group.set_pose_reference_frame("/base_link")
      group_variable_values = group.get_current_joint_values()
      group_variable_values[0] = joint[0]
      group_variable_values[1] = joint[1]
      group.set_joint_value_target(group_variable_values)
      group.go(wait=True)
  def arm_joint(self, joint):
      group = self.group_dual_arm
      group.set_pose_reference_frame("/base_link")
      group_variable_values = group.get_current_joint_values()
      group_variable_values[10] = joint
      group.set_joint_value_target(group_variable_values)
      group.go(wait=True)
  # 单个关节角改变
  def arm_joint_change(self, group_name, joint):
    group = self.group_dual_arm
    group.set_pose_reference_frame("/base_link")
    group_variable_values = group.get_current_joint_values()

    if group_name == "L_joint1":
        group_variable_values[0] += joint
    if group_name == "L_joint2":
        group_variable_values[1] += joint
    if group_name == "L_joint3":
        group_variable_values[2] += joint
    if group_name == "L_joint4":
        group_variable_values[3] += joint
    if group_name == "L_joint5":
        group_variable_values[4] += joint
    if group_name == "L_joint6":
        group_variable_values[5] += joint
    if group_name == "L_joint7":
        group_variable_values[6] += joint
    if group_name == "R_joint1":
        group_variable_values[7] += joint
    if group_name == "R_joint2":
        group_variable_values[8] += joint
    if group_name == "R_joint3":
        group_variable_values[9] += joint
    if group_name == "R_joint4":
        group_variable_values[10] += joint
    if group_name == "R_joint5":
        group_variable_values[11] += joint
    if group_name == "R_joint6":
        group_variable_values[12] += joint
    if group_name == "R_joint7":
        group_variable_values[13] += joint

    group.set_joint_value_target(group_variable_values)
    group.go(wait=True)

    print "============ STOPPING"

  # 单臂关节运动
  def arm_joint_control(self, group_name, joint):
    if group_name == "left_arm":
        group = self.group_left_arm
    if group_name == "right_arm":
        group = self.group_right_arm

    group.set_pose_reference_frame("/base_link")
    group_variable_values = group.get_current_joint_values()
    group_variable_values[0] = joint[0]
    group_variable_values[1] = joint[1]
    group_variable_values[2] = joint[2]
    group_variable_values[3] = joint[3]
    group_variable_values[4] = joint[4]
    group_variable_values[5] = joint[5]
    group_variable_values[6] = joint[6]

    group.set_joint_value_target(group_variable_values)

    # group.plan()
    group.go(wait=True)
    end_effector_link = group.get_end_effector_link()
    end_pose = group.get_current_pose(end_effector_link).pose
    print end_pose
    print "============ STOPPING"

  # 双臂同时关节运动
  def dual_arm_joint_control(self, joint_left, joint_right):

    group = moveit_commander.MoveGroupCommander("dual_arm")
    group.set_pose_reference_frame("/base_link")
    group_variable_values = group.get_current_joint_values()

    # 设置左臂、右臂每个关节的关节角
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
    plan = group.plan()
    group.execute(plan)
    print "============ STOPPING"

  # 单臂笛卡尔运动
  def Plan_Cartesian_Path(self, group_name, pose, scale=1):
    # 是否需要使用笛卡尔运动
    cartesian = rospy.get_param('~cartesian', True)

    if group_name == "left_arm":
        group = self.group_left_arm
        wpose = group.get_current_pose("L_ee").pose
    elif group_name == "right_arm":
        group = self.group_right_arm
        wpose = group.get_current_pose("R_ee").pose

    group.set_pose_reference_frame("/base_link")
    # 设置位置和姿态的允许误差
    group.set_goal_position_tolerance(0.01)
    group.set_goal_orientation_tolerance(0.05)

    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x = pose[0]
    target_pose.position.y = pose[1]
    target_pose.position.z = pose[2]

    # 初始化路点列表
    waypoints = []
    # 路点数据
    wpose.position.x += scale * (pose[0])
    wpose.position.y += scale * (pose[1])
    wpose.position.z += scale * (pose[2])

    # 笛卡尔运动
    if cartesian:
        waypoints.append(copy.deepcopy(wpose))

        fraction = 0.0
        i = 0
        # while fraction < 1.0:
        (plan, fraction) = group.compute_cartesian_path(
                                           waypoints,   # 路点列表
                                           0.01,        # 终端步进值
                                           0.0)         # 跳跃阈值
        print 'fraction = ', fraction
        # if fraction < 1.0:
        #     rospy.logerr(group_name, " cartesian planning failure")
        #     return 0

    self.Display_Trajectory(plan)
    #raw_input()
    group.execute(plan, wait=True)

  # 双臂同时笛卡尔运动
  def dual_Plan_Cartesian_Path(self, pose_left, pose_right, scale=1):
    # 是否需要使用笛卡尔运动
    cartesian = rospy.get_param('~cartesian', True)
    group = self.group_dual_arm
    group_left = self.group_left_arm
    group_right = self.group_right_arm
    wpose_left = group_left.get_current_pose("L_ee").pose
    wpose_right = group_right.get_current_pose("R_ee").pose
    group.set_pose_reference_frame("/base_link")


    # 设置位置和姿态的允许误差
    group.set_goal_position_tolerance(0.01)
    group.set_goal_orientation_tolerance(0.05)

    target_pose_left = geometry_msgs.msg.Pose()
    target_pose_right = geometry_msgs.msg.Pose()
    target_pose_left.position.x = pose_left[0]
    target_pose_left.position.y = pose_left[1]
    target_pose_left.position.z = pose_left[2]
    target_pose_right.position.x = pose_right[0]
    target_pose_right.position.y = pose_right[1]
    target_pose_right.position.z = pose_right[2]

    # 左臂路点数据
    waypoints_left = []

    wpose_left.position.x += scale * (pose_left[0])
    wpose_left.position.y += scale * (pose_left[1])
    wpose_left.position.z += scale * (pose_left[2])

    # 右臂路点数据
    waypoints_right = []

    wpose_right.position.x += scale * (pose_right[0])
    wpose_right.position.y += scale * (pose_right[1])
    wpose_right.position.z += scale * (pose_right[2])

    # 笛卡尔运动
    if cartesian:
        waypoints_left.append(copy.deepcopy(wpose_left))
        waypoints_right.append(copy.deepcopy(wpose_right))
        n = 0
        while True:
            fraction1 = 0.0
            fraction2 = 0.0
            i = 0
            j = 0
            while fraction1 < 1.0 and i < 10:
                (plan_left, fraction1) = group_left.compute_cartesian_path(
                                                              waypoints_left,  # 路点列表
                                                              0.01,  # 终端步进值
                                                              0.0)  # 跳跃阈值
            while fraction2 < 1.0 and j < 10:
                (plan_right, fraction2) = group_right.compute_cartesian_path(
                                                              waypoints_right,  # 路点列表
                                                              0.01,  # 终端步进值
                                                              0.0)  # 跳跃阈值
            # 左臂和右臂的points的数量
            len_left = len(plan_left.joint_trajectory.points)
            len_right = len(plan_right.joint_trajectory.points)

            if len_left != len_right:
                print "left = ", len_left, "right = ", len_right
                n = n+1
                while n == 20:
                    rospy.logerr("Dual_arm cartesian planning failure")
                    return 0
                continue
            else:
                print "left = ", len_left, "right = ", len_right
                break
        print "n = ", n

    # 双臂的plan
    plan = group.plan()
    plan.joint_trajectory.joint_names = plan_left.joint_trajectory.joint_names + plan_right.joint_trajectory.joint_names
    i = 0
    while i < len_left:
        plan.joint_trajectory.points.append(plan_left.joint_trajectory.points[i])
        i = i+1

    j = 0
    while j < len_left:
        plan.joint_trajectory.points[j].positions = plan_left.joint_trajectory.points[j].positions + \
                                                    plan_right.joint_trajectory.points[j].positions
        plan.joint_trajectory.points[j].velocities = plan_left.joint_trajectory.points[j].velocities + \
                                                     plan_right.joint_trajectory.points[j].velocities
        plan.joint_trajectory.points[j].accelerations = plan_left.joint_trajectory.points[j].accelerations + \
                                                        plan_right.joint_trajectory.points[j].accelerations
        j = j+1

    # 将左臂的plan和右臂的plan融合到一起
    # min_point = min(len_left, len_right)
    # max_point = max(len_left, len_right)
    # plan.joint_trajectory.joint_names = plan_left.joint_trajectory.joint_names + plan_right.joint_trajectory.joint_names
    #
    # i = 0
    # if len_left > len_right:
    #     while i < len_left:
    #         plan.joint_trajectory.points.append(plan_left.joint_trajectory.points[i])
    #         i = i+1
    # else:
    #     while i < len_right:
    #         plan.joint_trajectory.points.append(plan_right.joint_trajectory.points[i])
    #         i = i+1
    # j = 0
    # while j < max_point:
    #     while j < min_point:
    #         plan.joint_trajectory.points[j].positions = plan_left.joint_trajectory.points[j].positions + \
    #                                                     plan_right.joint_trajectory.points[j].positions
    #         plan.joint_trajectory.points[j].velocities = plan_left.joint_trajectory.points[j].velocities + \
    #                                                     plan_right.joint_trajectory.points[j].velocities
    #         plan.joint_trajectory.points[j].accelerations = plan_left.joint_trajectory.points[j].accelerations + \
    #                                                     plan_right.joint_trajectory.points[j].accelerations
    #         j = j+1
    #         continue
    #     k = j
    #     while k < max_point:
    #         if len_left > len_right:
    #             plan.joint_trajectory.points[j].positions = plan_left.joint_trajectory.points[k].positions + \
    #                                                         plan_right.joint_trajectory.points[min_point-1].positions
    #             plan.joint_trajectory.points[j].velocities = plan_left.joint_trajectory.points[k].velocities + \
    #                                                         plan_right.joint_trajectory.points[min_point-1].velocities
    #             plan.joint_trajectory.points[j].accelerations = plan_left.joint_trajectory.points[k].accelerations + \
    #                                                     plan_right.joint_trajectory.points[min_point-1].accelerations
    #         elif len_left < len_right:
    #             plan.joint_trajectory.points[j].positions = plan_left.joint_trajectory.points[min_point-1].positions + \
    #                                                         plan_right.joint_trajectory.points[k].positions
    #             plan.joint_trajectory.points[j].velocities = plan_left.joint_trajectory.points[min_point-1].velocities \
    #                                                          + plan_right.joint_trajectory.points[k].velocities
    #             plan.joint_trajectory.points[j].accelerations = \
    #                 plan_left.joint_trajectory.points[min_point-1].accelerations + \
    #                                                     plan_right.joint_trajectory.points[k].accelerations
    #         k = k+1
    #         j = j+1
    #raw_input()
    print "fraction1 = ", fraction1, "fraction2 = ", fraction2
    print "left points = ", len_left
    print "right points = ", len_right

    # i = 0
    # while i < len_left:
    #     print plan.joint_trajectory.points[i].time_from_start.secs
    #     print plan.joint_trajectory.points[i].time_from_start.nsecs
    #     i = i+1
    self.Display_Trajectory(plan)
    raw_input()
    group.execute(plan, wait=True)
    print "========== STOP =========="

  # 工作空间运动(pos,rot)
  def arm_pose_control(self, group_name, pos, rot):
    if group_name == "left_arm":
        group = self.group_left_arm
    elif group_name == "right_arm":
        group = self.group_right_arm
    group.set_planner_id("RRTConnect")
    group.set_pose_reference_frame("base_link")

    # 获取末端执行器当前姿态
    group_end_effector_link = group.get_end_effector_link()
    # 设置目标点末端执行器的位姿
    target_pose = geometry_msgs.msg.PoseStamped()
    target_pose.header.frame_id = "base_link"
    target_pose.pose.position.x = pos[0]
    target_pose.pose.position.y = pos[1]
    target_pose.pose.position.z = pos[2]
    quaternion = transformations.quaternion_from_euler(rot[0], rot[1], rot[2])
    target_pose.pose.orientation.x = quaternion[0]
    target_pose.pose.orientation.y = quaternion[1]
    target_pose.pose.orientation.z = quaternion[2]
    target_pose.pose.orientation.w = quaternion[3]

    print target_pose.pose.orientation.x
    print target_pose.pose.orientation.y

    print target_pose.pose.orientation.z
    print target_pose.pose.orientation.w

    group.set_pose_target(target_pose, group_end_effector_link)
    plan = group.plan()
    a = geometry_msgs.msg.Transform(target_pose.pose.position, target_pose.pose.orientation)
    #raw_input()
    group.execute(plan)

  #  print group.get_current_pose(group_end_effector_link).pose

  # 双臂同时工作空间运动
  def dual_arm_pose_control(self, pos_left, rot_left, pos_right, rot_right):

    group = self.group_dual_arm
    #group.set_planner_id("LazyPRMstarkConfigDefault")
    group.set_planner_id("RRTConnect")
    #group.set_planner_id("RRTstarkConfigDefault")
    group.set_pose_reference_frame("/base_link")

    group.allow_replanning(True)

    target_pose_left = geometry_msgs.msg.PoseStamped()
    target_pose_right = geometry_msgs.msg.PoseStamped()
    target_pose_left.header.frame_id = 'base_link'
    target_pose_left.pose.position.x = pos_left[0]
    target_pose_left.pose.position.y = pos_left[1]
    target_pose_left.pose.position.z = pos_left[2]
    quaternion = transformations.quaternion_from_euler(rot_left[0], rot_left[1], rot_left[2])
    target_pose_left.pose.orientation.x = quaternion[0]
    target_pose_left.pose.orientation.y = quaternion[1]
    target_pose_left.pose.orientation.z = quaternion[2]
    target_pose_left.pose.orientation.w = quaternion[3]

    #angle = transformations.euler_from_quaternion(quaternion[0],quaternion[1],quaternion[2],quaternion[3])
    #print angle

    target_pose_right.header.frame_id = 'base_link'
    target_pose_right.pose.position.x = pos_right[0]
    target_pose_right.pose.position.y = pos_right[1]
    target_pose_right.pose.position.z = pos_right[2]
    quaternion = transformations.quaternion_from_euler(rot_right[0], rot_right[1], rot_right[2])
    target_pose_right.pose.orientation.x = quaternion[0]
    target_pose_right.pose.orientation.y = quaternion[1]
    target_pose_right.pose.orientation.z = quaternion[2]
    target_pose_right.pose.orientation.w = quaternion[3]

    group.set_pose_target(target_pose_left, "L_ee")
    group.set_pose_target(target_pose_right, "R_ee")

    # 设置方向约束
    # constraints = moveit_msgs.msg.Constraints()
    # orientation_constraints = moveit_msgs.msg.OrientationConstraint()
    # orientation_constraints.header = target_pose_left.header
    # orientation_constraints.link_name = "L_ee"
    # orientation_constraints.orientation.x = 0.5
    # orientation_constraints.orientation.y = 0.5
    # orientation_constraints.orientation.z = 0.5
    # orientation_constraints.orientation.w = 0.5
    # orientation_constraints.absolute_x_axis_tolerance = 0.14
    # orientation_constraints.absolute_y_axis_tolerance = 0.14
    # orientation_constraints.absolute_z_axis_tolerance = 0.14
    # orientation_constraints.weight = 1.0
    # constraints.orientation_constraints.append(orientation_constraints)
    # group.set_path_constraints(constraints)

    # 设置目标点末端执行器的位姿
    plan = group.plan()

    # traj_pub = rospy.Publisher('/traj_point', String, queue_size=10)
    # rate = rospy.Rate(10)  # 10hz
    # # i = 0
    # # while (i <= 10):
    # # #while not rospy.is_shutdown():
    # #     traj = "start"
    # #     rospy.loginfo(traj)
    # #     traj_pub.publish(traj)
    # #     i = i+1
    # #     rate.sleep()

    #raw_input()

    group.execute(plan)

    # while (i <= 10):
    # #while not rospy.is_shutdown():
    #     traj = "end"
    #     rospy.loginfo(traj)
    #     traj_pub.publish(traj)
    #     i = i+1
    #     rate.sleep()

    pnum = len(plan.joint_trajectory.points)
    print "points = ", pnum
    #print plan

    group.clear_pose_target("L_ee")
    group.clear_pose_target("R_ee")

  # 双臂同时工作空间运动+display
  def dual_arm_pose_control_display(self, pos_left, rot_left, pos_right, rot_right, N):

    group = self.group_dual_arm
    #group.set_planner_id("LazyPRMstarkConfigDefault")
    group.set_planner_id("RRTConnect")
    #group.set_planner_id("RRTstarkConfigDefault")
    group.set_pose_reference_frame("/base_link")

    group.allow_replanning(True)

    target_pose_left = geometry_msgs.msg.PoseStamped()
    target_pose_right = geometry_msgs.msg.PoseStamped()
    target_pose_left.header.frame_id = 'base_link'
    target_pose_left.pose.position.x = pos_left[0]
    target_pose_left.pose.position.y = pos_left[1]
    target_pose_left.pose.position.z = pos_left[2]
    quaternion = transformations.quaternion_from_euler(rot_left[0], rot_left[1], rot_left[2])
    target_pose_left.pose.orientation.x = quaternion[0]
    target_pose_left.pose.orientation.y = quaternion[1]
    target_pose_left.pose.orientation.z = quaternion[2]
    target_pose_left.pose.orientation.w = quaternion[3]

    #angle = transformations.euler_from_quaternion(quaternion[0],quaternion[1],quaternion[2],quaternion[3])
    #print angle

    target_pose_right.header.frame_id = 'base_link'
    target_pose_right.pose.position.x = pos_right[0]
    target_pose_right.pose.position.y = pos_right[1]
    target_pose_right.pose.position.z = pos_right[2]
    quaternion = transformations.quaternion_from_euler(rot_right[0], rot_right[1], rot_right[2])
    target_pose_right.pose.orientation.x = quaternion[0]
    target_pose_right.pose.orientation.y = quaternion[1]
    target_pose_right.pose.orientation.z = quaternion[2]
    target_pose_right.pose.orientation.w = quaternion[3]

    group.set_pose_target(target_pose_left, "L_ee")
    group.set_pose_target(target_pose_right, "R_ee")

    # 设置方向约束
    # constraints = moveit_msgs.msg.Constraints()
    # orientation_constraints = moveit_msgs.msg.OrientationConstraint()
    # orientation_constraints.header = target_pose_left.header
    # orientation_constraints.link_name = "L_ee"
    # orientation_constraints.orientation.x = 0.5
    # orientation_constraints.orientation.y = 0.5
    # orientation_constraints.orientation.z = 0.5
    # orientation_constraints.orientation.w = 0.5
    # orientation_constraints.absolute_x_axis_tolerance = 0.14
    # orientation_constraints.absolute_y_axis_tolerance = 0.14
    # orientation_constraints.absolute_z_axis_tolerance = 0.14
    # orientation_constraints.weight = 1.0
    # constraints.orientation_constraints.append(orientation_constraints)
    # group.set_path_constraints(constraints)

    # 设置目标点末端执行器的位姿
    plan = group.plan()

    # traj_pub = rospy.Publisher('/traj_point', String, queue_size=10)
    # rate = rospy.Rate(10)  # 10hz
    # # i = 0
    # # while (i <= 10):
    # # #while not rospy.is_shutdown():
    # #     traj = "start"
    # #     rospy.loginfo(traj)
    # #     traj_pub.publish(traj)
    # #     i = i+1
    # #     rate.sleep()

    #raw_input()

    if N==1:
        q1 = []
        q2 = []
        q3 = []
        q4 = []
        q5 = []
        q6 = []
        q7 = []

        v1 = []
        v2 = []
        v3 = []
        v4 = []
        v5 = []
        v6 = []
        v7 = []

        a1 = []
        a2 = []
        a3 = []
        a4 = []
        a5 = []
        a6 = []
        a7 = []

        print len(plan.joint_trajectory.points)
        print plan.joint_trajectory
        for j in range(len(plan.joint_trajectory.points)):
            # q1.append(plan.joint_trajectory.points[j].positions[0])
            # q2.append(plan.joint_trajectory.points[j].positions[1])
            # q3.append(plan.joint_trajectory.points[j].positions[2])
            # q4.append(plan.joint_trajectory.points[j].positions[3])
            # q5.append(plan.joint_trajectory.points[j].positions[4])
            # q6.append(plan.joint_trajectory.points[j].positions[5])
            # q7.append(plan.joint_trajectory.points[j].positions[6])
            # v1.append(plan.joint_trajectory.points[j].velocities[0])
            # v2.append(plan.joint_trajectory.points[j].velocities[1])
            # v3.append(plan.joint_trajectory.points[j].velocities[2])
            # v4.append(plan.joint_trajectory.points[j].velocities[3])
            # v5.append(plan.joint_trajectory.points[j].velocities[4])
            # v6.append(plan.joint_trajectory.points[j].velocities[5])
            # v7.append(plan.joint_trajectory.points[j].velocities[6])
            a1.append(plan.joint_trajectory.points[j].accelerations[0])
            a2.append(plan.joint_trajectory.points[j].accelerations[1])
            a3.append(plan.joint_trajectory.points[j].accelerations[2])
            a4.append(plan.joint_trajectory.points[j].accelerations[3])
            a5.append(plan.joint_trajectory.points[j].accelerations[4])
            a6.append(plan.joint_trajectory.points[j].accelerations[5])
            a7.append(plan.joint_trajectory.points[j].accelerations[6])

            # q1.append(plan.joint_trajectory.points[j].positions[7])
            # q2.append(plan.joint_trajectory.points[j].positions[8])
            # q3.append(plan.joint_trajectory.points[j].positions[9])
            # q4.append(plan.joint_trajectory.points[j].positions[10])
            # q5.append(plan.joint_trajectory.points[j].positions[11])
            # q6.append(plan.joint_trajectory.points[j].positions[12])
            # q7.append(plan.joint_trajectory.points[j].positions[13])
            # v1.append(plan.joint_trajectory.points[j].velocities[7])
            # v2.append(plan.joint_trajectory.points[j].velocities[8])
            # v3.append(plan.joint_trajectory.points[j].velocities[9])
            # v4.append(plan.joint_trajectory.points[j].velocities[10])
            # v5.append(plan.joint_trajectory.points[j].velocities[11])
            # v6.append(plan.joint_trajectory.points[j].velocities[12])
            # v7.append(plan.joint_trajectory.points[j].velocities[13])
            # a1.append(plan.joint_trajectory.points[j].accelerations[7])
            # a2.append(plan.joint_trajectory.points[j].accelerations[8])
            # a3.append(plan.joint_trajectory.points[j].accelerations[9])
            # a4.append(plan.joint_trajectory.points[j].accelerations[10])
            # a5.append(plan.joint_trajectory.points[j].accelerations[11])
            # a6.append(plan.joint_trajectory.points[j].accelerations[12])
            # a7.append(plan.joint_trajectory.points[j].accelerations[13])
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

        # plt.plot(v1)
        # plt.plot(v2)
        # plt.plot(v3)
        # plt.plot(v4)
        # plt.plot(v5)
        # plt.plot(v6)
        # plt.plot(v7)
        # plt.grid(True)
        # plt.axis('tight')
        # plt.xlabel('points')
        # plt.ylabel('velocities')

        plt.plot(a1)
        plt.plot(a2)
        plt.plot(a3)
        plt.plot(a4)
        plt.plot(a5)
        plt.plot(a6)
        plt.plot(a7)
        plt.grid(True)
        plt.axis('tight')
        plt.xlabel('points')
        plt.ylabel('accelerations')
        plt.ylim(-1.5, 1.5)
        # # plt.xlim(0, 250)
        plt.show()

    group.execute(plan)

    # while (i <= 10):
    # #while not rospy.is_shutdown():
    #     traj = "end"
    #     rospy.loginfo(traj)
    #     traj_pub.publish(traj)
    #     i = i+1
    #     rate.sleep()

    pnum = len(plan.joint_trajectory.points)
    print "points = ", pnum
    #print plan

    group.clear_pose_target("L_ee")
    group.clear_pose_target("R_ee")


  # 单个夹爪姿态数据
  def gripper_joint_control(self, group_name, control, t):
    # 仿真平台
    group = self.group_dual_gripper
    group.set_pose_reference_frame("/base_link")

    group_variable_values = group.get_current_joint_values()

    if group_name == "left_gripper":
        if control == "open":
          group_variable_values[0] = 0.35
          group_variable_values[1] = -0.35
        elif control == "close":
          group_variable_values[0] = -t
          group_variable_values[1] = t
    elif group_name == "right_gripper":
        if control == "open":
          group_variable_values[2] = 0.35
          group_variable_values[3] = -0.35
        elif control == "close":
          group_variable_values[2] = -t
          group_variable_values[3] = t
    group.set_joint_value_target(group_variable_values)
    group.plan()
    #raw_input()
    group.go(wait=True)

    # 实际平台
    # gripper_cmd = GripperCommand()
    # if group_name == "left_gripper":
    #     gripper_cmd.name = "Gripper1"
    #     if control == "open":
    #         gripper_cmd.positionL = 0.00
    #         gripper_cmd.positionR = 0.0
    #     elif control == "close":
    #         gripper_cmd.positionL = 390.0
    #         gripper_cmd.positionR = 390.0
    #     self.girpper_pub_left.publish(gripper_cmd)
    # elif group_name == "right_gripper":
    #     gripper_cmd.name = "Gripper2"
    #     if control == "open":
    #         gripper_cmd.positionL = 0.0
    #         gripper_cmd.positionR = 0.0
    #     elif control == "close":
    #         gripper_cmd.positionL = 390.0
    #         gripper_cmd.positionR = 390.0
    #     self.gripper_pub_right.publish(gripper_cmd)
    # print " = ", gripper_cmd, "= \n"
    # rospy.sleep(2)

  # 两个夹爪姿态数据
  def dual_gripper_joint_control(self, control_left, control_right, l, r):
    # 仿真平台
    group = self.group_dual_gripper
    group.set_pose_reference_frame("/base_link")

    group_variable_values = group.get_current_joint_values()

    if control_left == "open":
      group_variable_values[0] = 0.35
      group_variable_values[1] = -0.35
    elif control_left == "close":
      group_variable_values[0] = -l
      group_variable_values[1] = l
    if control_right == "open":
      group_variable_values[2] = 0.35
      group_variable_values[3] = -0.35
    elif control_right == "close":
      group_variable_values[2] = -r
      group_variable_values[3] = r
    group.set_joint_value_target(group_variable_values)
    group.plan()
    group.go(wait=True)

    # 实际平台
    # gripper_cmd_left = GripperCommand()
    # gripper_cmd_right = GripperCommand()
    # gripper_cmd_left.name = "Gripper1"
    # gripper_cmd_right.name = "Gripper2"
    #
    # if control_left == "open":
    #     gripper_cmd_left.positionL = 0.0
    #     gripper_cmd_left.positionR = 0.0
    # elif control_left == "close":
    #     gripper_cmd_left.positionL = 390.0
    #     gripper_cmd_left.positionR = 390.0
    # self.girpper_pub_left.publish(gripper_cmd_left)
    #
    # if control_right == "open":
    #     gripper_cmd_right.positionL = 0.0
    #     gripper_cmd_right.positionR = 0.0
    # elif control_right == "close":
    #     gripper_cmd_right.positionL = 390.0
    #     gripper_cmd_right.positionR = 390.0
    # self.gripper_pub_right.publish(gripper_cmd_right)
    # # print " = ", gripper_cmd, "= \n"
    # rospy.sleep(2)

  # 物体姿态转换（欧拉角转四元数）
  def Pose_Transform(self, pose_euler):
    # br = tf.TransformBroadcaster()
    # br.sendTransform()

    pose_quaternion = geometry_msgs.msg.PoseStamped()
    pose_quaternion.header.frame_id = "base_link"
    pose_quaternion.pose.position.x = pose_euler[0]
    pose_quaternion.pose.position.y = pose_euler[1]
    pose_quaternion.pose.position.z = pose_euler[2]
    quaternion = transformations.quaternion_from_euler(pose_euler[3], pose_euler[4], pose_euler[5])
    pose_quaternion.pose.orientation.x = quaternion[0]
    pose_quaternion.pose.orientation.y = quaternion[1]
    pose_quaternion.pose.orientation.z = quaternion[2]
    pose_quaternion.pose.orientation.w = quaternion[3]
    return pose_quaternion

  # 添加物体
  def Add_Box(self, name, pose, size, timeout=4):
    scene = self.scene
    rospy.sleep(0.5)
    is_known = name in scene.get_known_object_names()
    if is_known == True:
      self.Remove_Box(name)
    scene.add_box(name, pose, size)
    return self.Wait_For_Update(name, box_is_known=True, timeout=timeout)

  #移除物体
  def Remove_Box(self, name, timeout=4):
    scene = self.scene
    scene.remove_world_object(name)

    return self.Wait_For_Update(name, box_is_attached=False, box_is_known=False, timeout=timeout)

  # 将物体附着在夹爪上，表明可抓取
  def Attach_Box(self, group_name, name, timeout=4):
    robot = self.robot
    scene = self.scene
    if group_name == "left_arm":
      ee_link = "L_ee"
      grasping_group = 'left_gripper'
    elif group_name == "right_arm":
      ee_link = "R_ee"
      grasping_group = 'right_gripper'
    touch_links = robot.get_link_names(group=grasping_group)
    scene.attach_box(ee_link, name, touch_links=touch_links)

    return self.Wait_For_Update(name, box_is_attached=True, box_is_known=False, timeout=timeout)

  # 将夹爪和物体分开
  def Detach_Box(self, group_name, name, timeout=4):
    scene = self.scene
    scene = self.scene
    if group_name == "left_arm":
      ee_link = "L_ee"
      grasping_group = 'left_gripper'
    elif group_name == "right_arm":
      ee_link = "R_ee"
      grasping_group = 'right_gripper'

    # 移除场景中之前与机器臂绑定的物体
    scene.remove_attached_object(ee_link, name)

    return self.Wait_For_Update(name, box_is_known=True, box_is_attached=False, timeout=timeout)


  # 双臂同时单轴运动
  def dual_arm_shift_control(self, axis_left, value_left, axis_right, value_right):
    group = self.group_dual_arm
    group.set_pose_reference_frame("/base_link")

    group.shift_pose_target(axis_left, value_left, "L_ee")
    group.shift_pose_target(axis_right, value_right, "R_ee")

    # 设置目标点末端执行器的位姿
    plan = group.plan()
    #raw_input()
    group.execute(plan)
    group.clear_pose_target("L_ee")
    group.clear_pose_target("R_ee")

  #工作空间运动（poses）
  def arm_poses_control(self, group_name, poses):
    if group_name == "left_arm":
        group = self.group_left_arm
    elif group_name == "right_arm":
        group = self.group_right_arm
    group.set_planner_id("RRTConnectkConfigDefault")
    group.set_pose_reference_frame("/base_link")

    # 获取末端执行器当前姿态
    group_end_effector_link = group.get_end_effector_link()
    # 设置目标点末端执行器的位姿
    group.set_pose_targets(poses)
    plan = group.plan()
    #raw_input()
    group.execute(plan)


  # 单臂笛卡尔运动（直线插补）
  def Plan_Cartesian_Path_Inter(self, group_name, pose, N, scale=1):
      # 是否需要使用笛卡尔运动
      cartesian = rospy.get_param('~cartesian', True)

      if group_name == "left_arm":
          group = self.group_left_arm
          start = group.get_current_pose("L_ee").pose
      elif group_name == "right_arm":
          group = self.group_right_arm
          start = group.get_current_pose("R_ee").pose
      group.set_pose_reference_frame("/base_link")
      # 设置位置和姿态的允许误差
      group.set_goal_position_tolerance(0.01)
      group.set_goal_orientation_tolerance(0.05)

      target_pose = geometry_msgs.msg.Pose()
      target_pose.position.x = pose[0]
      target_pose.position.y = pose[1]
      target_pose.position.z = pose[2]

      waypoints = []
      # 路点数据
      wpose = copy.deepcopy(start)

      wpose.position.x += scale * (pose[0])
      wpose.position.y += scale * (pose[1])
      wpose.position.z += scale * (pose[2])

      # wpose.position.x += scale * (pose.position.x)
      # wpose.position.y += scale * (pose.position.y)
      # wpose.position.z += scale * (pose.position.z)

      # 直线插补
      X = numpy.square(wpose.position.x - start.position.x)
      Y = numpy.square(wpose.position.y - start.position.y)
      Z = numpy.square(wpose.position.z - start.position.z)
      L = numpy.sqrt(X + Y + Z)
      D = L / (N + 1)

      n1 = (wpose.position.x - start.position.x) / L
      n2 = (wpose.position.y - start.position.y) / L
      n3 = (wpose.position.z - start.position.z) / L

      # 插补路点
      waypoint = copy.deepcopy(start)
      i = 1
      while i <= N:
          x0 = start.position.x
          y0 = start.position.y
          z0 = start.position.z

          waypoint.position.x = x0 + n1*i*D
          waypoint.position.y = y0 + n2*i*D
          waypoint.position.z = z0 + n3*i*D

          waypoints.append(copy.deepcopy(waypoint))

          i = i + 1
      print "len_waypoints = ", len(waypoints)
      # 笛卡尔运动
      if cartesian:
          waypoints.append(copy.deepcopy(wpose))

          fraction = 0.0
          i = 0
          while fraction < 1.0 and i < 5:
              (plan, fraction) = group.compute_cartesian_path(
                  waypoints,  # 路点列表
                  0.01,  # 终端步进值
                  0.0)  # 跳跃阈值
              i = i + 1
              print 'fraction = ', fraction
      #     if fraction < 0.5:
      #         rospy.logerr(group_name, " cartesian planning failure")
      #         return 0
      #     num_point = len(plan.joint_trajectory.points)
      #
      # print "points = ", num_point
      #
      # self.Display_Trajectory(plan)
      # raw_input()

      n = len(plan.joint_trajectory.points)
      for i in range(n):
        print "joints[",i,"]=",plan.joint_trajectory.points[i].positions
      print "**************"
      group.execute(plan, wait=True)



  # 双臂同时笛卡尔运动（直线插补）
  def dual_Plan_Cartesian_Path_Inter(self, pose_left, pose_right, N, scale=1):
      # 是否需要使用笛卡尔运动
      cartesian = rospy.get_param('~cartesian', True)
      group = self.group_dual_arm
      group_left = self.group_left_arm
      group_right = self.group_right_arm
      start_left = group_left.get_current_pose("L_ee").pose
      start_right = group_right.get_current_pose("R_ee").pose
      group.set_pose_reference_frame("/base_link")

      # 设置位置和姿态的允许误差
      group.set_goal_position_tolerance(0.01)
      group.set_goal_orientation_tolerance(0.05)

      target_pose_left = geometry_msgs.msg.Pose()
      target_pose_right = geometry_msgs.msg.Pose()
      target_pose_left.position.x = pose_left[0]
      target_pose_left.position.y = pose_left[1]
      target_pose_left.position.z = pose_left[2]
      target_pose_right.position.x = pose_right[0]
      target_pose_right.position.y = pose_right[1]
      target_pose_right.position.z = pose_right[2]

      # 左臂路点数据
      waypoints_left = []

      wpose_left = copy.deepcopy(start_left)

      wpose_left.position.x += scale * (pose_left[0])
      wpose_left.position.y += scale * (pose_left[1])
      wpose_left.position.z += scale * (pose_left[2])

      w_left_x = wpose_left.position.x
      w_left_y = wpose_left.position.y
      w_left_z = wpose_left.position.z

      # 左臂直线插补
      X = numpy.square(w_left_x-start_left.position.x)
      Y = numpy.square(w_left_y-start_left.position.y)
      Z = numpy.square(w_left_z-start_left.position.z)

      L = numpy.sqrt(X + Y + Z)
      D1 = L / (N + 1)

      l1 = (w_left_x - start_left.position.x) / L
      l2 = (w_left_y - start_left.position.y) / L
      l3 = (w_left_z - start_left.position.z) / L

      waypoint_left = copy.deepcopy(start_left)
      i = 1
      while i <= N:
          x0 = start_left.position.x
          y0 = start_left.position.y
          z0 = start_left.position.z

          waypoint_left.position.x = x0 + l1*i*D1
          waypoint_left.position.y = y0 + l2*i*D1
          waypoint_left.position.z = z0 + l3*i*D1

          waypoints_left.append(copy.deepcopy(waypoint_left))

          i = i + 1
      print "len_waypoints_left = ", len(waypoints_left)


      # 右臂路点数据
      waypoints_right = []

      wpose_right = copy.deepcopy(start_right)

      wpose_right.position.x += scale * (pose_right[0])
      wpose_right.position.y += scale * (pose_right[1])
      wpose_right.position.z += scale * (pose_right[2])

      w_right_x = wpose_right.position.x
      w_right_y = wpose_right.position.y
      w_right_z = wpose_right.position.z

      # 右臂直线插补
      X2 = numpy.square(w_right_x - start_right.position.x)
      Y2 = numpy.square(w_right_y - start_right.position.y)
      Z2 = numpy.square(w_right_z - start_right.position.z)

      R = numpy.sqrt(X2 + Y2 + Z2)
      D2 = R / (N + 1)

      r1 = (w_right_x - start_right.position.x) / L
      r2 = (w_right_y - start_right.position.y) / L
      r3 = (w_right_z - start_right.position.z) / L

      waypoint_right = copy.deepcopy(start_right)
      i = 1
      while i <= N:
          x0 = start_right.position.x
          y0 = start_right.position.y
          z0 = start_right.position.z

          waypoint_right.position.x = x0 + r1 * i * D2
          waypoint_right.position.y = y0 + r2 * i * D2
          waypoint_right.position.z = z0 + r3 * i * D2

          waypoints_right.append(copy.deepcopy(waypoint_right))

          i = i + 1
      print "len_waypoints_right = ", len(waypoints_right)

      # 笛卡尔运动
      if cartesian:
          waypoints_left.append(copy.deepcopy(wpose_left))
          waypoints_right.append(copy.deepcopy(wpose_right))
          n = 0
          while True:
              fraction1 = 0.0
              fraction2 = 0.0
              i = 0
              j = 0
              while fraction1 < 1.0 and i < 10:
                  (plan_left, fraction1) = group_left.compute_cartesian_path(
                      waypoints_left,  # 路点列表
                      0.01,  # 终端步进值
                      0.0)  # 跳跃阈值
              while fraction2 < 1.0 and j < 10:
                  (plan_right, fraction2) = group_right.compute_cartesian_path(
                      waypoints_right,  # 路点列表
                      0.01,  # 终端步进值
                      0.0)  # 跳跃阈值
              # 左臂和右臂的points的数量
              len_left = len(plan_left.joint_trajectory.points)
              len_right = len(plan_right.joint_trajectory.points)

              if len_left != len_right:
                  print "left = ", len_left, "right = ", len_right
                  n = n + 1
                  while n == 20:
                      rospy.logerr("Dual_arm cartesian planning failure")
                      return 0
                  continue
              else:
                  print "left = ", len_left, "right = ", len_right
                  break
          print "n = ", n

      # 双臂的plan
      plan = group.plan()
      #print plan
      plan.joint_trajectory.joint_names = plan_left.joint_trajectory.joint_names + plan_right.joint_trajectory.joint_names
      i = 0
      while i < len_left:
          plan.joint_trajectory.points.append(plan_left.joint_trajectory.points[i])
          i = i + 1

      j = 0
      while j < len_left:
          plan.joint_trajectory.points[j].positions = plan_left.joint_trajectory.points[j].positions + \
                                                      plan_right.joint_trajectory.points[j].positions
          plan.joint_trajectory.points[j].velocities = plan_left.joint_trajectory.points[j].velocities + \
                                                       plan_right.joint_trajectory.points[j].velocities
          plan.joint_trajectory.points[j].accelerations = plan_left.joint_trajectory.points[j].accelerations + \
                                                          plan_right.joint_trajectory.points[j].accelerations
          j = j + 1


      # raw_input()
      print "fraction1 = ", fraction1, "fraction2 = ", fraction2
      print "left points = ", len_left
      print "right points = ", len_right
      print plan.joint_trajectory.points[1].velocities[0]

      # v1 = []
      # v2 = []
      # v3 = []
      # v4 = []
      # v5 = []
      # v6 = []
      # v7 = []
      #
      # a1 = []
      # a2 = []
      # a3 = []
      # a4 = []
      # a5 = []
      # a6 = []
      # a7 = []
      #
      # for j in range(len_left):
      #     v1.append(plan.joint_trajectory.points[j].velocities[0])
      #     v2.append(plan.joint_trajectory.points[j].velocities[1])
      #     v3.append(plan.joint_trajectory.points[j].velocities[2])
      #     v4.append(plan.joint_trajectory.points[j].velocities[3])
      #     v5.append(plan.joint_trajectory.points[j].velocities[4])
      #     v6.append(plan.joint_trajectory.points[j].velocities[5])
      #     v7.append(plan.joint_trajectory.points[j].velocities[6])
      #
      #     a1.append(plan.joint_trajectory.points[j].accelerations[0])
      #     a2.append(plan.joint_trajectory.points[j].accelerations[1])
      #     a3.append(plan.joint_trajectory.points[j].accelerations[2])
      #     a4.append(plan.joint_trajectory.points[j].accelerations[3])
      #     a5.append(plan.joint_trajectory.points[j].accelerations[4])
      #     a6.append(plan.joint_trajectory.points[j].accelerations[5])
      #     a7.append(plan.joint_trajectory.points[j].accelerations[6])
      # plt.plot(v1)
      # plt.plot(v2)
      # plt.plot(v3)
      # plt.plot(v4)
      # plt.plot(v5)
      # plt.plot(v6)
      # plt.plot(v7)
      # plt.grid(True)
      # plt.axis('tight')
      # plt.xlabel('points')
      # plt.ylabel('velocities')
      # plt.ylim(-3.5, 3.5)
      # # plt.xlim(0, 250)
      # plt.show()



      self.Display_Trajectory(plan)
      #raw_input()
      group.execute(plan, wait=True)
      print "========== STOP =========="

  # 采样点测试逆解
  def sample_arm_pose_control(self, group_name, pos, rot):
    if group_name == "left_arm":
        group = self.group_left_arm
    elif group_name == "right_arm":
        group = self.group_right_arm
    group.set_planner_id("RRTConnectkConfigDefault")
    group.set_pose_reference_frame("/base_link")

    # 获取末端执行器当前姿态
    group_end_effector_link = group.get_end_effector_link()
    # 设置目标点末端执行器的位姿
    target_pose = geometry_msgs.msg.PoseStamped()
    target_pose.header.frame_id = 'base_link'
    target_pose.pose.position.x = pos[0]
    target_pose.pose.position.y = pos[1]
    target_pose.pose.position.z = pos[2]
    quaternion = transformations.quaternion_from_euler(rot[0], rot[1], rot[2])
    target_pose.pose.orientation.x = quaternion[0]
    target_pose.pose.orientation.y = quaternion[1]
    target_pose.pose.orientation.z = quaternion[2]
    target_pose.pose.orientation.w = quaternion[3]

    group.set_pose_target(target_pose)
    plan = group.plan()
    #group.execute(plan, wait=True)
    print "plan:", plan.joint_trajectory.joint_names, "\n"
    if plan.joint_trajectory.joint_names == []:
        return 0
    else:
        return 1

    #raw_input()
    #group.execute(plan)

  # 采样点测试逆解（笛卡尔）
  def sample_Plan_Cartesian_Path(self, group_name, pose, scale=1):
      # 是否需要使用笛卡尔运动
      cartesian = rospy.get_param('~cartesian', True)

      if group_name == "left_arm":
          group = self.group_left_arm
          start_pose = group.get_current_pose("L_ee").pose
      elif group_name == "right_arm":
          group = self.group_right_arm
          start_pose = group.get_current_pose("R_ee").pose
      group.set_pose_reference_frame("/base_link")
      # 设置位置和姿态的允许误差
      group.set_goal_position_tolerance(0.01)
      group.set_goal_orientation_tolerance(0.05)

      waypoints = []
      # 路点数据
      wpose = start_pose
      wpose.position.x = scale * (pose[0])
      wpose.position.y = scale * (pose[1])
      wpose.position.z = scale * (pose[2])

      # 笛卡尔运动
      if cartesian:
          # waypoints.append(copy.deepcopy(start_pose))
          waypoints.append(copy.deepcopy(wpose))

          fraction = 0.0
          i = 0
          (plan, fraction) = group.compute_cartesian_path(
              waypoints,  # 路点列表
              0.01,  # 终端步进值
              0.0)  # 跳跃阈值
          print 'fraction = ', fraction
          # self.Display_Trajectory(plan)
          # #raw_input()
          # group.execute(plan, wait=True)
      if fraction < 1.0:
          return 0
      else:
          return 1

  # 轨迹修改（速度）(revise_trajetory)
  def trajectory_change(self, traj, scale):

      # 创建一个新的轨迹
      new_traj = self.new_traj

      # Initialize the new trajectory to be the same as the input trajectory
      new_traj.joint_trajectory = traj.joint_trajectory

      # 获取轨迹的关节数和路点数
      n_joints = len(traj.joint_trajectory.joint_names)
      n_points = len(traj.joint_trajectory.points)

      # 存储轨迹点
      points = list(traj.joint_trajectory.points)

      # 将轨迹路点信息赋给新的轨迹 # Cycle through all points and joints and scale the time from start,
      # as well as joint speed and acceleration
      for i in range(n_points):
          point = JointTrajectoryPoint()

          point.positions = traj.joint_trajectory.points[i].positions
          point.time_from_start = traj.joint_trajectory.points[i].time_from_start / scale
          point.velocities = list(traj.joint_trajectory.points[i].velocities)
          point.accelerations = list(traj.joint_trajectory.points[i].accelerations)

          # 设置速度比例因子
          for j in range(n_joints):
              point.velocities[j] = point.velocities[j] * scale
              point.accelerations[j] = point.accelerations[j] * scale * scale

          points[i] = point

      new_traj.joint_trajectory.points = points

      #print new_traj.joint_trajectory.points[5].velocities
      #return new_traj

  # 设置速度 
  def set_trajectory_speed(self, traj, speed):

      # 创建一个新的轨迹
      new_traj = self.new_traj
      new_traj.joint_trajectory = traj.joint_trajectory
      # 获取轨迹的关节数和路点数
      n_joints = len(traj.joint_trajectory.joint_names)
      n_points = len(traj.joint_trajectory.points)

      # 存储轨迹点
      points = list(traj.joint_trajectory.points)

      # 将轨迹路点信息赋给新的轨迹
      for i in range(n_points):
          point = JointTrajectoryPoint()

          point.positions = traj.joint_trajectory.points[i].positions
          point.time_from_start = traj.joint_trajectory.points[i].time_from_start

          # 设置速度和加速度
          point.velocities = [speed] * n_joints
          point.accelerations = [speed / 4.0] * n_joints
          points[i] = point

      new_traj.joint_trajectory.points = points

  def dual_arm_pose_control_constraints(self, pos_left, rot_left, pos_right, rot_right):
    group = self.group_dual_arm
    group.set_planner_id("RRTConnect")
    #group.set_planner_id("RRTstarkConfigDefault")
    group.set_planning_time(5)
    group.set_pose_reference_frame("/base_link")

    group.allow_replanning(True)

    target_pose_left = geometry_msgs.msg.PoseStamped()
    target_pose_right = geometry_msgs.msg.PoseStamped()
    target_pose_left.header.frame_id = 'base_link'
    target_pose_left.pose.position.x = pos_left[0]
    target_pose_left.pose.position.y = pos_left[1]
    target_pose_left.pose.position.z = pos_left[2]
    quaternion = transformations.quaternion_from_euler(rot_left[0], rot_left[1], rot_left[2])
    target_pose_left.pose.orientation.x = quaternion[0]
    target_pose_left.pose.orientation.y = quaternion[1]
    target_pose_left.pose.orientation.z = quaternion[2]
    target_pose_left.pose.orientation.w = quaternion[3]

    target_pose_right.header.frame_id = 'base_link'
    target_pose_right.pose.position.x = pos_right[0]
    target_pose_right.pose.position.y = pos_right[1]
    target_pose_right.pose.position.z = pos_right[2]
    quaternion = transformations.quaternion_from_euler(rot_right[0], rot_right[1], rot_right[2])
    target_pose_right.pose.orientation.x = quaternion[0]
    target_pose_right.pose.orientation.y = quaternion[1]
    target_pose_right.pose.orientation.z = quaternion[2]
    target_pose_right.pose.orientation.w = quaternion[3]

    group.set_pose_target(target_pose_left, "L_ee")
    group.set_pose_target(target_pose_right, "R_ee")

    # 设置方向约束
    constraints = moveit_msgs.msg.Constraints()
    orientation_constraints = moveit_msgs.msg.OrientationConstraint()
    orientation_constraints.header = target_pose_left.header
    orientation_constraints.link_name = "L_ee"
    orientation_constraints.orientation.x = 0.5
    orientation_constraints.orientation.y = 0.5
    orientation_constraints.orientation.z = 0.5
    orientation_constraints.orientation.w = 0.5
    orientation_constraints.absolute_x_axis_tolerance = 0.1
    orientation_constraints.absolute_y_axis_tolerance = 0.1
    orientation_constraints.absolute_z_axis_tolerance = 0.1
    orientation_constraints.weight = 1.0
    constraints.orientation_constraints.append(orientation_constraints)
    group.set_path_constraints(constraints)
    # constraints = moveit_msgs.msg.Constraints()
    # position_constraints = moveit_msgs.msg.PositionConstraint()
    # position_constraints.header = target_pose_left.header
    # position_constraints.link_name = "L_ee"
    # position_constraints.target_point_offset.x =
    # position_constraints.target_point_offset.y =
    # position_constraints.constraint_region
    # position_constraints.weight = 1
    # a = moveit_msgs.msg.BoundingVolume()


    # 设置目标点末端执行器的位姿
    plan = group.plan()
    #raw_input()
    group.execute(plan)

    pnum = len(plan.joint_trajectory.points)
    print "points = ", pnum
    #print plan

    group.clear_pose_target("L_ee")
    group.clear_pose_target("R_ee")


  def rot_from_quaternoin(self, quaternion):
    # 由四元数转旋转矩阵得到的结果
    trans_matrix = transformations.quaternion_matrix(quaternion)

    # 从旋转矩阵中提取姿态矩阵
    rot_matrix = numpy.array([[trans_matrix[0][0], trans_matrix[0][1], trans_matrix[0][2]],
                                  [trans_matrix[1][0], trans_matrix[1][1], trans_matrix[1][2]],
                                  [trans_matrix[2][0], trans_matrix[2][1], trans_matrix[2][2]]])
    return rot_matrix

  # 变换矩阵
  def transform_array(self, a, alpha, d, theta):
    return numpy.array([[cos(theta), -sin(theta), 0, a],
                  [sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                  [sin(theta)*sin(alpha), cos(theta)*sin(alpha), cos(alpha), cos(alpha)*d],
                  [0, 0, 0, 1]])

  # 旋转矩阵
  def Rotation_martix(self, alpha, theta):
    return numpy.array([[cos(theta), -sin(theta), 0],
                  [sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha)],
                  [sin(theta)*sin(alpha), cos(theta)*sin(alpha), cos(alpha)]])

  # 正运动学方程
  def forward_kinematics(self, theta):
      # Mod_DH参数
      a1 = 0
      a2 = 0
      a3 = 0
      a4 = 0
      a5 = 0
      a6 = 0
      a7 = 0

      alpha1 = 0
      alpha2 = -1.57
      alpha3 = 1.57
      alpha4 = -1.57
      alpha5 = 1.57
      alpha6 = -1.57
      alpha7 = 1.57

      d1 = numpy.linalg.norm([0.000, 0.052, 0.164])
      d2 = 0
      d3 = numpy.linalg.norm([0.004, -0.292, -0.002])
      d4 = 0
      d5 = numpy.linalg.norm([0.003, -0.242, -0.015])
      d6 = 0
      d7 = numpy.linalg.norm([0.003, -0.234, -0.035])

      T1 = self.transform_array(a1, alpha1, d1, theta[0])
      T2 = self.transform_array(a2, alpha2, d2, theta[1])
      T3 = self.transform_array(a3, alpha3, d3, theta[2])
      T4 = self.transform_array(a4, alpha4, d4, theta[3])
      T5 = self.transform_array(a5, alpha5, d5, theta[4])
      T6 = self.transform_array(a6, alpha6, d6, theta[5])
      T7 = self.transform_array(a7, alpha7, d7, theta[6])

      T = T7
      T = numpy.dot(T6,T)
      T = numpy.dot(T5,T)
      T = numpy.dot(T4,T)
      T = numpy.dot(T3,T)
      T = numpy.dot(T2,T)
      T = numpy.dot(T1,T)

      print T
      return T

  # 肩关节正运动学方程
  def forward_kinematics_shouler(self, theta):
      # Mod_DH参数
      a1 = 0
      a2 = 0

      alpha1 = 0
      alpha2 = -1.57

      d1 = 0.164
      d2 = 0

      T1 = self.transform_array(a1, alpha1, d1, theta[0])
      T2 = self.transform_array(a2, alpha2, d2, theta[1])

      T = T2
      T = numpy.dot(T1, T)

      V_bs = numpy.array([[T[0][3]], [T[1][3]], [T[2][3]]])
      #print V_bs
      return V_bs

  # 肘关节正运动学方程
  def forward_kinematics_elbow(self, theta):
      # Mod_DH参数
      a1 = 0
      a2 = 0
      a3 = 0
      a4 = 0


      alpha1 = 0
      alpha2 = -1.57
      alpha3 = 1.57
      alpha4 = -1.57

      d1 = 0.164
      d2 = 0
      d3 = 0.292
      d4 = 0

      T1 = self.transform_array(a1, alpha1, d1, theta[0])
      T2 = self.transform_array(a2, alpha2, d2, theta[1])
      T3 = self.transform_array(a3, alpha3, d3, theta[2])
      T4 = self.transform_array(a4, alpha4, d4, theta[3])

      T = T4
      T = numpy.dot(T3,T)
      T = numpy.dot(T2,T)
      T = numpy.dot(T1,T)

      V_be = numpy.array([[T[0][3]], [T[1][3]], [T[2][3]]])
      #print T
      #print V_be
      return V_be

  # 腕关节正运动学方程
  def forward_kinematics_wrist(self, theta):
      # Mod_DH参数
      a1 = 0
      a2 = 0
      a3 = 0
      a4 = 0
      a5 = 0
      a6 = 0

      alpha1 = 0
      alpha2 = -1.57
      alpha3 = 1.57
      alpha4 = -1.57
      alpha5 = 1.57
      alpha6 = -1.57

      d1 = 0.164
      d2 = 0
      d3 = 0.292
      d4 = 0
      d5 = 0.242
      d6 = 0

      T1 = self.transform_array(a1, alpha1, d1, theta[0])
      T2 = self.transform_array(a2, alpha2, d2, theta[1])
      T3 = self.transform_array(a3, alpha3, d3, theta[2])
      T4 = self.transform_array(a4, alpha4, d4, theta[3])
      T5 = self.transform_array(a5, alpha5, d5, theta[4])
      T6 = self.transform_array(a6, alpha6, d6, theta[5])

      T = T6
      T = numpy.dot(T5,T)
      T = numpy.dot(T4,T)
      T = numpy.dot(T3,T)
      T = numpy.dot(T2,T)
      T = numpy.dot(T1,T)

      V_bw = numpy.array([[T[0][3]], [T[1][3]], [T[2][3]]])
      #print V_bw
      return V_bw

  # def Vector_shouler_elbow(self, theta):
  #     # Mod_DH参数
  #     a3 = 0
  #     a4 = 0
  #
  #     alpha3 = 1.57
  #     alpha4 = -1.57
  #
  #     d3 = numpy.linalg.norm([0.004, -0.292, -0.002])
  #     d4 = 0
  #
  #     T3 = self.transform_array(a3, alpha3, d3, theta[2])
  #     T4 = self.transform_array(a4, alpha4, d4, theta[3])
  #
  #     T = T4s
  #     T = numpy.dot(T3, T)
  #
  #     v_se = numpy.array([[T[0][3]], [T[1][3]], [T[2][3]]])
  #
  #     print T
  #     print v_se
  #     return v_se
  #
  # def Vector_shouler_wrist(self, theta):
  #     # Mod_DH参数
  #
  #     a3 = 0
  #     a4 = 0
  #     a5 = 0
  #     a6 = 0
  #
  #     alpha3 = 1.57
  #     alpha4 = -1.57
  #     alpha5 = 1.57
  #     alpha6 = -1.57
  #
  #     # d1 = 0.164
  #     # d2 = 0
  #     d3 = 0.292
  #     d4 = 0
  #     d5 = 0.242
  #     d6 = 0
  #
  #     T3 = self.transform_array(a3, alpha3, d3, theta[2])
  #     T4 = self.transform_array(a4, alpha4, d4, theta[3])
  #     T5 = self.transform_array(a5, alpha5, d5, theta[4])
  #     T6 = self.transform_array(a6, alpha6, d6, theta[5])
  #
  #     T = T6
  #     T = numpy.dot(T5, T)
  #     T = numpy.dot(T4, T)
  #     T = numpy.dot(T3, T)
  #
  #     v_sw = numpy.array([[T[0][3]], [T[1][3]], [T[2][3]]])
  #
  #     print v_sw
  #     return v_sw

  def arm_angle(self, group_name, joints):

      # 重新设置一个参考坐标系,以后所有的计算都以该坐标系为参考坐标系
      # 参考坐标系方向为x向前,y向左,z向上
      if group_name == "left_arm":
          T = transformations.euler_matrix(3.142, 1.561, -2.096)
          R = numpy.array([[T[0][0], T[0][1], T[0][2]],
                        [T[1][0], T[1][1], T[1][2]],
                        [T[2][0], T[2][1], T[2][2]]])
      elif group_name == "right_arm":
          T = transformations.euler_matrix(0.000, -1.562, 2.093)
          R = numpy.array([[T[0][0], T[0][1], T[0][2]],
                        [T[1][0], T[1][1], T[1][2]],
                        [T[2][0], T[2][1], T[2][2]]])

      # V_bs = self.forward_kinematics_shouler(joints)
      # V_be = self.forward_kinematics_elbow(joints)
      # V_bw = self.forward_kinematics_wrist(joints)
      #
      # V_se = V_be - V_bs
      # V_sw = V_bw - V_bs
      # V = numpy.array([[1],[0],[0]])     #向量[0, 0, 1]

      V_bs = numpy.dot(R, self.forward_kinematics_shouler(joints))
      V_be = numpy.dot(R, self.forward_kinematics_elbow(joints))
      V_bw = numpy.dot(R, self.forward_kinematics_wrist(joints))

      V_se = V_be - V_bs
      V_sw = V_bw - V_bs
      V = numpy.array([[0],[0],[-1]])     #向量[0, 0, -1]

      n_arm = numpy.cross(V_sw.reshape((-1)),V_se.reshape((-1)))    #手臂平面
      n_ref = numpy.cross(V_sw.reshape((-1)), V.reshape((-1)))      #参考平面

      # print V_se.reshape((-1))
      # print V_sw.reshape((-1))
      # print n_arm
      # print n_ref

      V_sw_unit = V_sw/numpy.linalg.norm(V_sw)   #向量sw的单位向量
      #
      arm_angle = math.atan2(-numpy.dot(numpy.cross(n_ref, n_arm), V_sw_unit), numpy.dot(n_ref, n_arm))
      #theta = math.acos(numpy.dot(n_ref, n_arm))

      print "arm_angle =", arm_angle
      return arm_angle

  # 旋转角
  def Rotation_axis(self, w, theta):
      # 反对称矩阵
      skew_symmetric_matrix = numpy.array([[0, -w[2], w[1]],
                                        [w[2], 0, -w[0]],
                                        [-w[1], w[0], 0]])
      unit_matrix = self.unit_matrix

      # 罗德格里斯公式
      Rodrigucs = unit_matrix + skew_symmetric_matrix * sin(theta) + numpy.dot(skew_symmetric_matrix,
                                                                            skew_symmetric_matrix) * (1 - cos(theta))

      return Rodrigucs

  # 运动旋量
  def Exp_twist(self, w, q, theta):
      unit_matrix = self.unit_matrix

      # 转化为行向量
      w_vector = w.reshape((-1))
      q_vector = q.reshape((-1))

      # 线速度
      v = -numpy.cross(w_vector, q_vector)

      rotation_axis = self.Rotation_axis(w, theta)

      # 矩阵转置
      w_trans = numpy.transpose(w)

      # 位置
      P1 = unit_matrix - rotation_axis
      P2 = numpy.cross(w_vector, v)
      P3 = numpy.dot(P1, P2)

      P4 = numpy.dot(w, w_trans)
      P5 = numpy.dot(P4, v)
      P6 = P5 * theta

      position = P3 + P6

      # rotation_axis[0][3] = P7[0]
      # 旋量公式
      exp_twist = numpy.array([[rotation_axis[0][0], rotation_axis[0][1], rotation_axis[0][2], position[0]],
                            [rotation_axis[1][0], rotation_axis[1][1], rotation_axis[1][2], position[1]],
                            [rotation_axis[2][0], rotation_axis[2][1], rotation_axis[2][2], position[2]],
                            [0, 0, 0, 1]])

      return exp_twist

  # 球关节逆解
  def SphericalJoint_inverse(self, R):
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
      #print thetas
      return thetas

  # 臂形角方法求逆
  def arm_angle_inverse(self, group_name, pos_from_b, rot_from_b, arm_angle):

      if group_name == "left_arm":
          l1 = 0.164
          l2 = 0.292
          l3 = 0.242
          l4 = 0.234
          T_base = transformations.euler_matrix(1.583, -0.525, -1.577)
          T_base = numpy.array([[T_base[0][0], T_base[0][1], T_base[0][2], 1.153],
                             [T_base[1][0], T_base[1][1], T_base[1][2], -0.278],
                             [T_base[2][0], T_base[2][1], T_base[2][2], 0.085],
                             [0, 0, 0, 1]])

          #末端相对于双臂基坐标系的位姿
          T_from_b = transformations.euler_matrix(rot_from_b[0], rot_from_b[1], rot_from_b[2])
          T_from_b = numpy.array([[T_from_b[0][0], T_from_b[0][1], T_from_b[0][2], pos_from_b[0]],
                               [T_from_b[1][0], T_from_b[1][1], T_from_b[1][2], pos_from_b[1]],
                               [T_from_b[2][0], T_from_b[2][1], T_from_b[2][2], pos_from_b[2]],
                               [0, 0, 0, 1]])

          #末端相对于手臂基座的位姿
          T = numpy.dot(T_base, T_from_b)
          pos = [T[0][3], T[1][3],T[2][3]]
          rot = transformations.euler_from_matrix(T)
          print "pos_from_Link0 =", pos
          print "rot_from_Link0 =", rot

          #参考平面选取
          V = numpy.array([[-1], [0], [0]])

      elif group_name == "right_arm":
          l1 = 0.174
          l2 = 0.292
          l3 = 0.242
          l4 = 0.234
          T_base = transformations.euler_matrix(-1.561, -0.522, -1.576)
          T_base = numpy.array([[T_base[0][0], T_base[0][1], T_base[0][2], -1.157],
                             [T_base[1][0], T_base[1][1], T_base[1][2], -0.282],
                             [T_base[2][0], T_base[2][1], T_base[2][2], 0.066],
                             [0, 0, 0, 1]])

          #末端相对于双臂基坐标系的位姿
          T_from_b = transformations.euler_matrix(rot_from_b[0], rot_from_b[1], rot_from_b[2])
          T_from_b = numpy.array([[T_from_b[0][0], T_from_b[0][1], T_from_b[0][2], pos_from_b[0]],
                               [T_from_b[1][0], T_from_b[1][1], T_from_b[1][2], pos_from_b[1]],
                               [T_from_b[2][0], T_from_b[2][1], T_from_b[2][2], pos_from_b[2]],
                               [0, 0, 0, 1]])

          #末端相对于手臂基座的位姿
          T = numpy.dot(T_base, T_from_b)
          pos = [T[0][3], T[1][3],T[2][3]]
          rot = transformations.euler_from_matrix(T)
          print "pos_from_Rink0 =", pos
          print "rot_from_Rink0 =", rot

          # 参考平面选取
          V = numpy.array([[1], [0], [0]])

      #所有向量,几何关系的参考系均为手臂基座
      T = transformations.euler_matrix(rot[0], rot[1], rot[2])
      R = numpy.array([[T[0][0], T[0][1], T[0][2]],
                    [T[1][0], T[1][1], T[1][2]],
                    [T[2][0], T[2][1], T[2][2]]])

      V_bs = numpy.array([[0],[0],[l1]])
      #V_tw_from_t = numpy.array([[0],[0],[-l4]])
      V_wt_from_t = numpy.array([[0],[0],[l4]])
      V_wt = numpy.dot(R, V_wt_from_t)

      V_bt = numpy.array([[pos[0]],[pos[1]],[pos[2]]])
      V_bw = V_bt - V_wt
      V_sw = V_bt - V_bs - V_wt
      L_sw = numpy.linalg.norm(V_sw)          #向量sw的模

      beta = math.acos((math.pow(l2, 2) + math.pow(l3, 2) - math.pow(L_sw, 2))/(2*l2*l3))

      # 肘关节角
      elbow_theta = []
      elbow_theta.append(math.pi - beta)
      elbow_theta.append(-math.pi + beta)

      # if arm_angle > 0:
      #     print elbow_theta
      # else:
      #     elbow_theta = -elbow_theta
      #     print elbow_theta

      alpha = math.acos((math.pow(l2, 2)  + math.pow(L_sw, 2) - math.pow(l3, 2))/(2*l2*L_sw))

      #肘关节中心位置
      n_ref = numpy.cross(V_sw.reshape((-1)), V.reshape((-1)))  # 参考平面

      n_ref_unit = n_ref/numpy.linalg.norm(n_ref)   #单位向量
      V_sw_unit = V_sw/numpy.linalg.norm(V_sw)

      q = numpy.array([[0], [0], [0]])
      exp_ref = self.Exp_twist(-n_ref_unit, q, alpha)     #旋量公式,绕参考平面的法向量旋转alpha角
      exp_sw = self.Exp_twist(-V_sw_unit, q, arm_angle)     #旋量公式,绕sw旋转arm_angle角

      P_e = numpy.dot(exp_sw, exp_ref)
      P_e = numpy.dot(P_e, [[l2*V_sw_unit[0]], [l2*V_sw_unit[1]], [l2*V_sw_unit[2]], [1]])
      P_e = numpy.array([P_e[0], P_e[1], [P_e[2][0] + l1]])   #算出来的e的位置相对于Link2,要转换为相对于Link_0
      print "肘关节位置 =", P_e

      #肘关节中心姿态
      V_se = P_e - V_bs
      W_e = numpy.cross(V_se.reshape((-1)), V_sw.reshape((-1)))  #方向向量
      W_e = W_e/numpy.linalg.norm(W_e)

      V_se_unit = V_se/numpy.linalg.norm(V_se)

      # R_e = numpy.array([[V_se_unit[0][0], numpy.cross(W_e.reshape((-1)), V_se_unit.reshape((-1)))[0], W_e[0]],
      #                   [V_se_unit[1][0], numpy.cross(W_e.reshape((-1)), V_se_unit.reshape((-1)))[1], W_e[1]],
      #                   [V_se_unit[2][0], numpy.cross(W_e.reshape((-1)), V_se_unit.reshape((-1)))[2], W_e[2]]])

      if group_name == "left_arm":
          R_e = numpy.array([[-numpy.cross(W_e.reshape((-1)), V_se_unit.reshape((-1)))[0], -W_e[0], V_se_unit[0][0]],
                            [-numpy.cross(W_e.reshape((-1)), V_se_unit.reshape((-1)))[1], -W_e[1], V_se_unit[1][0]],
                            [-numpy.cross(W_e.reshape((-1)), V_se_unit.reshape((-1)))[2], -W_e[2], V_se_unit[2][0]]])
      elif group_name == "right_arm":
          R_e = numpy.array([[numpy.cross(W_e.reshape((-1)), V_se_unit.reshape((-1)))[0], W_e[0], V_se_unit[0][0]],
                            [numpy.cross(W_e.reshape((-1)), V_se_unit.reshape((-1)))[1], W_e[1], V_se_unit[1][0]],
                            [numpy.cross(W_e.reshape((-1)), V_se_unit.reshape((-1)))[2], W_e[2], V_se_unit[2][0]]])

      print "肘关节姿态 =", transformations.euler_from_matrix(R_e)              #第3个关节的姿态
      R_e2 = R_e

      # 第一个球关节的关节角
      shouler_theta = self.SphericalJoint_inverse(R_e)

      R_e = numpy.dot(R_e, self.Rotation_martix(-1.5708, elbow_theta[0]))
      #print transformations.euler_from_matrix(self.Rotation_martix(-1.5708, elbow_theta))
      print "肘关节姿态2 =", transformations.euler_from_matrix(R_e)    #第4个关节的姿态

      R_e_inv = numpy.linalg.inv(R_e)
      R_t_from_e = numpy.dot(R_e_inv, R)                             #末端相对于肘关节的姿态
      print transformations.euler_from_matrix(R_t_from_e)

      T_e_init = transformations.euler_matrix(1.571, -0.000, 0.000)
      R_e_init = numpy.array([[T_e_init[0][0], T_e_init[0][1], T_e_init[0][2]],
                    [T_e_init[1][0], T_e_init[1][1], T_e_init[1][2]],
                    [T_e_init[2][0], T_e_init[2][1], T_e_init[2][2]]])   #初始状态末端相对于肘关节的姿态

      R_t_from_e = numpy.dot(numpy.linalg.inv(R_e_init), R_t_from_e)     #末端相对于肘关节的旋转变换
      # print R_w_from_e
      # print transformations.euler_from_matrix(R_t_from_e)

      # 第二个球关节的关节角
      wrist_theta = self.SphericalJoint_inverse(R_t_from_e)
      #print wrist_theta

      #7个关节角
      theta1 = []
      theta2 = []
      theta3 = []
      theta4 = []

      theta1.append(shouler_theta[0][0])
      theta1.append(shouler_theta[0][1])
      theta1.append(shouler_theta[0][2])
      theta1.append(elbow_theta[0])
      theta1.append(wrist_theta[0][0])
      theta1.append(wrist_theta[0][1])
      theta1.append(wrist_theta[0][2])

      theta2.append(shouler_theta[1][0])
      theta2.append(shouler_theta[1][1])
      theta2.append(shouler_theta[1][2])
      theta2.append(elbow_theta[0])
      theta2.append(wrist_theta[0][0])
      theta2.append(wrist_theta[0][1])
      theta2.append(wrist_theta[0][2])

      theta3.append(shouler_theta[0][0])
      theta3.append(shouler_theta[0][1])
      theta3.append(shouler_theta[0][2])
      theta3.append(elbow_theta[0])
      theta3.append(wrist_theta[1][0])
      theta3.append(wrist_theta[1][1])
      theta3.append(wrist_theta[1][2])

      theta4.append(shouler_theta[1][0])
      theta4.append(shouler_theta[1][1])
      theta4.append(shouler_theta[1][2])
      theta4.append(elbow_theta[0])
      theta4.append(wrist_theta[1][0])
      theta4.append(wrist_theta[1][1])
      theta4.append(wrist_theta[1][2])

      R_e = numpy.dot(R_e2, self.Rotation_martix(-1.5708, elbow_theta[1]))
      # print transformations.euler_from_matrix(self.Rotation_martix(-1.5708, elbow_theta))
      print "肘关节姿态2 =", transformations.euler_from_matrix(R_e)  # 第4个关节的姿态

      R_e_inv = numpy.linalg.inv(R_e)
      R_t_from_e = numpy.dot(R_e_inv, R)  # 末端相对于肘关节的姿态
      print transformations.euler_from_matrix(R_t_from_e)

      T_e_init = transformations.euler_matrix(1.571, -0.000, 0.000)
      R_e_init = numpy.array([[T_e_init[0][0], T_e_init[0][1], T_e_init[0][2]],
                              [T_e_init[1][0], T_e_init[1][1], T_e_init[1][2]],
                              [T_e_init[2][0], T_e_init[2][1], T_e_init[2][2]]])  # 初始状态末端相对于肘关节的姿态

      R_t_from_e = numpy.dot(numpy.linalg.inv(R_e_init), R_t_from_e)  # 末端相对于肘关节的旋转变换
      # print R_w_from_e
      # print transformations.euler_from_matrix(R_t_from_e)

      # 第二个球关节的关节角
      wrist_theta = self.SphericalJoint_inverse(R_t_from_e)
      # print wrist_theta

      theta5 = []
      theta6 = []
      theta7 = []
      theta8 = []

      theta5.append(shouler_theta[0][0])
      theta5.append(shouler_theta[0][1])
      theta5.append(shouler_theta[0][2])
      theta5.append(elbow_theta[1])
      theta5.append(wrist_theta[0][0])
      theta5.append(wrist_theta[0][1])
      theta5.append(wrist_theta[0][2])

      theta6.append(shouler_theta[1][0])
      theta6.append(shouler_theta[1][1])
      theta6.append(shouler_theta[1][2])
      theta6.append(elbow_theta[1])
      theta6.append(wrist_theta[0][0])
      theta6.append(wrist_theta[0][1])
      theta6.append(wrist_theta[0][2])

      theta7.append(shouler_theta[0][0])
      theta7.append(shouler_theta[0][1])
      theta7.append(shouler_theta[0][2])
      theta7.append(elbow_theta[1])
      theta7.append(wrist_theta[1][0])
      theta7.append(wrist_theta[1][1])
      theta7.append(wrist_theta[1][2])

      theta8.append(shouler_theta[1][0])
      theta8.append(shouler_theta[1][1])
      theta8.append(shouler_theta[1][2])
      theta8.append(elbow_theta[1])
      theta8.append(wrist_theta[1][0])
      theta8.append(wrist_theta[1][1])
      theta8.append(wrist_theta[1][2])

      thetas = []
      thetas.append(theta1)
      thetas.append(theta2)
      thetas.append(theta3)
      thetas.append(theta4)
      thetas.append(theta5)
      thetas.append(theta6)
      thetas.append(theta7)
      thetas.append(theta8)

      print ""
      print "*****所有的关节角*****"
      print ""
      print "theta1 =", theta1
      print "theta2 =", theta2
      print "theta3 =", theta3
      print "theta4 =", theta4
      print "theta5 =", theta5
      print "theta6 =", theta6
      print "theta7 =", theta7
      print "theta8 =", theta8

      return thetas


  def arm_angle_inverse_test(self,  arm_angle):

      V_sw = [-0.109, -0.204, -0.197]
      l2 = 0.292
      l3 = 0.242

      T = transformations.euler_matrix(-0.056, 0.272, 2.376)
      R = numpy.array([[T[0][0], T[0][1], T[0][2]],
                       [T[1][0], T[1][1], T[1][2]],
                       [T[2][0], T[2][1], T[2][2]]])

      V_sw = numpy.dot(R, V_sw)
      print "V_sw =", V_sw
      L_sw = numpy.linalg.norm(V_sw)
      print "L_sw =", L_sw
      alpha = math.acos((math.pow(l2, 2) + math.pow(L_sw, 2) - math.pow(l3, 2)) / (2 * l2 * L_sw))
      print "alpha =", alpha

      V = numpy.array([[0], [0], [1]])
      n_ref = numpy.cross(V_sw.reshape((-1)), V.reshape((-1)))  # 参考平面

      n_ref_unit = n_ref / numpy.linalg.norm(n_ref)
      V_sw_unit = V_sw / numpy.linalg.norm(V_sw)

      q = numpy.array([[0], [0], [0]])
      exp_ref = self.Exp_twist(-n_ref_unit, q, alpha)
      exp_sw = self.Exp_twist(-V_sw_unit, q, arm_angle)

      P_e = numpy.dot(exp_sw, exp_ref)
      P_e = numpy.dot(P_e, [[l2 * V_sw_unit[0]], [l2 * V_sw_unit[1]], [l2 * V_sw_unit[2]], [1]])


      # 肘关节中心姿态
      V_se = numpy.array([P_e[0], P_e[1], [P_e[2]]])
      print P_e
      print V_sw
      W_e = numpy.cross(V_se.reshape((-1)), V_sw.reshape((-1)))  # 方向向量
      W_e = W_e / numpy.linalg.norm(W_e)

      V_se_unit = V_se / numpy.linalg.norm(V_se)

      print "*******"
      print V_se
      print V_se_unit
      print numpy.cross(W_e.reshape((-1)), V_se_unit.reshape((-1)))
      print W_e

      R_e = numpy.array([[numpy.cross(W_e.reshape((-1)), V_se_unit.reshape((-1)))[0], W_e[0], V_se_unit[0][0]],
                        [numpy.cross(W_e.reshape((-1)), V_se_unit.reshape((-1)))[1], W_e[1], V_se_unit[1][0]],
                        [numpy.cross(W_e.reshape((-1)), V_se_unit.reshape((-1)))[2], W_e[2], V_se_unit[2][0]]])

      print "*******"
      print R_e

      R_e = transformations.euler_from_matrix(R_e)
      print R_e
      print transformations.euler_matrix(1.494, -0.790, 2.445)

  # 逆运动学方程
  def inverse_kinematics(self, group_name, pos, rot):
      if group_name == "left_arm":
          group = self.group_left_arm
      elif group_name == "right_arm":
          group = self.group_right_arm
      group.set_planner_id("RRTConnectkConfigDefault")
      group.set_pose_reference_frame("/base_link")

      # 获取末端执行器当前姿态
      group_end_effector_link = group.get_end_effector_link()
      # 设置目标点末端执行器的位姿
      target_pose = geometry_msgs.msg.PoseStamped()
      target_pose.header.frame_id = 'base_link'
      target_pose.pose.position.x = pos[0]
      target_pose.pose.position.y = pos[1]
      target_pose.pose.position.z = pos[2]
      quaternion = transformations.quaternion_from_euler(rot[0], rot[1], rot[2])
      target_pose.pose.orientation.x = quaternion[0]
      target_pose.pose.orientation.y = quaternion[1]
      target_pose.pose.orientation.z = quaternion[2]
      target_pose.pose.orientation.w = quaternion[3]

      group.set_pose_target(target_pose)
      plan = group.plan()
      n_points = len(plan.joint_trajectory.points)
      #return plan.joint_trajectory.points[n_points - 1].positions
      return plan.joint_trajectory.points[n_points - 1]


  # 双臂协调
  def dual_arm_coordinate(self, pos_left, rot_left, pos_r_from_l, rot_r_from_l):
    # 左臂基坐标系相对于base_link的变换矩阵
    T_L0_from_base = numpy.array([[0, -0.865, 0.5017, -0.278],
                 [0, 0.5017, 0.865, 0.076],
                 [-1, 0, 0, 1.155],
                 [0, 0, 0, 1]])
    # 右臂基坐标系相对于base_link的变换矩阵
    T_R0_from_base = numpy.array([[0, -0.866, 0.4988, -0.278],
                 [0, -0.4988, -0.8668, -0.076],
                 [1, 0, 0, 1.155],
                 [0, 0, 0, 1]])

    # 从臂相对于主臂的变换矩阵
    T_R_from_L = transformations.euler_matrix(rot_r_from_l[0], rot_r_from_l[1], rot_r_from_l[2])
    T_R_from_L[0][3] = pos_r_from_l[0]
    T_R_from_L[1][3] = pos_r_from_l[1]
    T_R_from_L[2][3] = pos_r_from_l[2]

    group = self.group_dual_arm
    group_left = self.group_left_arm
    group_right = self.group_right_arm

    #start_left = group_left.get_current_pose("L_ee").pose
    #start_right = group_right.get_current_pose("R_ee").pose

    # 设置位置和姿态的允许误差
    group_left.set_goal_position_tolerance(0.01)
    group_left.set_goal_orientation_tolerance(0.05)

    group_left.set_planner_id("RRTConnectkConfigDefault")
    group_left.set_pose_reference_frame("/base_link")

    # 获取末端执行器当前姿态
    #group_end_effector_link = group_left.get_end_effector_link()
    # 设置目标点末端执行器的位姿
    target_pose_left = geometry_msgs.msg.PoseStamped()
    target_pose_left.header.frame_id = 'base_link'
    target_pose_left.pose.position.x = pos_left[0]
    target_pose_left.pose.position.y = pos_left[1]
    target_pose_left.pose.position.z = pos_left[2]
    quaternion = transformations.quaternion_from_euler(rot_left[0], rot_left[1], rot_left[2])
    target_pose_left.pose.orientation.x = quaternion[0]
    target_pose_left.pose.orientation.y = quaternion[1]
    target_pose_left.pose.orientation.z = quaternion[2]
    target_pose_left.pose.orientation.w = quaternion[3]

    # 规划左臂的轨迹
    group_left.set_pose_target(target_pose_left)
    plan_left = group_left.plan()

    n_joints = len(plan_left.joint_trajectory.joint_names)
    n_points = len(plan_left.joint_trajectory.points)
    print "n_points = ", n_points

  # 计算出每一个路点左臂末端位姿
    # 左臂路点列表
    joints_left = []
    # 左臂末端轨迹路点列表(相对左臂基坐标系)
    end_left_from_L0 = []
    # 左臂末端轨迹路点列表
    end_left = []
    # 右臂末端轨迹路点列表
    end_right = []
    # 右臂路点列表
    joints_right = []
    # 右臂速度列表
    joints_velocities_right = []
    # 右臂加速度列表
    joints_accelerations_right = []

    # 右臂位置和姿态
    for i in range(n_points):
        #for j in range(n_joints):
        joints_left.append([plan_left.joint_trajectory.points[i].positions[0],
                           plan_left.joint_trajectory.points[i].positions[1],
                           plan_left.joint_trajectory.points[i].positions[2],
                           plan_left.joint_trajectory.points[i].positions[3],
                           plan_left.joint_trajectory.points[i].positions[4],
                           plan_left.joint_trajectory.points[i].positions[5],
                           plan_left.joint_trajectory.points[i].positions[6],])
        #print "joint_left [", i ,"] = ", joint_left[i]
        end_left_from_L0.append(self.forward_kinematics(joints_left[i]))

        end_left.append(numpy.dot(T_L0_from_base, end_left_from_L0[i]))
        #print end_left[i]

        end_right.append(numpy.dot(end_left[i], T_R_from_L))

        # 右臂末端位置和姿态
        pos_right = []
        rot_right = transformations.euler_from_matrix(end_right[i])
        for j in range(3):
            pos_right.append(end_right[i][j][3])

        #print "point_left", i, " = ",(end_left[i][0][3], end_left[i][1][3], end_left[i][2][3])
        #print "point_right", i, " = ",pos_right, "\n"

        # 逆解求右臂关节角
        joint_right = self.inverse_kinematics("right_arm", pos_right, rot_right).positions
        joints_right.append(joint_right)

        # 右臂速度和加速度
        joint_velocities_right = self.inverse_kinematics("right_arm", pos_right, rot_right).velocities
        joints_velocities_right.append(joint_velocities_right)
        joint_accelerations_right = self.inverse_kinematics("right_arm", pos_right, rot_right).accelerations
        joints_accelerations_right.append(joint_accelerations_right)
    #print joint_right
    print joints_left[0]
    print end_left_from_L0[0]

    # 双臂运动轨迹
    dual_arm_traj = moveit_commander.RobotTrajectory()

    s = group_right.get_current_joint_values()
    joint_right_start = (s[0], s[1], s[2], s[3], s[4], s[5], s[6])

    joint_names_right = ['R_joint1', 'R_joint2', 'R_joint3', 'R_joint4', 'R_joint5', 'R_joint6', 'R_joint7']
    # for i in range(n_points):
    #     joint_positions_right.append((i*3.14/180, i*3.14/180, i*3.14/180, i*3.14/180, i*3.14/180, i*3.14/180,
    #                                   i*3.14/180))
    joint_velocities_right_start = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    joint_accelerations_right_start = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    dual_arm_traj.joint_trajectory.joint_names = plan_left.joint_trajectory.joint_names + joint_names_right
    dual_arm_traj.joint_trajectory.points.append(plan_left.joint_trajectory.points[0])
    dual_arm_traj.joint_trajectory.points[0].positions = (plan_left.joint_trajectory.points[0].positions +
                                                          joint_right_start)
    dual_arm_traj.joint_trajectory.points[0].velocities = (plan_left.joint_trajectory.points[0].velocities +
                                                           joint_velocities_right_start)
    dual_arm_traj.joint_trajectory.points[0].accelerations = (plan_left.joint_trajectory.points[0].accelerations +
                                                              joint_accelerations_right_start)

    i = 1
    while i < n_points:
        dual_arm_traj.joint_trajectory.points.append(plan_left.joint_trajectory.points[i])
        dual_arm_traj.joint_trajectory.points[i].positions = (plan_left.joint_trajectory.points[i].positions +
                                                              joints_right[i])
        dual_arm_traj.joint_trajectory.points[i].velocities = (plan_left.joint_trajectory.points[i].velocities +
                                                               joints_velocities_right[i])
        dual_arm_traj.joint_trajectory.points[i].accelerations = (plan_left.joint_trajectory.points[i].accelerations +
                                                                  joints_accelerations_right[i])
        i = i + 1

    rospy.sleep(1)
    print dual_arm_traj.joint_trajectory
    group.execute(dual_arm_traj)

  # 双臂协调(双臂跟随物体)
  def dual_arm_coordinate2(self, pos_obj, n):
    # 左臂末端相对于物体的变换矩阵
    T_L_from_obj = numpy.array([[0, 0, 1, 0],
                                [1, 0, 0, 0.120],
                                [0, 1, 0, 0],
                                [0, 0, 0, 1]])
    # 右臂末端相对于物体的变换矩阵
    T_R_from_obj = numpy.array([[0, 0, 1, 0],
                                [1, 0, 0, -0.120],
                                [0, 1, 0, 0],
                                [0, 0, 0, 1]])

    # 物体相对于基座标系的变换矩阵
    T_obj_from_base = numpy.array([[1, 0, 0, 0.95],
                                [0, 1, 0, 0],
                                [0, 0, 1, 1.02],
                                [0, 0, 0, 1]])

    # 左臂关节角列表
    joints_left = []

    # 右臂关节角列表
    joints_right = []

    # 双臂轨迹
    group = self.group_dual_arm
    dual_arm_traj = moveit_commander.RobotTrajectory()

    joint_names = ['L_Joint1', 'L_Joint2', 'L_Joint3', 'L_Joint4', 'L_Joint5', 'L_Joint6', 'L_Joint7',
                   'R_Joint1', 'R_Joint2', 'R_Joint3', 'R_Joint4', 'R_Joint5', 'R_Joint6', 'R_Joint7']
    dual_arm_traj.joint_trajectory.joint_names = joint_names

    for i in range(n):
        # 物体轨迹
        T_obj_from_base[2][3] = pos_obj[2]+0.01*i
        print T_obj_from_base[2][3]

        # 左臂和右臂相对于基坐标系的变换矩阵
        T_L_from_B = numpy.dot(T_obj_from_base, T_L_from_obj)
        T_R_from_B = numpy.dot(T_obj_from_base, T_R_from_obj)

        pos_left = [T_L_from_B[0][3], T_L_from_B[1][3], T_L_from_B[2][3]]
        rot_left = transformations.euler_from_matrix(T_L_from_B)
        pos_right = [T_R_from_B[0][3], T_R_from_B[1][3], T_R_from_B[2][3]]
        rot_right = transformations.euler_from_matrix(T_R_from_B)

        joint_left = self.inverse_kinematics("left_arm", pos_left, rot_left).positions
        joints_left.append(joint_left)

        joint_right = self.inverse_kinematics("right_arm", pos_right, rot_right).positions
        joints_right.append(joint_right)

        # 左臂和右臂轨迹数据
        dual_arm_traj.joint_trajectory.points.append(self.inverse_kinematics("left_arm", pos_left, rot_left))
        #dual_arm_traj.joint_trajectory.points.append(self.inverse_kinematics("right_arm", pos_right, rot_right))

    joint_velocities_right = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    joint_accelerations_right = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    i = 0
    while i < n:
        dual_arm_traj.joint_trajectory.points[i].positions = (joints_left[i] + joints_right[i])
        dual_arm_traj.joint_trajectory.points[i].velocities = (dual_arm_traj.joint_trajectory.points[i].velocities +
                                                               joint_velocities_right)
        dual_arm_traj.joint_trajectory.points[i].accelerations = (dual_arm_traj.joint_trajectory.points[i].accelerations
                                                                  + joint_accelerations_right)
        i = i + 1
    rospy.sleep(1)
    # print dual_arm_traj.joint_trajectory
    print len(dual_arm_traj.joint_trajectory.points)
    print rot_left
    print rot_right
    print dual_arm_traj.joint_trajectory.points[0]
    group.execute(dual_arm_traj)

  def test(self, pose_left, pose_right):
      group_left = self.group_left_arm
      wpose1 = group_left.get_current_pose("L_ee").pose
      group_right = self.group_right_arm
      wpose2 = group_right.get_current_pose("R_ee").pose

      group_left.set_pose_reference_frame("/base_link")
      group_right.set_pose_reference_frame("/base_link")

      # 获取末端执行器当前姿态

      waypoints1 = []
      # 路点数据
      wpose1.position.x += pose_left[0]
      wpose1.position.y += pose_left[1]
      wpose1.position.z += pose_left[2]
      # 笛卡尔运动
      waypoints1.append(copy.deepcopy(wpose1))

      waypoints2 = []

      # 路点数据
      wpose2.position.x += pose_right[0]
      wpose2.position.y += pose_right[1]
      wpose2.position.z += pose_right[2]
      # 笛卡尔运动
      waypoints2.append(copy.deepcopy(wpose2))

      (plan_left, fraction) = group_left.compute_cartesian_path(
          waypoints1,  # 路点列表
          0.01,  # 终端步进值
          0.0)  # 跳跃阈值

      (plan_right, fraction) = group_right.compute_cartesian_path(
          waypoints2,  # 路点列表
          0.01,  # 终端步进值
          0.0)  # 跳跃阈值

      n_points = len(plan_left.joint_trajectory.points)
      for i in range(n_points-1):
          print "left", i, "=", plan_left.joint_trajectory.points[i].velocities
          print "right", i, "=", plan_right.joint_trajectory.points[i].velocities
          print "\n"
      # new_traj = moveit_commander.RobotTrajectory()
      # new_traj.joint_trajectory.joint_names = plan.joint_trajectory.joint_names
      #
      # n_points = len(plan.joint_trajectory.points)
      # a = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
      #
      # for i in range(n_points):
      #     new_traj.joint_trajectory.points.append(plan.joint_trajectory.points[i])
      #     #new_traj.joint_trajectory.points[i].velocities = a
      #     #new_traj.joint_trajectory.points[i].accelerations = a
      #
      group_left.execute(plan_left)
      # print new_traj
      group_right.execute(plan_right)

      # 设置场景物体的颜色

  def setColor(self, name, r, g, b, a=0.9):
      # 初始化moveit颜色对象
      color = ObjectColor()

      # 设置颜色值
      color.id = name
      color.color.r = r
      color.color.g = g
      color.color.b = b
      color.color.a = a

      # 更新颜色字典
      self.colors[name] = color

  # 将颜色设置发送并应用到moveit场景当中
  def sendColors(self):
      # 初始化规划场景对象
      p = PlanningScene()

      # 需要设置规划场景是否有差异
      p.is_diff = True

      # 从颜色字典中取出颜色设置
      for color in self.colors.values():
          p.object_colors.append(color)

      # 发布场景物体颜色设置
      self.scene_pub.publish(p)


  def test1(self, group_name, pos, rot):
      if group_name == "left_arm":
          group = self.group_left_arm
      elif group_name == "right_arm":
          group = self.group_right_arm
      group.set_planner_id("RRTConnectkConfigDefault")
      group.set_pose_reference_frame("/base_link")

      joint_state = JointState()
      joint_state.header.stamp = rospy.Time.now()
      joint_state.header.frame_id = ""
      joint_state.name.append("L_Joint1")
      joint_state.name.append("L_Joint2")
      joint_state.name.append("L_Joint3")
      joint_state.name.append("L_Joint4")
      joint_state.name.append("L_Joint5")
      joint_state.name.append("L_Joint6")
      joint_state.name.append("L_Joint7")
      joint_state.name.append("L_left_joint")
      joint_state.name.append("L_right_joint")
      joint_state.name.append("R_Joint1")
      joint_state.name.append("R_Joint2")
      joint_state.name.append("R_Joint3")
      joint_state.name.append("R_Joint4")
      joint_state.name.append("R_Joint5")
      joint_state.name.append("R_Joint6")
      joint_state.name.append("R_Joint7")
      joint_state.name.append("R_left_joint")
      joint_state.name.append("R_right_joint")
      joint_state.name.append("H_Joint1")
      joint_state.name.append("H_Joint2")
      joint_state.position.append(0)
      joint_state.position.append(0)
      joint_state.position.append(0)
      joint_state.position.append(0)
      joint_state.position.append(0)
      joint_state.position.append(0)
      joint_state.position.append(0)
      joint_state.position.append(0)
      joint_state.position.append(0)
      joint_state.position.append(-2.210056429022826)
      joint_state.position.append(0.26788235910926605)
      joint_state.position.append(1.200076413567296)
      joint_state.position.append(0.41010590800445534)
      joint_state.position.append(2.4559447478455088)
      joint_state.position.append(-0.6531048157423776)
      joint_state.position.append(-1.4547845692550574)
      joint_state.position.append(0)
      joint_state.position.append(0)
      joint_state.position.append(0)
      joint_state.position.append(0)

      print joint_state
      moveit_robot_state = RobotState()
      moveit_robot_state.joint_state = joint_state
      group.set_start_state(moveit_robot_state)

      #group.get_end_effector_link()

      target_pose = geometry_msgs.msg.PoseStamped()
      target_pose.header.frame_id = 'base_link'
      target_pose.pose.position.x = pos[0]
      target_pose.pose.position.y = pos[1]
      target_pose.pose.position.z = pos[2]
      quaternion = transformations.quaternion_from_euler(rot[0], rot[1], rot[2])
      target_pose.pose.orientation.x = quaternion[0]
      target_pose.pose.orientation.y = quaternion[1]
      target_pose.pose.orientation.z = quaternion[2]
      target_pose.pose.orientation.w = quaternion[3]

      group.set_pose_target(target_pose)
      group.plan()

  def test2(self, pos, rot):
      group = self.group_left_arm
      group.set_planner_id("RRTConnectkConfigDefault")
      group.set_pose_reference_frame("/base_link")

      # 获取末端执行器当前姿态
      group_end_effector_link = group.get_end_effector_link()
      # 设置目标点末端执行器的位姿
      target_pose = geometry_msgs.msg.PoseStamped()
      target_pose.header.frame_id = "base_link"
      target_pose.pose.position.x = pos[0]
      target_pose.pose.position.y = pos[1]
      target_pose.pose.position.z = pos[2]
      quaternion = transformations.quaternion_from_euler(rot[0], rot[1], rot[2])
      target_pose.pose.orientation.x = quaternion[0]
      target_pose.pose.orientation.y = quaternion[1]
      target_pose.pose.orientation.z = quaternion[2]
      target_pose.pose.orientation.w = quaternion[3]

      print target_pose.pose.orientation.x
      print target_pose.pose.orientation.y
      print target_pose.pose.orientation.z
      print target_pose.pose.orientation.w

      group.set_pose_target(target_pose, group_end_effector_link)
      plan = group.plan()

      traj_replan = moveit_commander.RobotTrajectory()

      joint_velocities = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
      joint_accelerations = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

      traj_replan.joint_trajectory.joint_names = plan.joint_trajectory.joint_names

      n_points = len(plan.joint_trajectory.points)
      i = 0
      while i < n_points:
          traj_replan.joint_trajectory.points.append(plan.joint_trajectory.points[i])
          traj_replan.joint_trajectory.points[i].positions = (plan.joint_trajectory.points[i].positions)
          traj_replan.joint_trajectory.points[i].velocities = (joint_velocities)
          traj_replan.joint_trajectory.points[i].accelerations = (joint_velocities)
          i = i + 1
      print traj_replan.joint_trajectory
      group.execute(traj_replan)


  def Plan_Cartesian_Path_Rot(self, group_name, pos, rot, num, scale=1):
    # 是否需要使用笛卡尔运动
    cartesian = rospy.get_param('~cartesian', True)

    if group_name == "left_arm":
        group = self.group_left_arm
        wpose = group.get_current_pose("L_ee").pose
    elif group_name == "right_arm":
        group = self.group_right_arm
        wpose = group.get_current_pose("R_ee").pose

    group.set_pose_reference_frame("/base_link")
    # 设置位置和姿态的允许误差
    group.set_goal_position_tolerance(0.01)
    group.set_goal_orientation_tolerance(0.05)

    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x = pos[0]
    target_pose.position.y = pos[1]
    target_pose.position.z = pos[2]
    quaternion = transformations.quaternion_from_euler(rot[0], rot[1], rot[2])
    target_pose.orientation.x = quaternion[0]
    target_pose.orientation.y = quaternion[1]
    target_pose.orientation.z = quaternion[2]
    target_pose.orientation.w = quaternion[3]

    waypoints = []
    # 路点数据
    wpose.position.x = scale * (pos[0])
    wpose.position.y = scale * (pos[1])
    wpose.position.z = scale * (pos[2])
    wpose.orientation.x = scale * (quaternion[0])
    wpose.orientation.y = scale * (quaternion[1])
    wpose.orientation.z = scale * (quaternion[2])
    wpose.orientation.w = scale * (quaternion[3])

    # 笛卡尔运动
    if cartesian:
        waypoints.append(copy.deepcopy(wpose))

        fraction = 0.0
        i = 0
        # while fraction < 1.0:
        (plan, fraction) = group.compute_cartesian_path(
                                           waypoints,   # 路点列表
                                           0.01,        # 终端步进值
                                           0.0)         # 跳跃阈值
        print 'fraction = ', fraction
        if fraction < 0.7:
            print group_name, " cartesian planning failure"
            group.execute(plan, wait=True)
            exit()
    n_joints = len(plan.joint_trajectory.joint_names)
    n_points = len(plan.joint_trajectory.points)

    #raw_input()
    group.execute(plan, wait=True)

    if num == 1:
        arm_angles = []
        for i in range(n_points):
            for j in range(n_joints):
                joints = plan.joint_trajectory.points[i].positions
            arm_angle = self.arm_angle("left_arm", joints)
            arm_angles.append(arm_angle)
        return arm_angles

    elif num == 2:
        joints = []
        for i in range(n_points):
            joints.append(plan.joint_trajectory.points[i].positions)
        P_elbow = self.forward_kinematics_elbow(joints)
        print P_elbow

    else:
        joints = []
        for i in range(n_points):
            joints.append(plan.joint_trajectory.points[i].positions)
        print joints
        return joints


    # if num == 1:
    #     # for i in range(7):
    #     print plan.joint_trajectory.points[n_points-1].positions
    #     return plan.joint_trajectory.points[n_points-1].positions

  #圆弧轨迹规划
  def Plan_Cartesian_Path_Circle(self, group_name, num):
      # 是否需要使用笛卡尔运动
      cartesian = rospy.get_param('~cartesian', True)

      if group_name == "left_arm":
          group = self.group_left_arm
          wpose = group.get_current_pose("L_ee").pose
      elif group_name == "right_arm":
          group = self.group_right_arm
          wpose = group.get_current_pose("R_ee").pose

      group.set_pose_reference_frame("/base_link")
      # 设置位置和姿态的允许误差
      group.set_goal_position_tolerance(0.01)
      group.set_goal_orientation_tolerance(0.05)

      # joints = [-0.7589887631976004, 1.1816009121835886, -0.7778825300814339, 1.7246689402469935,
      #           0.674630790683862, -1.41401442293726, 1.9804284685614664]
      # group.set_joint_value_target(joints)
      # group.go()
      #
      # target_pose = PoseStamped()
      # target_pose.header.frame_id = 'base_link'
      # target_pose.pose.position.x = 0.293
      # target_pose.pose.position.y = 0
      # target_pose.pose.position.z = 1.05
      # rot = [-1.571, 1.571, -1.571]
      # quaternion = transformations.quaternion_from_euler(rot[0], rot[1], rot[2])
      # target_pose.pose.orientation.x = quaternion[0]
      # target_pose.pose.orientation.y = quaternion[1]
      # target_pose.pose.orientation.z = quaternion[2]
      # target_pose.pose.orientation.w = quaternion[3]
      #
      # # 设置机械臂终端运动的目标位姿
      # group.set_pose_target(target_pose)
      # plan_point = group.plan_point()
      # group.execute(plan_point)
      #
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
      # target_pose2 = target_pose
      # target_pose2.pose.position.y = 0.15
      # waypoints.append(target_pose2.pose)

      # fraction = 0.0  # 路径规划覆盖率
      # maxtries = 100  # 最大尝试规划次数
      # attempts = 0  # 已经尝试规划次数
      #
      # # 设置机器臂当前的状态作为运动初始状态
      # group.set_start_state_to_current_state()
      #
      # # 尝试规划一条笛卡尔空间下的路径，依次通过所有路点，完成圆弧轨迹
      # while fraction < 1.0 and attempts < maxtries:
      #     (plan, fraction) = group.compute_cartesian_path(
      #         waypoints,  # waypoint poses，路点列表
      #         0.01,  # eef_step，终端步进值
      #         0.0,  # jump_threshold，跳跃阈值
      #         True)  # avoid_collisions，避障规划
      #
      #     # 尝试次数累加
      #     attempts += 1
      #
      # # 如果路径规划成功（覆盖率100%）,则开始控制机械臂运动
      # if fraction == 1.0:
      #     rospy.loginfo("Path computed successfully. Moving the arm.")
      #     group.execute(plan)
      #     rospy.loginfo("Path execution complete.")
      #
      # rospy.sleep(10)

      waypoints = []

      target_pose = geometry_msgs.msg.PoseStamped()
      target_pose.pose.position.x = 0.3
      target_pose.pose.position.y = 0.2
      target_pose.pose.position.z = 1.05
      rot = [-1.571, 1.571, -1.571]
      quaternion = transformations.quaternion_from_euler(rot[0], rot[1], rot[2])
      target_pose.pose.orientation.x = quaternion[0]
      target_pose.pose.orientation.y = quaternion[1]
      target_pose.pose.orientation.z = quaternion[2]
      target_pose.pose.orientation.w = quaternion[3]

      centerA = target_pose.pose.position.y
      centerB = target_pose.pose.position.z
      radius = 0.15


      for th in numpy.arange(0, 6.30, 0.02):
          target_pose.pose.position.y = centerA + radius * math.cos(th)
          target_pose.pose.position.z = centerB + radius * math.sin(th)
          wpose = copy.deepcopy(target_pose.pose)
          waypoints.append(copy.deepcopy(wpose))

          # print('%f, %f' % (Y, Z))

      fraction = 0.0  # 路径规划覆盖率
      maxtries = 100  # 最大尝试规划次数
      attempts = 0  # 已经尝试规划次数

      # 设置机器臂当前的状态作为运动初始状态
      group.set_start_state_to_current_state()

      # 尝试规划一条笛卡尔空间下的路径，依次通过所有路点，完成圆弧轨迹
      while fraction < 1.0 and attempts < maxtries:
          (plan, fraction) = group.compute_cartesian_path(
              waypoints,  # waypoint poses，路点列表
              0.01,  # eef_step，终端步进值
              0.0,  # jump_threshold，跳跃阈值
              True)  # avoid_collisions，避障规划

          # 尝试次数累加
          attempts += 1

          # 打印运动规划进程
          if attempts % 10 == 0:
              rospy.loginfo("Still trying after " + str(attempts) + " attempts...")

      # 如果路径规划成功（覆盖率100%）,则开始控制机械臂运动
      if fraction == 1.0:
          rospy.loginfo("Path computed successfully. Moving the arm.")
          group.execute(plan)
          rospy.loginfo("Path execution complete.")
      # 如果路径规划失败，则打印失败信息
      else:
          rospy.loginfo("Path planning failed with only " + str(fraction) + " success after " + str(
              maxtries) + " attempts.")

      rospy.sleep(1)

      n_joints = len(plan.joint_trajectory.joint_names)
      n_points = len(plan.joint_trajectory.points)

      if num == 1:
          arm_angles = []
          for i in range(n_points):
              for j in range(n_joints):
                  joints = plan.joint_trajectory.points[i].positions
              arm_angle = self.arm_angle("left_arm", joints)
              arm_angles.append(arm_angle)
          return arm_angles

      elif num == 2:
          joints = []
          for i in range(n_points):
              joints.append(plan.joint_trajectory.points[i].positions)
          P_elbow = self.forward_kinematics_elbow(joints)
          print P_elbow

      else:
          joints = []
          for i in range(n_points):
              joints.append(plan.joint_trajectory.points[i].positions)
          print joints
          return joints


  #圆弧轨迹规划(改变角度)
  def Plan_Cartesian_Path_Circle2(self, group_name, num):
      # 是否需要使用笛卡尔运动
      cartesian = rospy.get_param('~cartesian', True)

      if group_name == "left_arm":
          group = self.group_left_arm
          wpose = group.get_current_pose("L_ee").pose
      elif group_name == "right_arm":
          group = self.group_right_arm
          wpose = group.get_current_pose("R_ee").pose

      group.set_pose_reference_frame("/base_link")
      # 设置位置和姿态的允许误差
      group.set_goal_position_tolerance(0.01)
      group.set_goal_orientation_tolerance(0.05)

      # joints = [-0.7589887631976004, 1.1816009121835886, -0.7778825300814339, 1.7246689402469935,
      #           0.674630790683862, -1.41401442293726, 1.9804284685614664]
      # group.set_joint_value_target(joints)
      # group.go()
      #
      # target_pose = PoseStamped()
      # target_pose.header.frame_id = 'base_link'
      # target_pose.pose.position.x = 0.293
      # target_pose.pose.position.y = 0
      # target_pose.pose.position.z = 1.05
      # rot = [-1.571, 1.571, -1.571]
      # quaternion = transformations.quaternion_from_euler(rot[0], rot[1], rot[2])
      # target_pose.pose.orientation.x = quaternion[0]
      # target_pose.pose.orientation.y = quaternion[1]
      # target_pose.pose.orientation.z = quaternion[2]
      # target_pose.pose.orientation.w = quaternion[3]
      #
      # # 设置机械臂终端运动的目标位姿
      # group.set_pose_target(target_pose)
      # plan_point = group.plan_point()
      # group.execute(plan_point)
      #
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
      # target_pose2 = target_pose
      # target_pose2.pose.position.y = 0.15
      # waypoints.append(target_pose2.pose)

      # fraction = 0.0  # 路径规划覆盖率
      # maxtries = 100  # 最大尝试规划次数
      # attempts = 0  # 已经尝试规划次数
      #
      # # 设置机器臂当前的状态作为运动初始状态
      # group.set_start_state_to_current_state()
      #
      # # 尝试规划一条笛卡尔空间下的路径，依次通过所有路点，完成圆弧轨迹
      # while fraction < 1.0 and attempts < maxtries:
      #     (plan, fraction) = group.compute_cartesian_path(
      #         waypoints,  # waypoint poses，路点列表
      #         0.01,  # eef_step，终端步进值
      #         0.0,  # jump_threshold，跳跃阈值
      #         True)  # avoid_collisions，避障规划
      #
      #     # 尝试次数累加
      #     attempts += 1
      #
      # # 如果路径规划成功（覆盖率100%）,则开始控制机械臂运动
      # if fraction == 1.0:
      #     rospy.loginfo("Path computed successfully. Moving the arm.")
      #     group.execute(plan)
      #     rospy.loginfo("Path execution complete.")
      #
      # rospy.sleep(10)

      waypoints = []

      target_pose = geometry_msgs.msg.PoseStamped()
      target_pose.pose.position.x = 0.293
      target_pose.pose.position.y = 0.0
      target_pose.pose.position.z = 1.14
      rot = [-1.571, 1.571, -1.571]
      quaternion = transformations.quaternion_from_euler(rot[0], rot[1], rot[2])
      target_pose.pose.orientation.x = quaternion[0]
      target_pose.pose.orientation.y = quaternion[1]
      target_pose.pose.orientation.z = quaternion[2]
      target_pose.pose.orientation.w = quaternion[3]

      centerA = target_pose.pose.position.y
      centerB = target_pose.pose.position.z
      radius = 0.15


      for th in numpy.arange(0, 6.30, 0.02):
          target_pose.pose.position.y = centerA + radius * math.cos(th+0.6) # 改变（th+/）
          target_pose.pose.position.z = centerB + radius * math.sin(th+0.6) # 
          wpose = copy.deepcopy(target_pose.pose)
          waypoints.append(copy.deepcopy(wpose))

          # print('%f, %f' % (Y, Z))

      fraction = 0.0  # 路径规划覆盖率
      maxtries = 100  # 最大尝试规划次数
      attempts = 0  # 已经尝试规划次数

      # 设置机器臂当前的状态作为运动初始状态
      group.set_start_state_to_current_state()

      # 尝试规划一条笛卡尔空间下的路径，依次通过所有路点，完成圆弧轨迹
      while fraction < 1.0 and attempts < maxtries:
          (plan, fraction) = group.compute_cartesian_path(
              waypoints,  # waypoint poses，路点列表
              0.01,  # eef_step，终端步进值
              0.0,  # jump_threshold，跳跃阈值
              True)  # avoid_collisions，避障规划

          # 尝试次数累加
          attempts += 1

          # 打印运动规划进程
          if attempts % 10 == 0:
              rospy.loginfo("Still trying after " + str(attempts) + " attempts...")

      # 如果路径规划成功（覆盖率100%）,则开始控制机械臂运动
      if fraction == 1.0:
          rospy.loginfo("Path computed successfully. Moving the arm.")
          group.execute(plan)
          rospy.loginfo("Path execution complete.")
      # 如果路径规划失败，则打印失败信息
      else:
          rospy.loginfo("Path planning failed with only " + str(fraction) + " success after " + str(
              maxtries) + " attempts.")

      rospy.sleep(1)

      n_joints = len(plan.joint_trajectory.joint_names)
      n_points = len(plan.joint_trajectory.points)

      if num == 1:
          arm_angles = []
          for i in range(n_points):
              for j in range(n_joints):
                  joints = plan.joint_trajectory.points[i].positions
              arm_angle = self.arm_angle("left_arm", joints)
              arm_angles.append(arm_angle)
          return arm_angles

      elif num == 2:
          joints = []
          for i in range(n_points):
              joints.append(plan.joint_trajectory.points[i].positions)
          P_elbow = self.forward_kinematics_elbow(joints)
          print P_elbow

      else:
          joints = []
          for i in range(n_points):
              joints.append(plan.joint_trajectory.points[i].positions)
          print joints
          return joints
