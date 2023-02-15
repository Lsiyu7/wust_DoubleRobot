import rospy, sys
import moveit_commander
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import PoseStamped, Pose
from moveit_msgs.msg import RobotTrajectory, TrajectoryProcessing
from trajectory_msgs.msg import JointTrajectoryPoint

class Wust_Robot_Continue_Trajectory:
    def __init__(self):
	# 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)
	#初始化ROS节点
	rospy.init_node('Wust_Robot_Continue_Trajectory', anonymous=True)
	# 初始化需要使用move group控制的机械臂中的group
	group_dual_arm = moveit_commander.MoveGroupCommander("dual_arm")
	self.group_dual_arm = group_dual_arm

    def dual_arm_joint_control(self, joint_left, joint_right): #

        arm.set_goal_joint_tolerance(0.001)
	accScal = 0.5
        velScale = 0.5
        arm.set_max_acceleration_scaling_factor(accScale)
        arm.set_max_velocity_scaling_factor(velScale)

	group.set_pose_reference_frame("/base_link")
	group_variable_values = group.get_current_joint_values()

	# 控制机械臂先回到初始化位置
        #arm.set_named_target('home')
        #arm.go()
        #rospy.sleep(1)

	# 获取机器人的起始位置
	group.set_start_state_to_current_state()
	#

	#
	#

	# 设置第一个目标点
	joint_positions = [-0.6]
        arm.set_joint_value_target(joint_positions)
       
        # 计算第一条轨迹
        plan1 = group.plan()
      moveit_robot_state = RobotState()
      moveit_robot_state.joint_state = joint_state
      group.set_start_state(moveit_robot_state)
        
	# 设置第2个目标点
	joint_positions = [-0.6]
        arm.set_joint_value_target(joint_positions)
       
        # 计算第2条轨迹
        plan1 = group.plan()
      moveit_robot_state = RobotState()
      moveit_robot_state.joint_state = joint_state
      group.set_start_state(moveit_robot_state)

	#连接两条轨迹
        traj = moveit_commander.RobotTrajectory()
        traj.joint_trajectory.joint_names = plan1.joint_trajectory.joint_names
	traj.joint_trajectory.joint_names = joint_names
        traj.joint_trajectory.points.append(self.inverse_kinematics("dual_arm", pos_, rot_))







































