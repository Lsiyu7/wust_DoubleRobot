import rospy
import math
import geometry_msgs.msg
from tf import transformations
from wust_robot_moveit_control_full import Wust_Robot_Moveit_Control, Object_parameters
import visualization_msgs.msg
import numpy
import moveit_msgs.msg
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import commands
import os

def publish(JS):
    pub = rospy.Publisher('joint_states', JointState, queue_size=30)
    rate = rospy.Rate(10)
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
    LR_shoulder=[0,0,0]
    JLE=1.575
    JR_wrist=[0,0,0]
    J_S = [0.0, 0.0, 0.0,LR_shoulder[0], LR_shoulder[1], LR_shoulder[2], JLE, 
                JR_wrist[0], JR_wrist[1], JR_wrist[2],0.0, 0.0,
                0, 0, 0, 0, 
                0,0,0, 0.0, 0.0, 0.0, 0.0]
        publish(J_S)