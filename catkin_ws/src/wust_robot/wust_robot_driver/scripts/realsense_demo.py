#!/usr/bin/env python

import copy
import math
import rospy
import actionlib
import tf

from geometry_msgs.msg import *
from moveit_msgs.msg import *
from moveit_msgs.srv import *
from actionlib_msgs.msg import *
from tf.listener import *

from ar_track_alvar_msgs.msg import AlvarMarkers

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from moveit_commander import PlanningSceneInterface, MoveGroupCommander
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class two_dimension_code:
    def __init__(self):
        rospy.init_node('two_dimension_code', anonymous=False)
        self.ar_poses = []
        self.target_pose = None
        rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.ar_pose_cb)

    def ar_pose_cb(self, msg):
        if len(msg.markers)>0:
            pose_info = msg.markers[0].pose.pose
            ar_pose = [pose_info.position.x, pose_info.position.y, pose_info.position.z]
            print ar_pose
            self.ar_poses.append(ar_pose)
        print msg.markers
        if len(self.ar_poses) == 3:
            px = []
            py = []
            pz = []
            for pose_list in self.ar_poses:
                px.append(pose_list[0])
                py.append(pose_list[1])
                pz.append(pose_list[2])
            self.target_pose = [sum(px)/3, sum(py)/3, sum(pz)/3]
            #print self.target_pose
            self.ar_poses = []  

if __name__ == '__main__':
    try:
        two_dimension_code()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("two_dimension_code test finished.")  
