#!/usr/bin/env python
#coding:utf-8

import rospy
import geometry_msgs.msg
import visualization_msgs.msg
from mrp2a_moveit_control_full import Mrp2a_Moveit_Control, Object_parameters
from std_msgs.msg import String
import tf



class Traj_Marker:
    def __init__(self):
        #rospy.init_node('visualization_marker', anonymous=False)
        marker_pub = rospy.Publisher('/visualization_marker', visualization_msgs.msg.Marker, queue_size=1)

        points = visualization_msgs.msg.Marker()
        points.ns = "visualization_marker"
        points.header.stamp = rospy.Time.now()
        points.type = visualization_msgs.msg.Marker.POINTS
        points.action = visualization_msgs.msg.Marker.ADD
        #points.header.frame_id = "/R_Link0"

        points.id = 0
        points.scale.x = 0.005
        points.scale.y = 0.005
        points.scale.z = 0.005

        points2 = visualization_msgs.msg.Marker()
        points2.ns = "visualization_marker"
        points2.header.stamp = rospy.Time.now()
        points2.type = visualization_msgs.msg.Marker.POINTS
        points2.action = visualization_msgs.msg.Marker.ADD
        #points2.header.frame_id = "/R_Link0"

        points2.id = 1
        points2.scale.x = 0.005
        points2.scale.y = 0.005
        points2.scale.z = 0.005

        self.marker_pub = marker_pub
        self.points = points
        self.points2 = points2

        rospy.Subscriber("/traj_point", String, self.callback)



    def Display_Point(self, x, y, z, result):
        marker_pub = self.marker_pub
        points = self.points
        points2 = self.points2
        p = geometry_msgs.msg.Point()
        p.x = x
        p.y = y
        p.z = z
        if result == "success":
            # points.color.r = 1.0
            points.color.g = 1.0
            # points.color.b = 1.0
            points.color.a = 1.0  # 透明度
            points.header.frame_id = "/base_link"

            points.points.append(p)
            marker_pub.publish(points)
        elif result == "failure":

            points2.color.r = 1.0
            # points2.color.g = 1.0
            # points2.color.b = 1.0
            points2.color.a = 1.0  # 透明度

            points2.points.append(p)
            marker_pub.publish(points2)

    def callback(self, data):
        marker_pub = self.marker_pub
        points = self.points
        points2 = self.points2

        while not rospy.is_shutdown():
            tf_listener = tf.TransformListener()
            try:
                (trans, rot) = tf_listener.lookupTransform('/base_link', '/R_ee', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            # pose_quaternion = geometry_msgs.msg.PoseStamped()
            # pose_quaternion.header.frame_id = "base_link"
            # pose_quaternion.pose.position.x = trans[0]
            # pose_quaternion.pose.position.y = trans[1]
            # pose_quaternion.pose.position.z = trans[2]
            # # quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)
            # pose_quaternion.pose.orientation.x = rot[0]
            # pose_quaternion.pose.orientation.y = rot[1]
            # pose_quaternion.pose.orientation.z = rot[2]
            # pose_quaternion.pose.orientation.w = rot[3]
            p = geometry_msgs.msg.Point()
            p.x = trans[0]
            p.y = trans[1]
            p.z = trans[2]
            points.color.g = 1.0
            points.color.a = 1.0  # 透明度
            points.header.frame_id = "/base_link"
            points.points.append(p)
            marker_pub.publish(points)
        # p = geometry_msgs.msg.Point()
        # p.x = x
        # p.y = y
        # p.z = z
        # if result == "success":
        #     # points.color.r = 1.0
        #     points.color.g = 1.0
        #     # points.color.b = 1.0
        #     points.color.a = 1.0  # 透明度
        #     points.header.frame_id = "/base_link"
        #
        #     points.points.append(p)
        #     marker_pub.publish(points)
        # elif result == "failure":
        #
        #     points2.color.r = 1.0
        #     # points2.color.g = 1.0
        #     # points2.color.b = 1.0
        #     points2.color.a = 1.0  # 透明度
        #
        #     points2.points.append(p)
        #     marker_pub.publish(points2)




# 监听机械臂末端位姿
if __name__ == '__main__':

    mrp2a = Mrp2a_Moveit_Control()
    traj_marker = Traj_Marker()

    tf_listener = tf.TransformListener()

    cout = mrp2a.cout

    pos_left = [0.95, 0.15, 1.02]
    rot_left = [1.57, 0, 1.57]
    pos_right = [0.95, -0.15, 1.02]
    rot_right = [1.57, 0, 1.57]
    mrp2a.dual_arm_pose_control(pos_left, rot_left, pos_right, rot_right)
    rospy.sleep(0.5)

    while not rospy.is_shutdown():
        try:
            (trans, rot) = tf_listener.lookupTransform('/base_link', '/R_ee', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        pose_quaternion = geometry_msgs.msg.PoseStamped()
        pose_quaternion.header.frame_id = "base_link"
        pose_quaternion.pose.position.x = trans[0]
        pose_quaternion.pose.position.y = trans[1]
        pose_quaternion.pose.position.z = trans[2]
        # quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)
        pose_quaternion.pose.orientation.x = rot[0]
        pose_quaternion.pose.orientation.y = rot[1]
        pose_quaternion.pose.orientation.z = rot[2]
        pose_quaternion.pose.orientation.w = rot[3]

        print cout

        if cout == 100:
            traj_marker.Display_Point(trans[0], trans[1], trans[2], "success")

        # points.color.g = 1.0
        # points.color.a = 1.0  # 透明度
        # points.header.frame_id = "/base_link"
        # points.points.append(p)
        # marker_pub.publish(points)


