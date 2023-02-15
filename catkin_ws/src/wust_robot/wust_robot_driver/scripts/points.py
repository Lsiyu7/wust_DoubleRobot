#!/usr/bin/env python
#coding:utf-8

import rospy
import visualization_msgs.msg
import geometry_msgs.msg

class Point():
    def __init__(self):
        rospy.init_node('sample_points', anonymous=False)
        vis_pub = rospy.Publisher('/sample_points', visualization_msgs.msg.Marker, queue_size=20)
        self.vis_pub = vis_pub
        while True:
            points = visualization_msgs.msg.Marker()
            points.header.frame_id = "/base_link"
            points.header.stamp = rospy.Time.now()
            points.ns = "sample_points"
            points.type = visualization_msgs.msg.Marker.LINE_LIST
            points.action = visualization_msgs.msg.Marker.ADD
            # rospy.loginfo("x,y,z = %f %f, %f", points.pose.position.x, points.pose.position.y, points.pose.position.z)

            points.id = 0
            points.scale.x = 0.05
            #points.scale.y = 0.01
            #points.scale.z = 0.01

            points.color.r = 1.0
            # points.color.g = 1.0
            # points.color.b = 1.0
            points.color.a = 1.0   # 透明度

            # i = 1
            # while i <= 5:
            #     p = geometry_msgs.msg.Point()
            #     x = 1
            #     y = 1
            #     z = 1
            #     p.x = x
            #     p.y = i - 5
            #     p.z = z
            #     points.points.append(p)
            #     i = i + 0.2
            # vis_pub.publish(points)

            i = 1
            while i <= 5:
                j = 1
                while j <= 3:
                    k = 1
                    while k <= 2:
                        p = geometry_msgs.msg.Point()
                        x = 1
                        y = 1
                        z = 1
                        p.x = k
                        p.y = i - 3
                        p.z = j - 1
                        points.points.append(p)
                        k = k + 0.05
                    j = j + 0.05
                i = i + 0.05
            vis_pub.publish(points)

            #print points
            #print len(points)
            #print len(points.points)

            # i = 1
            # p.x = 1.138
            # while i <= 2:
            #     p.x = p.x - 0.01
            #     j = 1
            #     p.y = -0.511
            #     while j <= 2:
            #         p.y = p.y - 0.01
            #         k = 1
            #         p.z = 1.260
            #         while k <= 2:
            #             p.z = p.z - 0.01
            #             points.points.append(p)
            #             print points
            #             vis_pub.publish(points)
            #             k = k + 1
            #         j = j + 1
            #     i = i + 1



if __name__ == '__main__':
    Point()