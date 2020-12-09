#!/usr/bin/env python

import rospy
import numpy as np
import yaml
import math
from geometry_msgs.msg import PoseStamped, PointStamped
from geometry_msgs.msg import Pose, PoseArray
from sklearn.cluster import KMeans


k = 2


def insert_np(pose):
    x = np.empty((0,2))

    for i in range(len(pose.poses)):
        x = np.vstack((x, [pose.poses[i].position.x, pose.poses[i].position.y]))

    return x

def callback(data):

    points = insert_np(data)

    km = KMeans(n_clusters=k, init='random')
    g = km.fit_predict(points)

    output0_x = [points[index][0] for index in np.where(g==0)[0]]
    output1_x = [points[index][0] for index in np.where(g==1)[0]]
    output0_y = [points[index][1] for index in np.where(g==0)[0]]
    output1_y = [points[index][1] for index in np.where(g==1)[0]]

    output0_x_ave = np.average(output0_x)
    output0_y_ave = np.average(output0_y)
    output1_x_ave = np.average(output1_x)
    output1_y_ave = np.average(output1_y)
    distance0 = math.hypot(output0_x_ave, output0_y_ave)
    distance1 = math.hypot(output1_x_ave, output1_y_ave)


    if distance0 > distance1:
        msg = PointStamped()
        msg.point.x = output1_x_ave
        msg.point.y = output1_y_ave
        msg.header.frame_id = "/base_scan"
        msg.header.stamp = rospy.Time.now()
        pub.publish(msg)
    else:
        msg = PointStamped()
        msg.point.x = output0_x_ave
        msg.point.y = output0_y_ave
        msg.header.frame_id = "/base_scan"
        msg.header.stamp = rospy.Time.now()
        pub.publish(msg)


rospy.init_node('clustering_node')


pub = rospy.Publisher("target_point", PointStamped, queue_size=10)

img_sub = rospy.Subscriber('/scan/detect_leg_person', PoseArray, callback)

rospy.spin()
