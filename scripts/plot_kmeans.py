#!/usr/bin/python
import rospy
import numpy as np
from geometry_msgs.msg import Pose, PoseArray
import laser_geometry.laser_geometry as lg
import sensor_msgs.point_cloud2 as pc2

import matplotlib.pyplot as plt
import math

from sklearn.cluster import KMeans

def extract(point):
    return [point[0], point[1], point[2]]

def plot(pose):
    x = np.empty((0,2))

    for i in range(len(pose.poses)):
        x = np.vstack((x, [pose.poses[i].position.x, pose.poses[i].position.y]))

    return x

if __name__ == '__main__':
    rospy.init_node('plot_posearray_node', anonymous=True)

    k = 2

    while True:
        plt.ion()
        plt.title('HEATMAP')
        plt.xlim(-4,4)
        plt.ylim(0,5)
        plt.grid()

        try:
            data = rospy.wait_for_message('/scan/detect_leg_person', PoseArray, timeout=1)
            points = plot(data)
            print points

            km = KMeans(n_clusters=k, init='random')
            g = km.fit_predict(points)
            cmap = plt.get_cmap("tab10")
            print g
            for ell in range(k):
                plt.scatter(points[g==ell,0], points[g==ell,1], s=2,color=cmap(ell))
        except:
            pass

        plt.draw()
        plt.pause(0.1)
        plt.clf()


    # rospy.spin()
