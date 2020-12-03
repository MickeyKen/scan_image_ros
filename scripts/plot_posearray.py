#!/usr/bin/python
import rospy
import numpy as np
from geometry_msgs.msg import Pose, PoseArray
import laser_geometry.laser_geometry as lg
import sensor_msgs.point_cloud2 as pc2

import matplotlib.pyplot as plt
import math

def extract(point):
    return [point[0], point[1], point[2]]

def plot(pose):
    ps = []

    for i in range(len(pose.poses)):
        ps.append([pose.poses[i].position.x,pose.poses[i].position.y])
    # print ps

    return ps

if __name__ == '__main__':
    rospy.init_node('plot_posearray_node', anonymous=True)

    while True:
        plt.ion()
        plt.title('HEATMAP')
        plt.xlim(-10,10)
        plt.ylim(-10,10)
        plt.grid()

        try:
            data = rospy.wait_for_message('/scan/detect_leg_person', PoseArray, timeout=1)
            points = plot(data)

            for p in points:
                plt.plot(p[0] ,p[1] , 'o', color="blue", markersize=3)
        except:
            pass

        plt.draw()
        plt.pause(0.1)
        plt.clf()


    # rospy.spin()
