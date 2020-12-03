#!/usr/bin/python
import rospy
import numpy as np
from sensor_msgs.msg import Image, LaserScan, PointCloud2
import laser_geometry.laser_geometry as lg
import sensor_msgs.point_cloud2 as pc2

import matplotlib.pyplot as plt
import math

def extract(point):
    return [point[0], point[1], point[2]]

def plot(scan):

    lp = lg.LaserProjection()
    cloud = lp.projectLaser(scan)
    # print cloud
    points = pc2.read_points(cloud)
    print type(points)
    objPoints = np.array(map(extract, points))
    print objPoints
    return objPoints

if __name__ == '__main__':
    rospy.init_node('plot_scan_node', anonymous=True)

    while True:
        plt.ion()
        plt.title('HEATMAP')
        plt.xlim(-10,10)
        plt.ylim(-10,10)
        plt.grid()
        data = rospy.wait_for_message('/front_laser_scan', LaserScan, timeout=5)

    	points = plot(data)

        for p in points:
            plt.plot(p[0] ,p[1] , 'o', color="blue", markersize=3)

        plt.draw()
        plt.pause(0.1)
        plt.clf()


    # rospy.spin()
