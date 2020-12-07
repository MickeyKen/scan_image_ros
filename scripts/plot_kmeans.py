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
            # print points

            km = KMeans(n_clusters=k, init='random')
            g = km.fit_predict(points)
            cmap = plt.get_cmap("tab10")
            # print (np.where(g==0))
            # l = np.where(g==0)
            # print points
            output0_x = [points[index][0] for index in np.where(g==0)[0]]
            output1_x = [points[index][0] for index in np.where(g==1)[0]]
            output0_y = [points[index][1] for index in np.where(g==0)[0]]
            output1_y = [points[index][1] for index in np.where(g==1)[0]]
            # print output0_x, output0_y
            # print output1_x, output1_y
            output0_x_ave = np.average(output0_x)
            output0_y_ave = np.average(output0_y)
            output1_x_ave = np.average(output1_x)
            output1_y_ave = np.average(output1_y)
            print output0_x_ave, output0_y_ave
            print output1_x_ave, output1_y_ave
            distance0 = math.hypot(output0_x_ave, output0_y_ave)
            distance1 = math.hypot(output1_x_ave, output1_y_ave)

            if distance0 > distance1:
                print "defeat output1"
                plt.plot(output1_x_ave ,output1_y_ave , 'o', color="#CCCCFF", markersize=90, alpha=0.5)
                # plt.plot(output1_x_ave ,output1_y_ave , 'o', color="red", markersize=40, fill=false)
                # plt.Circle((output1_x_ave, output1_y_ave), 50, color='red', fill=False)
            else:
                print "defeat output0"
                plt.plot(output0_x_ave ,output0_y_ave , 'o', color="#CCCCFF", markersize=90, alpha=0.5)
                # plt.plot(output0_x_ave ,output0_y_ave , 'o', color="red", markersize=40, fill=false)
                # plt.Circle((output0_x_ave, output0_y_ave), 50, color='red', fill=False)
            # print(np.mean(output0, axis=0))
            # print(np.mean(output0, axis=1))

                # for i in index:
                #     print i

            # output0 = [points[x] for x in range(np.where(g==0))]
            # print output0
            # print g
            for ell in range(k):
                plt.scatter(points[g==ell,0], points[g==ell,1], s=2,color=cmap(ell))
        except:
            pass

        plt.draw()
        plt.pause(0.1)
        plt.clf()


    # rospy.spin()
