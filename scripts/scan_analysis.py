#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import Image, LaserScan, PointCloud2
import laser_geometry.laser_geometry as lg
import sensor_msgs.point_cloud2 as pc2
import time
import cv2
import yaml
from pyquaternion import Quaternion

def extract(point):
    return [point[0], point[1], point[2]]

def callback(scan):
    print len(scan.ranges)

    lp = lg.LaserProjection()
    cloud = lp.projectLaser(scan)
    # print cloud
    points = pc2.read_points(cloud)
    # print type(points)
    objPoints = np.array(map(extract, points))
    # print len(objPoints)

    objPoints = np.reshape(objPoints, (1,objPoints.shape[0],objPoints.shape[1]))
    print objPoints[0]
    img_points, _ = cv2.fisheye.projectPoints(objPoints, rvec, tvec, K, D)
    # print img_points[0][0]
    target_img_points = np.array([[img_points[0][0][0], img_points[0][0][1], 1]])
    # world = np.dot(np.dot(target_img_points, np.linalg.inv(K)) - np.reshape(tvec,[3,-1]),np.linalg.inv(rot_mat))
    world = np.dot(np.dot(target_img_points, np.linalg.inv(K)) - tvec,np.linalg.inv(rot_mat))
    # print [img_points[0][0][0],img_points[0][0][1],0.0]
    # new_obj, _ = cv2.fisheye.projectPoints([img_points[0][0][0],img_points[0][0][1],0.0], rvec, tvec, K, D)
    # print new_obj
    point_out = cv2.fisheye.undistortPoints(img_points,K =K, D=D)[0]
    print point_out[0]
    # print np.dot(target_img_points, np.linalg.inv(K)) - tvec
    # print world



if __name__ == '__main__':
    rospy.init_node('scan_subscriber', anonymous=True)

    scan_data_sub = rospy.Subscriber("/scan",LaserScan,callback)
    with open("../data/calibration_result.txt", 'r') as f:
        data = f.read().split()
        qx = float(data[0])
        qy = float(data[1])
        qz = float(data[2])
        qw = float(data[3])
        tx = float(data[4])
        ty = float(data[5])
        tz = float(data[6])
    q = Quaternion(qw,qx,qy,qz).transformation_matrix
    q[0,3] = tx
    q[1,3] = ty
    q[2,3] = tz
    print("Extrinsic parameter - camera to laser")
    print("q =")
    print(q)
    tvec = q[:3,3]
    rot_mat = q[:3,:3]
    rvec, _ = cv2.Rodrigues(rot_mat)
    print("tvec =")
    print(tvec)
    print("rvec =")
    print(rvec)

    with open("../config/config_HD.yaml", 'r') as f:
        f.readline()
        config = yaml.load(f)
        lens = config['lens']
        fx = float(config['fx'])
        fy = float(config['fy'])
        cx = float(config['cx'])
        cy = float(config['cy'])
        k1 = float(config['k1'])
        k2 = float(config['k2'])
        p1 = float(config['p1/k3'])
        p2 = float(config['p2/k4'])
    K = np.matrix([[fx, 0.0, cx],
                   [0.0, fy, cy],
                   [0.0, 0.0, 1.0]])
    D = np.array([k1, k2, p1, p2])
    print("Camera parameters")
    print("Lens = %s" % lens)
    print("K =")
    print(K)
    print("D =")
    print(D)
    rospy.spin()
