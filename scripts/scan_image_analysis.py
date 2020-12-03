#!/usr/bin/env python
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from pyquaternion import Quaternion
import yaml
import numpy as np
import message_filters
from sensor_msgs.msg import Image, LaserScan, PointCloud2
import laser_geometry.laser_geometry as lg
import sensor_msgs.point_cloud2 as pc2
import time


def get_z(T_cam_world, T_world_pc, K):
    R = T_cam_world[:3,:3]
    t = T_cam_world[:3,3]
    proj_mat = np.dot(K, np.hstack((R, t[:,np.newaxis])))
    xyz_hom = np.hstack((T_world_pc, np.ones((T_world_pc.shape[0], 1))))
    xy_hom = np.dot(proj_mat, xyz_hom.T).T
    z = xy_hom[:, -1]
    z = np.asarray(z).squeeze()
    return z

def extract(point):
    return [point[0], point[1], point[2]]

def callback(scan,image):

    lp = lg.LaserProjection()
    cloud = lp.projectLaser(scan)
    # print cloud
    points = pc2.read_points(cloud)
    print type(points)
    objPoints = np.array(map(extract, points))
    print objPoints



if __name__ == '__main__':
    rospy.init_node('scan_subscriber', anonymous=True)

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
    print("rot_mat =")
    print(rot_mat)
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

    pub = rospy.Publisher("/reprojection", Image, queue_size=1)
    scan_sub = message_filters.Subscriber("/front_laser_scan", LaserScan, queue_size=1)
    image_sub = message_filters.Subscriber("/camea/rgb/image_raw", Image, queue_size=1)
    ts = message_filters.ApproximateTimeSynchronizer([scan_sub,image_sub], 10, 1)
    ts.registerCallback(callback)

    rospy.spin()
