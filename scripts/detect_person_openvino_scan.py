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
from object_msgs.msg import ObjectsInBoxes
from geometry_msgs.msg import Pose, PoseArray

HSV_MIN = np.array([0, 30, 60])
HSV_MAX = np.array([20, 150, 255])

FACE_HEIGHT = 20

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

def callback(scan,image, vino):
    rospy.loginfo("image timestamp: %d ns" % image.header.stamp.to_nsec())
    rospy.loginfo("scan timestamp: %d ns" % scan.header.stamp.to_nsec())
    diff = abs(image.header.stamp.to_nsec() - scan.header.stamp.to_nsec())
    rospy.loginfo("diff: %d ns" % diff)

    human = False

    img = bridge.imgmsg_to_cv2(image)
    h, w, _ = img.shape
    cloud = lp.projectLaser(scan)
    points = pc2.read_points(cloud)
    objPoints = np.array(map(extract, points))
    Z = get_z(q, objPoints, K)
    objPoints = objPoints[Z > 0]
    if lens == 'pinhole':
        img_points, _ = cv2.projectPoints(objPoints, rvec, tvec, K, D)
    elif lens == 'fisheye':
        objPoints = np.reshape(objPoints, (1,objPoints.shape[0],objPoints.shape[1]))
        img_points, _ = cv2.fisheye.projectPoints(objPoints, rvec, tvec, K, D)

    max_probability = 0.0
    max_human_pos = []
    # rect_msg = ObjectsInBoxes()

    for box in vino.objects_vector:
        if box.object.object_name == "person":
            if box.object.probability > max_probability:
                x_min = box.roi.x_offset
                y_min = box.roi.y_offset
                x_max = box.roi.x_offset + box.roi.width
                y_max = box.roi.y_offset + box.roi.height
                if x_max < w and y_max < h:
                    # rect_msg.objects_vector = box
                    max_probability = box.object.probability
                    max_human_pos = [x_min,y_min,x_max,y_max]

    if len(max_human_pos) == 4:
        kimg = img[y_min : (y_max-y_min)/4 + y_min, x_min : x_max]
        h, w, c = img.shape
        h2, w2, c2 = kimg.shape

        scale = float(640)/float(h2)
        resize_img = cv2.resize(kimg, dsize=None, fx=scale, fy=scale, interpolation = cv2.INTER_AREA)

        hsv_image = cv2.cvtColor(resize_img, cv2.COLOR_BGR2HSV)
        hsv_mask = cv2.inRange(hsv_image, HSV_MIN, HSV_MAX)
        label = cv2.connectedComponentsWithStats(hsv_mask)
        n = label[0] - 1
        data = np.delete(label[2], 0, 0)
        center = np.delete(label[3], 0, 0)
        print resize_img.shape
        for i in range(n):
            x0 = data[i][0]
            y0 = data[i][1]
            x1 = data[i][0] + data[i][2]
            y1 = data[i][1] + data[i][3]
            if (y1-y0) > 200:
                cv2.rectangle(resize_img, (x0, y0), (x1, y1), (0, 0, 255))
                human = True
                cv2.putText(resize_img, "ID: " +str(i + 1), (x1 - 20, y1 + 15), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 255))
                cv2.putText(resize_img, "S: " +str(data[i][4]), (x1 - 20, y1 + 30), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 255))

                cv2.putText(resize_img, "X: " + str(int(center[i][0])), (x1 - 30, y1 + 15), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 255))
                cv2.putText(resize_img, "Y: " + str(int(center[i][1])), (x1 - 30, y1 + 30), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 255))
        resize_msg = bridge.cv2_to_imgmsg((resize_img), encoding="bgr8")
        resize_msg.header.stamp = rospy.Time.now()
        vino_image_pub.publish(resize_msg)




        index_x_min = 1200
        index_x_max = 0
        posearray_msg = PoseArray()

        cv2.rectangle(img,(int(round(max_human_pos[0])),int(round(max_human_pos[1]))),(int(round(max_human_pos[2])),int(round(max_human_pos[3]))),(200,0,20),3)

        img_points = np.squeeze(img_points)
        for i in range(len(img_points)):
            if int(round(img_points[i][0])) > int(round(max_human_pos[0])) and int(round(img_points[i][0])) < int(round(max_human_pos[2])):
                if int(round(img_points[i][1])) > int(round(max_human_pos[1])) and int(round(img_points[i][1])) < int(round(max_human_pos[3])):
                    if int(round(img_points[i][0])) < index_x_min:
                        index_x_min = i
                    if int(round(img_points[i][0])) > index_x_max:
                        index_x_max = i
                    try:
                        cv2.circle(img, (int(round(img_points[i][0])),int(round(img_points[i][1]))), laser_point_radius, (0,255,0), 1)
                    except OverflowError:
                        continue
        # print ("MIN: ", index_x_min , ",MAX: ", index_x_max)

        detect_objPoints = objPoints[0][index_x_min:index_x_max]
        print detect_objPoints
        if len(detect_objPoints) == 0:
            print "vacant"
        else:
            count = 0

            for p in detect_objPoints:
                count += 1
                pose_msg = Pose()
                print "-----------------------------"

                pose_msg.position.x = p[0]
                pose_msg.position.y = p[1]
                pose_msg.orientation.w = 1
                if human:
                    pose_msg.position.z = 1
                posearray_msg.poses.append(pose_msg)

        posearray_msg.header.frame_id = "/base_scan"
        posearray_msg.header.stamp = rospy.Time.now()

        # rect_msg.header.stamp = rospy.Time.now()

        pose_pub.publish(posearray_msg)
        # rect_pub.publish(rect_msg)


    else:
        pass
    pub.publish(bridge.cv2_to_imgmsg(img))

rospy.init_node('reprojection')
scan_topic = rospy.get_param("~scan_topic")
image_topic = rospy.get_param("~image_topic")
calib_file = rospy.get_param("~calib_file")
config_file = rospy.get_param("~config_file")
laser_point_radius = rospy.get_param("~laser_point_radius")
time_diff = rospy.get_param("~time_diff")
bridge = CvBridge()
lp = lg.LaserProjection()

with open(calib_file, 'r') as f:
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
print(q)
tvec = q[:3,3]
rot_mat = q[:3,:3]
rvec, _ = cv2.Rodrigues(rot_mat)

with open(config_file, 'r') as f:
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

pub = rospy.Publisher("/human_head_image", Image, queue_size=1)
vino_image_pub = rospy.Publisher("/resize/camera/color/image_raw", Image, queue_size=1)
pose_pub = rospy.Publisher("/scan/detect_leg_person", PoseArray, queue_size=1)
rect_pub = rospy.Publisher("/openvino/person_rect", ObjectsInBoxes, queue_size=1)
scan_sub = message_filters.Subscriber(scan_topic, LaserScan, queue_size=1)
image_sub = message_filters.Subscriber(image_topic, Image, queue_size=1)
vino_sub = message_filters.Subscriber("/ros_openvino_toolkit/detected_objects", ObjectsInBoxes, queue_size=1)
ts = message_filters.ApproximateTimeSynchronizer([scan_sub,image_sub,vino_sub], 10, time_diff)
ts.registerCallback(callback)

rospy.spin()
