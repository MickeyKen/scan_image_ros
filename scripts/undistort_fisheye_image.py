#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2
import yaml

u = None
v = None

x = None
y = None

alpha = 1.0
beta = 0

clicked = False
fig = None

sub_img = np.array([])


def image_cb(msg):
    sub_img_distorted = bridge.imgmsg_to_cv2(msg)
    sub_img_distorted = cv2.cvtColor(sub_img_distorted, cv2.COLOR_BGR2RGB)
    sub_img = cv2.fisheye.undistortImage(sub_img_distorted, K, D, Knew = K)
    cv2.imshow('screen', sub_img)
    msg = bridge.cv2_to_imgmsg(sub_img, encoding="bgr8")
    image_pub.publish(msg)
    cv2.waitKey(1)

rospy.init_node('imshow_distort_fisheye_camera')
image_pub = rospy.Publisher('camera/color/image_raw', Image, queue_size=10)
image_topic = rospy.get_param('~image_topic')
config_file = rospy.get_param('~config_file')

img_sub = rospy.Subscriber(image_topic, Image, image_cb)
bridge = CvBridge()

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
if lens not in ['pinhole', 'fisheye']:
    print('Invalid lens, using pinhole as default.')
    lens = 'pinhole'
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
