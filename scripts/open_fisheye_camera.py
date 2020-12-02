#!/usr/bin/env python
import os
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import datetime
import yaml
import numpy as np

def operator():
    rospy.init_node('open_fisheye_camera', anonymous=True)

    image_topic = rospy.get_param('~image_topic')
    config_file = rospy.get_param('~config_file')

    pub = rospy.Publisher(image_topic, Image, queue_size=10)

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
    # make bridge
    bridge = CvBridge()

    rate = rospy.Rate(30) # 1hz
    vcap = cv2.VideoCapture(0)
    vcap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('Y','U','Y','V'))
    vcap.set(cv2.CAP_PROP_FRAME_WIDTH, 1600)
    vcap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1200)
    vcap.set(cv2.CAP_PROP_FPS, 30)
    while not rospy.is_shutdown():
        ret,frame = vcap.read()
        # msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        undistort_img = cv2.fisheye.undistortImage(frame, K, D, Knew = K)
        msg = bridge.cv2_to_imgmsg(undistort_img, encoding="bgr8")
        # msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        msg.header.stamp = rospy.Time.now()
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        operator()
    except rospy.ROSInterruptException:
        pass
