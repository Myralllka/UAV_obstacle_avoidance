#!/bin/env python3

import cv2
import geometry_msgs.msg
import rospy
import random
import numpy as np
import image_geometry
import tf2_ros
import tf2_geometry_msgs
from mrs_msgs.msg._ImageLabeled import ImageLabeled
from mrs_msgs.msg._ImageLabeledArray import ImageLabeledArray

from cv_bridge import CvBridge, CvBridgeError
from typing import Tuple, List
from sensor_msgs.msg import Image, CameraInfo

img_right, img_left, RL_transform, LR_transform = None, None, None, None
bridge = CvBridge()

flag = False

CL = image_geometry.PinholeCameraModel()
CR = image_geometry.PinholeCameraModel()

is_cameras_initialized = False

is_left_init = False
is_right_init = False


def cross(a: Tuple, b: Tuple):
    c = [a[1] * b[2] - a[2] * b[1],
         a[2] * b[0] - a[0] * b[2],
         a[0] * b[1] - a[1] * b[0]]

    return c


def callback_multiview(data: ImageLabeledArray):
    if not is_cameras_initialized:
        return
    global img_left, img_right

    for img in data.imgs_labeled:
        img: ImageLabeled
        if img.label == "fright":
            img_right = bridge.imgmsg_to_cv2(img.img, 'bgr8')
        if img.label == "fleft":
            img_left = bridge.imgmsg_to_cv2(img.img, 'bgr8')

    img_left = cv2.cvtColor(img_left, cv2.COLOR_BGRA2GRAY)
    img_right = cv2.cvtColor(img_right, cv2.COLOR_BGRA2GRAY)

    mask_left = np.zeros(img_left.shape, dtype=np.uint8)
    mask_right = np.zeros(img_right.shape, dtype=np.uint8)

    mask_right[:, :img_right.shape[0] // 2] = 255
    mask_left[:, img_right.shape[0] // 2 + 500:] = 255

    orb = cv2.ORB_create()
    kp1, des1 = orb.detectAndCompute(img_left, mask_left)
    kp2, des2 = orb.detectAndCompute(img_right, mask_right)
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    matches = bf.match(des1, des2)
    matches = sorted(matches, key=lambda x: x.distance)
    img3 = cv2.drawMatches(img_left,
                           kp1,
                           img_right,
                           kp2,
                           matches[:20],
                           None,
                           flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
    img3 = cv2.resize(img3, (1920, 750), interpolation=cv2.INTER_AREA)
    cv2.imshow('image_matches', img3)
    # cv2.imshow('image_right', img_right)

    while cv2.waitKey(1) != 27:
        continue
    cv2.destroyAllWindows()

    print("callback impair complete")


def callback_cam_info_left(l_msg: CameraInfo):
    global is_left_init, is_right_init, is_cameras_initialized, CL
    if is_left_init:
        return

    if is_right_init:
        is_cameras_initialized = True

    CL.fromCameraInfo(l_msg)
    is_left_init = True


def callback_cam_info_right(l_msg: CameraInfo):
    global is_left_init, is_right_init, is_cameras_initialized, CR
    if is_right_init:
        return

    if is_left_init:
        is_cameras_initialized = True

    CR.fromCameraInfo(l_msg)
    is_right_init = True


def listener():
    global RL_transform, LR_transform, is_cameras_initialized

    rospy.init_node("listener", anonymous=True)

    rospy.Subscriber("/uav1/basler_stereo_driver/multiview_labeled", ImageLabeledArray, callback_multiview)
    rospy.Subscriber("/uav1/basler_left/camera_info", CameraInfo, callback_cam_info_left)
    rospy.Subscriber("/uav1/basler_right/camera_info", CameraInfo, callback_cam_info_right)

    tf_buffer = tf2_ros.Buffer(rospy.Duration(100))  # tf buffer length
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    while not is_cameras_initialized:
        continue
    # target, source, time, timeout
    RL_transform = tf_buffer.lookup_transform("uav1/basler_left_optical",
                                              "uav1/basler_right_optical",
                                              rospy.Time(),
                                              rospy.Duration(100))

    LR_transform = tf_buffer.lookup_transform("uav1/basler_right_optical",
                                              "uav1/basler_left_optical",
                                              rospy.Time(),
                                              rospy.Duration(100))
    rospy.spin()


if __name__ == "__main__":
    listener()
