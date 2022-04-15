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

    cv2.imshow('image_left', img_left)
    cv2.imshow('image_right', img_right)

    cv2.setMouseCallback('image_left', click_event_left)
    cv2.setMouseCallback('image_right', click_event_right)

    while cv2.waitKey(1) != 27:
        continue
    cv2.destroyAllWindows()

    print("callback impair complete")


def click_event_left(event, x, y, flags, params):
    if not is_cameras_initialized:
        return
    global RL_transform
    click_event(event, x, y, "fleft")


def click_event_right(event, x, y, flags, params):
    if not is_cameras_initialized:
        return
    global RL_transform
    click_event(event, x, y, "fright")


def click_event(event, x, y, name):
    global RL_transform
    # checking for left mouse clicks
    if not is_cameras_initialized:
        return
    if event == cv2.EVENT_LBUTTONDOWN:
        color = tuple(random.randint(0, 255) for _ in range(3))
        if name == "fleft":
            camera1 = CL
            camera2 = CR
            transform = LR_transform
            im1 = img_left
            im2 = img_right
        elif name == "fright":
            camera1 = CR
            camera2 = CL
            transform = RL_transform
            im1 = img_right
            im2 = img_left
        else:
            exit(-1)

        td_ray = camera1.projectPixelTo3dRay((x, y))
        td_ray_pt = geometry_msgs.msg.PointStamped()
        td_ray_pt.point.x = td_ray[0]
        td_ray_pt.point.y = td_ray[1]
        td_ray_pt.point.z = td_ray[2]

        td_ray_pt.header.frame_id = "uav1/basler_{}_optical".format(name[1:])
        td_ray_pt.header.stamp = rospy.Time.now()

        ray_transformed = tf2_geometry_msgs.do_transform_point(td_ray_pt, transform)
        epiline = (ray_transformed.point.x,
                   ray_transformed.point.y,
                   ray_transformed.point.z)

        origin = transform.transform.translation
        origin = (origin.x,
                  origin.y,
                  origin.z)

        pt: List = list(map(int, camera2.project3dToPixel(epiline)))
        o: List = list(map(int, camera2.project3dToPixel(origin)))
        pt.append(1)
        o.append(1)

        r = cross(pt, o)
        print(r)
        x0, y0 = map(int, [0, -r[2] / r[1]])
        x1, y1 = map(int, [1920, -(r[2] + r[0] * 1920) / r[1]])

        cv2.circle(im1, (x, y), 2, color, 2)
        # cv2.circle(img_left, tuple(pt[:-1]), 3, color, 3)

        cv2.line(im2, (x0, y0), (x1, y1), color, 2)
        if name == "fleft":
            cv2.imshow('image_left', im1)
            cv2.imshow('image_right', im2)
        elif name == "fright":
            cv2.imshow('image_left', im2)
            cv2.imshow('image_right', im1)


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
