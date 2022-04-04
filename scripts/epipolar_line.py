#!/bin/env python3

import cv2
import geometry_msgs.msg
import rospy
import random
import numpy as np
import image_geometry
import tf2_ros
import tf2_geometry_msgs

from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image, CameraInfo

img_right, img_left, RL_transform = None, None, None
bridge = CvBridge()

flag = False

CL = image_geometry.PinholeCameraModel()
CR = image_geometry.PinholeCameraModel()

is_cameras_initialized = False

is_left_init = False
is_right_init = False


def callback_right(data: Image):
    global img_right
    global flag
    img_right = bridge.imgmsg_to_cv2(data, 'bgr8')

    cv2.imshow('image right', img_right)
    cv2.imshow('image left', img_left)
    flag = True
    cv2.setMouseCallback('image right', click_event)
    while cv2.waitKey(1) != 27:
        continue
    cv2.destroyAllWindows()
    flag = False
    print("callback_right complete")


def callback_left(data: Image):
    if not is_cameras_initialized:
        return
    global img_left
    if flag:
        return
    img_left = bridge.imgmsg_to_cv2(data, 'bgr8')

    print("callback_left complete")


def click_event(event, x, y, flags, params):
    global RL_transform
    # checking for left mouse clicks
    if not is_cameras_initialized:
        return
    if event == cv2.EVENT_LBUTTONDOWN:
        print(x, ' ', y)
        color = tuple(random.randint(0, 255) for _ in range(3))

        td_ray = CR.projectPixelTo3dRay((x, y))
        td_ray_pt = geometry_msgs.msg.PointStamped()
        td_ray_pt.point.x = td_ray[0]
        td_ray_pt.point.y = td_ray[1]
        td_ray_pt.point.z = td_ray[2]

        td_ray_pt.header.frame_id = "uav1/basler_right_optical"
        td_ray_pt.header.stamp = rospy.Time.now()

        ray_transformed = tf2_geometry_msgs.do_transform_point(td_ray_pt, RL_transform)
        epiline = (ray_transformed.point.x,
                   ray_transformed.point.y,
                   ray_transformed.point.z)

        origin = RL_transform.transform.translation
        origin = (origin.x,
                  origin.y,
                  origin.z)

        pt = tuple(map(int, CL.project3dToPixel(epiline)))
        o = tuple(map(int, CL.project3dToPixel(origin)))

        cv2.circle(img_right, (x, y), 3, color, 3)
        cv2.circle(img_left, pt, 3, color, 3)
        cv2.line(img_left, pt, o, color, 2)
        cv2.imshow('image right', img_right)
        cv2.imshow('image left', img_left)


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
    global RL_transform
    rospy.init_node("listener", anonymous=True)
    rospy.Subscriber("/uav1/basler_right/image_rect", Image, callback_right)
    rospy.Subscriber("/uav1/basler_left/image_rect", Image, callback_left)
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
    rospy.spin()


if __name__ == "__main__":
    listener()
